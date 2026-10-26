#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FRAM_I2C.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define FRAM_I2C_ADDR 0x50

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

const uint8_t BROWNOUT_PIN = 2;   // LM393 OUT -> D2 / INT0
const uint8_t LED_PIN = 8;
const uint8_t RESET_BUTTON_PIN = 7;

const uint32_t COMMIT_VALUE = 0xBEEF1234UL;
const uint32_t MAX_REASONABLE_COUNTER = 1000000UL;

const uint16_t SLOT_SIZE = 24;
const uint16_t SLOT_A_BASE = 0;
const uint16_t SLOT_B_BASE = SLOT_SIZE;

const unsigned long SAVE_COOLDOWN_MS = 800;
const unsigned long COUNT_INTERVAL_MS = 500;
const unsigned long DISPLAY_INTERVAL_MS = 500;
const unsigned long DEBOUNCE_MS = 40;
const unsigned long STATUS_PRINT_MS = 1000;

// Debug controls
const bool DEBUG_I2C_SCAN = true;
const bool DEBUG_SLOT_DUMPS = true;
const bool DEBUG_HEARTBEAT = true;

uint8_t currentState = 0;
uint32_t counter = 0;
bool brownoutLatched = false;
bool oledReady = false;
bool framReady = false;

volatile bool brownoutTriggered = false;
volatile uint32_t brownoutInterruptCount = 0;

uint16_t nextSaveBase = SLOT_A_BASE;
uint32_t currentSequence = 0;
unsigned long previousSaveTimeUs = 0;
unsigned long lastSaveTimeMs = 0;
unsigned long lastCount = 0;
unsigned long lastDisplay = 0;
unsigned long lastStatusPrint = 0;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

void brownoutISR() {
  brownoutTriggered = true;
  brownoutInterruptCount++;
}

void scanI2CDevices() {
  Serial.println(F("[I2C] scanning..."));
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      Serial.print(F("[I2C] found 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      found++;
    }
  }

  if (found == 0) {
    Serial.println(F("[I2C] no devices found"));
  } else {
    Serial.print(F("[I2C] total="));
    Serial.println(found);
  }

  Serial.println(F("[I2C] expected OLED=0x3C FRAM=0x50"));
}

uint32_t read32(uint16_t addr) {
  uint32_t b0 = fram.read(addr + 0);
  uint32_t b1 = fram.read(addr + 1);
  uint32_t b2 = fram.read(addr + 2);
  uint32_t b3 = fram.read(addr + 3);
  return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

void printSlotContents(uint16_t base, const char* label) {
  if (!framReady) return;

  uint8_t state = fram.read(base + 0);
  uint32_t ctr = read32(base + 4);
  uint32_t ctrInv = read32(base + 8);
  uint32_t seq = read32(base + 12);
  uint32_t savedSaveUs = read32(base + 16);
  uint32_t commit = read32(base + 20);

  Serial.print(F("[FRAM] "));
  Serial.print(label);
  Serial.print(F(" slot="));
  Serial.print(base == SLOT_A_BASE ? 'A' : 'B');
  Serial.print(F(" state="));
  Serial.print(state);
  Serial.print(F(" ctr="));
  Serial.print(ctr);
  Serial.print(F(" invOK="));
  Serial.print((ctrInv == ~ctr) ? F("YES") : F("NO"));
  Serial.print(F(" seq="));
  Serial.print(seq);
  Serial.print(F(" prevUs="));
  Serial.print(savedSaveUs);
  Serial.print(F(" commitOK="));
  Serial.println(commit == COMMIT_VALUE ? F("YES") : F("NO"));
}

void updateOutput() {
  digitalWrite(LED_PIN, currentState ? HIGH : LOW);
}

void updateDisplay(bool brownoutNow) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println(F("TRUE BROWNOUT"));
  display.print(F("COUNT: "));   display.println(counter);
  display.print(F("STATE: "));   display.println(currentState);
  display.print(F("CMP: "));     display.println(brownoutNow ? F("LOW") : F("HIGH"));
  display.print(F("LATCH: "));   display.println(brownoutLatched ? F("YES") : F("NO"));
  display.print(F("SAVE us: ")); display.println(previousSaveTimeUs);
  display.print(F("FRAM: "));    display.println(framReady ? F("OK") : F("FAIL"));
  display.print(F("OLED: "));    display.println(oledReady ? F("OK") : F("FAIL"));

  display.display();
}

void resetCounterNow() {
  Serial.println(F("[BTN] reset pressed"));
  counter = 0;
  currentState = 0;
  brownoutLatched = false;
  brownoutTriggered = false;
  updateOutput();
  updateDisplay(digitalRead(BROWNOUT_PIN) == LOW);
}

void handleResetButton() {
  bool reading = digitalRead(RESET_BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) {
        resetCounterNow();
      }
    }
  }

  lastButtonReading = reading;
}

bool slotValid(uint16_t base, uint8_t &stateOut, uint32_t &counterOut, uint32_t &seqOut, uint32_t &savedSaveUsOut) {
  if (!framReady) return false;

  uint8_t state = fram.read(base + 0);
  uint32_t ctr = read32(base + 4);
  uint32_t ctrInv = read32(base + 8);
  uint32_t seq = read32(base + 12);
  uint32_t savedSaveUs = read32(base + 16);
  uint32_t commit = read32(base + 20);

  if (commit != COMMIT_VALUE) return false;
  if (ctrInv != ~ctr) return false;
  if (state > 1) return false;
  if (ctr > MAX_REASONABLE_COUNTER) return false;

  stateOut = state;
  counterOut = ctr;
  seqOut = seq;
  savedSaveUsOut = savedSaveUs;
  return true;
}

bool saveToSlot(uint16_t base, uint8_t stateVal, uint32_t counterVal, uint32_t seqVal, uint32_t prevSaveUsVal) {
  uint32_t counterInv = ~counterVal;

  Wire.beginTransmission(FRAM_I2C_ADDR);
  Wire.write((uint8_t)(base >> 8));
  Wire.write((uint8_t)(base & 0xFF));

  Wire.write(stateVal);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);

  Wire.write((counterVal >> 24) & 0xFF);
  Wire.write((counterVal >> 16) & 0xFF);
  Wire.write((counterVal >> 8) & 0xFF);
  Wire.write(counterVal & 0xFF);

  Wire.write((counterInv >> 24) & 0xFF);
  Wire.write((counterInv >> 16) & 0xFF);
  Wire.write((counterInv >> 8) & 0xFF);
  Wire.write(counterInv & 0xFF);

  Wire.write((seqVal >> 24) & 0xFF);
  Wire.write((seqVal >> 16) & 0xFF);
  Wire.write((seqVal >> 8) & 0xFF);
  Wire.write(seqVal & 0xFF);

  Wire.write((prevSaveUsVal >> 24) & 0xFF);
  Wire.write((prevSaveUsVal >> 16) & 0xFF);
  Wire.write((prevSaveUsVal >> 8) & 0xFF);
  Wire.write(prevSaveUsVal & 0xFF);

  Wire.write((COMMIT_VALUE >> 24) & 0xFF);
  Wire.write((COMMIT_VALUE >> 16) & 0xFF);
  Wire.write((COMMIT_VALUE >> 8) & 0xFF);
  Wire.write(COMMIT_VALUE & 0xFF);

  return (Wire.endTransmission() == 0);
}

bool saveStateFast() {
  if (!framReady) {
    Serial.println(F("[SAVE] skipped, FRAM not ready"));
    return false;
  }

  uint32_t newSeq = currentSequence + 1;
  uint16_t slotJustUsed = nextSaveBase;

  Serial.print(F("[SAVE] writing SLOT "));
  Serial.print(slotJustUsed == SLOT_A_BASE ? 'A' : 'B');
  Serial.print(F(" seq="));
  Serial.print(newSeq);
  Serial.print(F(" state="));
  Serial.print(currentState);
  Serial.print(F(" counter="));
  Serial.println(counter);

  bool ok = saveToSlot(slotJustUsed, currentState, counter, newSeq, previousSaveTimeUs);

  Serial.print(F("[SAVE] result="));
  Serial.println(ok ? F("OK") : F("FAIL"));

  if (ok) {
    currentSequence = newSeq;
    nextSaveBase = (nextSaveBase == SLOT_A_BASE) ? SLOT_B_BASE : SLOT_A_BASE;
  }

  if (DEBUG_SLOT_DUMPS) {
    printSlotContents(slotJustUsed, "WRITTEN");
  }

  return ok;
}

void restoreState() {
  if (!framReady) {
    Serial.println(F("[FRAM] restore skipped, FRAM not ready"));
    currentState = 0;
    counter = 0;
    currentSequence = 0;
    previousSaveTimeUs = 0;
    nextSaveBase = SLOT_A_BASE;
    return;
  }

  uint8_t stateA, stateB;
  uint32_t counterA, counterB, seqA, seqB;
  uint32_t saveUsA, saveUsB;

  bool validA = slotValid(SLOT_A_BASE, stateA, counterA, seqA, saveUsA);
  bool validB = slotValid(SLOT_B_BASE, stateB, counterB, seqB, saveUsB);

  Serial.println(F("[FRAM] restore check"));

  if (DEBUG_SLOT_DUMPS) {
    printSlotContents(SLOT_A_BASE, "RAW");
    printSlotContents(SLOT_B_BASE, "RAW");
  }

  Serial.print(F("[FRAM] valid A="));
  Serial.print(validA ? F("YES") : F("NO"));
  Serial.print(F(" B="));
  Serial.println(validB ? F("YES") : F("NO"));

  if (!validA && !validB) {
    Serial.println(F("[FRAM] no valid saved state, using defaults"));
    currentState = 0;
    counter = 0;
    currentSequence = 0;
    previousSaveTimeUs = 0;
    nextSaveBase = SLOT_A_BASE;
    return;
  }

  if (validA && (!validB || seqA >= seqB)) {
    currentState = stateA;
    counter = counterA;
    currentSequence = seqA;
    previousSaveTimeUs = saveUsA;
    nextSaveBase = SLOT_B_BASE;
    Serial.println(F("[FRAM] restored from SLOT A"));
  } else {
    currentState = stateB;
    counter = counterB;
    currentSequence = seqB;
    previousSaveTimeUs = saveUsB;
    nextSaveBase = SLOT_A_BASE;
    Serial.println(F("[FRAM] restored from SLOT B"));
  }

  Serial.print(F("[FRAM] state="));
  Serial.print(currentState);
  Serial.print(F(" counter="));
  Serial.print(counter);
  Serial.print(F(" seq="));
  Serial.print(currentSequence);
  Serial.print(F(" prevSaveUs="));
  Serial.println(previousSaveTimeUs);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BROWNOUT_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("==== BOOT ===="));

  Wire.begin();
  Wire.setClock(400000);

  if (DEBUG_I2C_SCAN) {
    scanI2CDevices();
  }

  Serial.print(F("[FRAM] begin... "));
  framReady = fram.begin();
  if (framReady) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("FAIL"));
  }

  restoreState();
  updateOutput();

  Serial.print(F("[OLED] begin... "));
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oledReady = true;
    display.setRotation(2);
    display.clearDisplay();
    display.display();
    Serial.println(F("OK"));
  } else {
    oledReady = false;
    Serial.println(F("FAIL"));
  }

  attachInterrupt(digitalPinToInterrupt(BROWNOUT_PIN), brownoutISR, FALLING);
  Serial.println(F("[IRQ] brownout interrupt attached"));

  Serial.print(F("[BOOT] CMP="));
  Serial.println(digitalRead(BROWNOUT_PIN) == LOW ? F("LOW") : F("HIGH"));

  updateDisplay(digitalRead(BROWNOUT_PIN) == LOW);

  Serial.println(F("==== RUN ===="));
}

void loop() {
  bool brownoutNow = (digitalRead(BROWNOUT_PIN) == LOW);

  handleResetButton();

  if (DEBUG_HEARTBEAT && millis() - lastStatusPrint >= STATUS_PRINT_MS) {
    lastStatusPrint = millis();

    Serial.print(F("[RUN] cmp="));
    Serial.print(brownoutNow ? F("LOW") : F("HIGH"));
    Serial.print(F(" latched="));
    Serial.print(brownoutLatched ? F("Y") : F("N"));
    Serial.print(F(" trig="));
    Serial.print(brownoutTriggered ? F("Y") : F("N"));
    Serial.print(F(" irq="));
    Serial.print(brownoutInterruptCount);
    Serial.print(F(" ctr="));
    Serial.print(counter);
    Serial.print(F(" st="));
    Serial.print(currentState);
    Serial.print(F(" next="));
    Serial.print(nextSaveBase == SLOT_A_BASE ? F("A") : F("B"));
    Serial.print(F(" fram="));
    Serial.print(framReady ? F("OK") : F("FAIL"));
    Serial.print(F(" oled="));
    Serial.println(oledReady ? F("OK") : F("FAIL"));
  }

  bool cooldownExpired = ((millis() - lastSaveTimeMs) > SAVE_COOLDOWN_MS);

  if (!brownoutLatched && brownoutTriggered && cooldownExpired) {
    Serial.println(F("[EVENT] brownout accepted"));

    brownoutLatched = true;
    brownoutTriggered = false;
    lastSaveTimeMs = millis();

    unsigned long saveStart = micros();
    bool saveOk = saveStateFast();
    unsigned long saveEnd = micros();
    previousSaveTimeUs = saveEnd - saveStart;

    Serial.print(F("[EVENT] saveTimeUs="));
    Serial.print(previousSaveTimeUs);
    Serial.print(F(" result="));
    Serial.println(saveOk ? F("OK") : F("FAIL"));
  }

  if (brownoutLatched && !brownoutNow) {
    brownoutLatched = false;
    Serial.println(F("[EVENT] brownout rearmed"));
  }

  if (!brownoutLatched && millis() - lastCount >= COUNT_INTERVAL_MS) {
    lastCount = millis();
    counter++;

    if ((counter % 4) == 0) {
      currentState = !currentState;
      updateOutput();

      Serial.print(F("[RUN] toggled state -> "));
      Serial.println(currentState);
    }
  }

  if (millis() - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = millis();
    updateDisplay(brownoutNow);
  }
}