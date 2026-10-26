#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FRAM_I2C.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define FRAM_I2C_ADDR 0x50

#define DEBUG_SERIAL 0

#if DEBUG_SERIAL
  #define DBG_BEGIN(x) Serial.begin(x)
  #define DBG_PRINT(x) Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_BEGIN(x)
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
#endif

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

// ATmega328P assignments
const uint8_t BROWNOUT_PIN = A0;   // physical pin 23
const uint8_t LED_PIN = 8;         // physical pin 14
const uint8_t RESET_BUTTON_PIN = 7;// physical pin 13

// Tune these on hardware
const int BROWNOUT_THRESHOLD_LOW  = 380;
const int BROWNOUT_THRESHOLD_HIGH = 430;

// FRAM layout
const uint32_t COMMIT_VALUE = 0xBEEF1234UL;
const uint32_t MAX_REASONABLE_COUNTER = 1000000UL;
const uint16_t SLOT_SIZE = 24;
const uint16_t SLOT_A_BASE = 0;
const uint16_t SLOT_B_BASE = SLOT_SIZE;

// Timing
const unsigned long SAVE_COOLDOWN_MS = 800;
const unsigned long COUNT_INTERVAL_MS = 500;
const unsigned long DISPLAY_INTERVAL_MS = 350;
const unsigned long DEBOUNCE_MS = 40;

uint8_t currentState = 0;
uint32_t counter = 0;
bool brownoutLatched = false;
bool oledReady = false;

uint16_t nextSaveBase = SLOT_A_BASE;
uint32_t currentSequence = 0;
unsigned long previousSaveTimeUs = 0;
unsigned long lastSaveTimeMs = 0;
unsigned long lastCount = 0;
unsigned long lastDisplay = 0;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

uint32_t read32(uint16_t addr) {
  uint32_t b0 = fram.read(addr + 0);
  uint32_t b1 = fram.read(addr + 1);
  uint32_t b2 = fram.read(addr + 2);
  uint32_t b3 = fram.read(addr + 3);
  return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

void updateOutput() {
  digitalWrite(LED_PIN, currentState ? HIGH : LOW);
}

void updateDisplay(int brownoutValue) {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.println("OBSERVER DEMO");
  display.print("COUNT: "); display.println(counter);
  display.print("STATE: "); display.println(currentState);
  display.print("ADC: "); display.println(brownoutValue);
  display.print("BRNOUT: "); display.println(brownoutLatched ? "YES" : "NO");
  display.print("SAVE us: "); display.println(previousSaveTimeUs);

  display.display();
}

void resetCounterNow() {
  counter = 0;
  currentState = 0;
  brownoutLatched = false;
  updateOutput();
}

void handleResetButton(int brownoutValue) {
  bool reading = digitalRead(RESET_BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) {
        resetCounterNow();
        updateDisplay(brownoutValue);
      }
    }
  }

  lastButtonReading = reading;
}

bool slotValid(uint16_t base, uint8_t &stateOut, uint32_t &counterOut, uint32_t &seqOut, uint32_t &savedSaveUsOut) {
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
  uint32_t newSeq = currentSequence + 1;
  bool ok = saveToSlot(nextSaveBase, currentState, counter, newSeq, previousSaveTimeUs);

  if (ok) {
    currentSequence = newSeq;
    nextSaveBase = (nextSaveBase == SLOT_A_BASE) ? SLOT_B_BASE : SLOT_A_BASE;
  }
  return ok;
}

void restoreState() {
  uint8_t stateA, stateB;
  uint32_t counterA, counterB, seqA, seqB;
  uint32_t saveUsA, saveUsB;

  bool validA = slotValid(SLOT_A_BASE, stateA, counterA, seqA, saveUsA);
  bool validB = slotValid(SLOT_B_BASE, stateB, counterB, seqB, saveUsB);

  if (!validA && !validB) {
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
  } else {
    currentState = stateB;
    counter = counterB;
    currentSequence = seqB;
    previousSaveTimeUs = saveUsB;
    nextSaveBase = SLOT_A_BASE;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  DBG_BEGIN(115200);

  Wire.begin();
  Wire.setClock(400000);

  if (!fram.begin()) {
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(150);
    }
  }

  restoreState();
  updateOutput();

  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oledReady = true;
    display.setRotation(2);
    display.clearDisplay();
    display.display();
  }

  updateDisplay(analogRead(BROWNOUT_PIN));
}

void loop() {
  int brownoutValue = analogRead(BROWNOUT_PIN);

  handleResetButton(brownoutValue);

  bool lowNow = (brownoutValue < BROWNOUT_THRESHOLD_LOW);
  bool cooldownExpired = ((millis() - lastSaveTimeMs) > SAVE_COOLDOWN_MS);

  if (!brownoutLatched && lowNow && cooldownExpired) {
    brownoutLatched = true;
    lastSaveTimeMs = millis();

    unsigned long saveStart = micros();
    bool saveOk = saveStateFast();
    unsigned long saveEnd = micros();

    previousSaveTimeUs = saveEnd - saveStart;

    if (!saveOk) {
      // observer demo can survive here anyway
    }
  }

  if (brownoutLatched && brownoutValue > BROWNOUT_THRESHOLD_HIGH) {
    brownoutLatched = false;
  }

  if (!brownoutLatched && millis() - lastCount >= COUNT_INTERVAL_MS) {
    lastCount = millis();
    counter++;

    if ((counter % 4) == 0) {
      currentState = !currentState;
      updateOutput();
    }
  }

  if (millis() - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = millis();
    updateDisplay(brownoutValue);
  }
}