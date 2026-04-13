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

const int BROWNOUT_PIN = A0;
const int LED_PIN = 8;
const int RESET_BUTTON_PIN = 2;

const int BROWNOUT_THRESHOLD_LOW  = 700;
const int BROWNOUT_THRESHOLD_HIGH = 900;

const uint32_t COMMIT_VALUE = 0xBEEF1234;
const uint32_t MAX_REASONABLE_COUNTER = 1000000UL;


const uint16_t SLOT_SIZE = 20;
const uint16_t SLOT_A_BASE = 0;
const uint16_t SLOT_B_BASE = SLOT_SIZE;

uint8_t currentState = 0;
uint32_t counter = 0;
bool brownoutLatched = false;
bool oledReady = false;

unsigned long lastPrint = 0;
unsigned long lastCount = 0;
unsigned long lastDisplay = 0;
unsigned long lastSaveTimeUs = 0;
unsigned long lastSaveTimeMs = 0;

const unsigned long SAVE_COOLDOWN_MS = 800;

// reset button debounce
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_MS = 40;


// latest restored sequence
uint32_t currentSequence = 0;

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
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("COUNT: ");
  display.println(counter);

  display.print("STATE: ");
  display.println(currentState);

  display.print("SEQ: ");
  display.println(currentSequence);

  display.print("A0: ");
  display.println(brownoutValue);

  display.print("BRNOUT: ");
  display.println(brownoutLatched ? "YES" : "NO");

  display.print("SAVE us: ");
  display.println(lastSaveTimeUs);

  display.display();
}

void resetCounterNow() {
  counter = 0;
  currentState = 0;
  brownoutLatched = false;
  updateOutput();
  Serial.println("COUNTER RESET TO 0");
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
        updateDisplay(analogRead(BROWNOUT_PIN));
      }
    }
  }

  lastButtonReading = reading;
}

bool slotValid(uint16_t base, uint8_t &stateOut, uint32_t &counterOut, uint32_t &seqOut) {
  uint8_t state = fram.read(base + 0);
  uint32_t ctr = read32(base + 4);
  uint32_t ctrInv = read32(base + 8);
  uint32_t seq = read32(base + 12);
  uint32_t commit = read32(base + 16);

  if (commit != COMMIT_VALUE) return false;
  if (ctrInv != ~ctr) return false;
  if (state > 1) return false;
  if (ctr > MAX_REASONABLE_COUNTER) return false;

  stateOut = state;
  counterOut = ctr;
  seqOut = seq;
  return true;
}

// Write to chosen slot, commit marker LAST
bool saveToSlot(uint16_t base, uint8_t stateVal, uint32_t counterVal, uint32_t seqVal) {
  uint32_t counterInv = ~counterVal;

  Wire.setClock(400000);

  Wire.beginTransmission(FRAM_I2C_ADDR);
  Wire.write((uint8_t)(base >> 8));
  Wire.write((uint8_t)(base & 0xFF));

  // state
  Wire.write(stateVal);

  // filler
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);

  // counter
  Wire.write((counterVal >> 24) & 0xFF);
  Wire.write((counterVal >> 16) & 0xFF);
  Wire.write((counterVal >> 8) & 0xFF);
  Wire.write(counterVal & 0xFF);

  // counter inverse
  Wire.write((counterInv >> 24) & 0xFF);
  Wire.write((counterInv >> 16) & 0xFF);
  Wire.write((counterInv >> 8) & 0xFF);
  Wire.write(counterInv & 0xFF);

  // sequence
  Wire.write((seqVal >> 24) & 0xFF);
  Wire.write((seqVal >> 16) & 0xFF);
  Wire.write((seqVal >> 8) & 0xFF);
  Wire.write(seqVal & 0xFF);

  // commit LAST
  Wire.write((COMMIT_VALUE >> 24) & 0xFF);
  Wire.write((COMMIT_VALUE >> 16) & 0xFF);
  Wire.write((COMMIT_VALUE >> 8) & 0xFF);
  Wire.write(COMMIT_VALUE & 0xFF);

  uint8_t result = Wire.endTransmission();

  Wire.setClock(100000);

  return (result == 0);
}

bool saveStateDualSlot() {
  uint8_t stateA, stateB;
  uint32_t counterA, counterB, seqA, seqB;
  bool validA = slotValid(SLOT_A_BASE, stateA, counterA, seqA);
  bool validB = slotValid(SLOT_B_BASE, stateB, counterB, seqB);

  uint16_t targetBase;

  if (!validA && !validB) {
    targetBase = SLOT_A_BASE;
    currentSequence = 0;
  } else if (validA && !validB) {
    targetBase = SLOT_B_BASE;
    currentSequence = seqA;
  } else if (!validA && validB) {
    targetBase = SLOT_A_BASE;
    currentSequence = seqB;
  } else {
    if (seqA >= seqB) {
      targetBase = SLOT_B_BASE;
      currentSequence = seqA;
    } else {
      targetBase = SLOT_A_BASE;
      currentSequence = seqB;
    }
  }

  uint32_t newSeq = currentSequence + 1;
  bool ok = saveToSlot(targetBase, currentState, counter, newSeq);

  if (ok) {
    currentSequence = newSeq;
  }

  return ok;
}

void restoreState() {
  uint8_t stateA, stateB;
  uint32_t counterA, counterB, seqA, seqB;

  bool validA = slotValid(SLOT_A_BASE, stateA, counterA, seqA);
  bool validB = slotValid(SLOT_B_BASE, stateB, counterB, seqB);

  if (!validA && !validB) {
    currentState = 0;
    counter = 0;
    currentSequence = 0;
    Serial.println("NO VALID SAVED STATE");
    return;
  }

  if (validA && (!validB || seqA >= seqB)) {
    currentState = stateA;
    counter = counterA;
    currentSequence = seqA;
    Serial.println("STATE RESTORED FROM SLOT A");
  } else {
    currentState = stateB;
    counter = counterB;
    currentSequence = seqB;
    Serial.println("STATE RESTORED FROM SLOT B");
  }

  Serial.print("Restored state = ");
  Serial.println(currentState);
  Serial.print("Restored counter = ");
  Serial.println(counter);
  Serial.print("Restored seq = ");
  Serial.println(currentSequence);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("BOOTING MAIN SKETCH...");

  Wire.begin();
  Wire.setClock(100000);
  Serial.println("I2C started");

  if (!fram.begin()) {
    Serial.println("ERROR: FRAM not found");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(150);
    }
  }
  Serial.println("FRAM detected");

  delay(200);
  Serial.print("Startup A0 estimate: ");
  Serial.println(analogRead(A0));
  delay(100);

  restoreState();
  updateOutput();

  delay(100);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("ERROR: OLED not found");
    oledReady = false;
  } else {
    Serial.println("OLED detected");
    oledReady = true;
    display.clearDisplay();
    display.display();
  }

  int a0 = analogRead(BROWNOUT_PIN);

  updateDisplay(a0);

  Serial.println("Setup complete");
}

void loop() {
  int brownoutValue = analogRead(BROWNOUT_PIN);

  handleResetButton();

  bool lowNow = (brownoutValue < BROWNOUT_THRESHOLD_LOW);
  bool cooldownExpired = ((millis() - lastSaveTimeMs) > SAVE_COOLDOWN_MS);

  if (!brownoutLatched && lowNow && cooldownExpired) {
    brownoutLatched = true;
    lastSaveTimeMs = millis();

    Serial.println("BROWNOUT DETECTED -> SAVING");
    Serial.print("Saving state=");
    Serial.print(currentState);
    Serial.print(" counter=");
    Serial.println(counter);

    unsigned long saveStart = micros();
    bool saveOk = saveStateDualSlot();
    unsigned long saveEnd = micros();

    lastSaveTimeUs = saveEnd - saveStart;

    Serial.print("Save time (us) = ");
    Serial.println(lastSaveTimeUs);
    Serial.println(saveOk ? "SAVE DONE" : "SAVE FAILED");

    updateDisplay(brownoutValue);
  }

  if (brownoutLatched && brownoutValue > BROWNOUT_THRESHOLD_HIGH) {
    brownoutLatched = false;
    Serial.println("RECOVERED -> REARMED");
  }

  const unsigned long COUNT_INTERVAL_MS = 200;

  if (!brownoutLatched && millis() - lastCount >= COUNT_INTERVAL_MS) {
    lastCount = millis();
    counter++;

    if ((counter % 4) == 0) {
      currentState = !currentState;
      updateOutput();
      Serial.print("TOGGLED STATE TO ");
      Serial.println(currentState);
    }
  }

  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("A0=");
    Serial.print(brownoutValue);
    Serial.print(" state=");
    Serial.print(currentState);
    Serial.print(" counter=");
    Serial.print(counter);
    Serial.print(" latched=");
    Serial.println(brownoutLatched ? "YES" : "NO");
  }

  if (millis() - lastDisplay >= 150) {
    lastDisplay = millis();
    updateDisplay(brownoutValue);
  }
}