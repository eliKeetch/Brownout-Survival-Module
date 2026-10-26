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

//  Pico pin assignments 
const int BROWNOUT_PIN = A0;        // GP26_A0
const int LED_PIN = 15;             // GP15
const int RESET_BUTTON_PIN = 14;    // GP14

// Pico ADC THREASHOLDS
// Tune these for modifying brownout level
const int BROWNOUT_THRESHOLD_LOW  = 850;
const int BROWNOUT_THRESHOLD_HIGH = 900;

// FRAM
const uint32_t COMMIT_VALUE = 0xBEEF1234;
const uint32_t MAX_REASONABLE_COUNTER = 1000000UL;
const uint16_t SLOT_SIZE = 24; // increase the memory
const uint16_t SLOT_A_BASE = 0;
const uint16_t SLOT_B_BASE = SLOT_SIZE;


//Variables
uint8_t currentState = 0;
uint32_t counter = 0;
bool brownoutLatched = false;
bool oledReady = false;

uint16_t nextSaveBase = SLOT_A_BASE;
unsigned previousSaveTimeUs = 0;

unsigned long lastPrint = 0;
unsigned long lastCount = 0;
unsigned long lastDisplay = 0;
unsigned long lastSaveTimeUs = 0;
unsigned long lastSaveTimeMs = 0;

const unsigned long SAVE_COOLDOWN_MS = 800; // to prevent back to back saving loops on brownouts close to the threashold

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

  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("PROTECTED DEVICE");
  display.print("COUNT: ");
  display.println(counter);
 

  display.print("STATE: ");
  display.println(currentState);

  display.print("ADC: ");
  display.println(brownoutValue);

  display.print("BRNOUT: ");
  display.println(brownoutLatched ? "YES" : "NO");

  display.print("LAST SAVE us: ");
display.println(previousSaveTimeUs);

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

// Write to chosen slot, commit marker LAST
bool saveToSlot(uint16_t base, uint8_t stateVal, uint32_t counterVal, uint32_t seqVal, uint32_t prevSaveUsVal) {
  uint32_t counterInv = ~counterVal;

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

  // previous save time
  Wire.write((prevSaveUsVal >> 24) & 0xFF);
  Wire.write((prevSaveUsVal >> 16) & 0xFF);
  Wire.write((prevSaveUsVal >> 8) & 0xFF);
  Wire.write(prevSaveUsVal & 0xFF);

  // commit LAST
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
    Serial.println("NO VALID SAVED STATE");
    return;
  }

  if (validA && (!validB || seqA >= seqB)) {
    currentState = stateA;
    counter = counterA;
    currentSequence = seqA;
    previousSaveTimeUs = saveUsA;
    nextSaveBase = SLOT_B_BASE;
    Serial.println("STATE RESTORED FROM SLOT A");
  } else {
    currentState = stateB;
    counter = counterB;
    currentSequence = seqB;
    previousSaveTimeUs = saveUsB;
    nextSaveBase = SLOT_A_BASE;
    Serial.println("STATE RESTORED FROM SLOT B");
  }

  Serial.print("Restored state = ");
  Serial.println(currentState);
  Serial.print("Restored counter = ");
  Serial.println(counter);
  Serial.print("Restored seq = ");
  Serial.println(currentSequence);
  Serial.print("Restored previous save us = ");
  Serial.println(previousSaveTimeUs);
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
  Wire.setClock(400000);
  Serial.println("I2C started");
delay()
  if (!fram.begin()) {
    Serial.println("ERROR: FRAM not found");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(150);
    }
  }
  Serial.println("FRAM found");

  delay(200);
  Serial.print("Startup ADC estimate: ");
  Serial.println(analogRead(BROWNOUT_PIN));
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
    display.setRotation(2);
    display.clearDisplay();
    display.display();
  }

  int brownoutValue = analogRead(BROWNOUT_PIN);

  updateDisplay(brownoutValue);

  Serial.println("Setup done");
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
   bool saveOk = saveStateFast();
unsigned long saveEnd = micros();

lastSaveTimeUs = saveEnd - saveStart;
previousSaveTimeUs = lastSaveTimeUs;

    Serial.print("Save time (us) = ");
    Serial.println(lastSaveTimeUs);
    Serial.println(saveOk ? "SAVE DONE" : "SAVE FAILED");

    updateDisplay(brownoutValue);
  }

  if (brownoutLatched && brownoutValue > BROWNOUT_THRESHOLD_HIGH) {
    brownoutLatched = false;
    Serial.println("RECOVERED -> REARMED");
  }

  const unsigned long COUNT_INTERVAL_MS = 500;

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
    Serial.print("ADC=");
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