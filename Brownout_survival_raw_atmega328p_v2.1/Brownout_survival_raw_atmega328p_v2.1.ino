#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FRAM_I2C.h>

// ===== OLED =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledReady = false;

// ===== FRAM =====
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
bool framReady = false;

// ===== PINS =====
const int LED_PIN = 8;
const int BROWNOUT_PIN = 2;      // LM393 output
const int RESET_BUTTON_PIN = 7;  // button to GND, uses INPUT_PULLUP

// ===== STATE =====
volatile bool brownoutTriggered = false;

uint32_t counter = 0;
uint8_t currentState = 0;
uint32_t lastSaveTimeUs = 0;
uint32_t currentSequence = 0;

// ===== RUN MODES =====
enum SystemMode {
  MODE_RUNNING,
  MODE_BROWNOUT_HOLD
};

SystemMode mode = MODE_RUNNING;

// ===== TIMING =====
const unsigned long COUNTER_INTERVAL_MS = 500;     // count every 500 ms
const unsigned long STATE_TOGGLE_MS = 1000;        // toggle state/LED every 1000 ms
const unsigned long DISPLAY_UPDATE_MS = 100;       // OLED refresh while running
const unsigned long RECOVERY_STABLE_MS = 300;      // must stay good this long before restore

unsigned long recoveryStartMs = 0;

// ===== STORAGE =====
// Slot layout:
// [0]      state
// [4..7]   counter
// [8..11]  sequence
// [12..15] lastSaveTimeUs
// [16..19] commit marker

const uint16_t SLOT_A = 0;
const uint16_t SLOT_B = 32;
uint16_t nextSlot = SLOT_A;

// ===== RESET BUTTON DEBOUNCE =====
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceMs = 30;

// ===== ISR =====
void brownoutISR() {
  brownoutTriggered = true;
}

// ===== POWER STATUS HELPERS =====
// Assumes LM393 output goes LOW during brownout, HIGH when power is good
bool powerIsBad() {
  return digitalRead(BROWNOUT_PIN) == HIGH;
}

bool powerIsGood() {
  return digitalRead(BROWNOUT_PIN) == LOW;
}

// ===== FRAM HELPERS =====
void write32(uint16_t addr, uint32_t value) {
  fram.write(addr + 0, (uint8_t)((value >> 24) & 0xFF));
  fram.write(addr + 1, (uint8_t)((value >> 16) & 0xFF));
  fram.write(addr + 2, (uint8_t)((value >> 8) & 0xFF));
  fram.write(addr + 3, (uint8_t)(value & 0xFF));
}

uint32_t read32(uint16_t addr) {
  uint32_t v = 0;
  v |= ((uint32_t)fram.read(addr + 0)) << 24;
  v |= ((uint32_t)fram.read(addr + 1)) << 16;
  v |= ((uint32_t)fram.read(addr + 2)) << 8;
  v |= ((uint32_t)fram.read(addr + 3));
  return v;
}

// ===== SAVE =====
void saveFast() {
  if (!framReady) return;

  uint16_t base = nextSlot;
  uint32_t newSeq = currentSequence + 1;

  // Write the full slot once with a temporary save-time value
  unsigned long t0 = micros();

  fram.write(base + 0, currentState);
  write32(base + 4, counter);
  write32(base + 8, newSeq);
  write32(base + 12, 0);               // temporary placeholder
  write32(base + 16, 0xDEADBEEF);      // commit marker last in first pass

  unsigned long t1 = micros();
  lastSaveTimeUs = t1 - t0;

  // Store measured save time so it survives recovery/reset
  write32(base + 12, lastSaveTimeUs);

  currentSequence = newSeq;
  nextSlot = (nextSlot == SLOT_A) ? SLOT_B : SLOT_A;
}

// ===== SLOT CHECK =====
bool slotValid(uint16_t base, uint8_t &state, uint32_t &cnt, uint32_t &seq, uint32_t &saveUs) {
  uint32_t commit = read32(base + 16);
  if (commit != 0xDEADBEEF) return false;

  state = fram.read(base + 0);
  cnt = read32(base + 4);
  seq = read32(base + 8);
  saveUs = read32(base + 12);
  return true;
}

// ===== RESTORE =====
void restoreState() {
  uint8_t sA = 0, sB = 0;
  uint32_t cA = 0, cB = 0;
  uint32_t qA = 0, qB = 0;
  uint32_t tA = 0, tB = 0;

  bool validA = slotValid(SLOT_A, sA, cA, qA, tA);
  bool validB = slotValid(SLOT_B, sB, cB, qB, tB);

  if (validA && validB) {
    if (qA >= qB) {
      currentState = sA;
      counter = cA;
      currentSequence = qA;
      lastSaveTimeUs = tA;
      nextSlot = SLOT_B;
    } else {
      currentState = sB;
      counter = cB;
      currentSequence = qB;
      lastSaveTimeUs = tB;
      nextSlot = SLOT_A;
    }
  } else if (validA) {
    currentState = sA;
    counter = cA;
    currentSequence = qA;
    lastSaveTimeUs = tA;
    nextSlot = SLOT_B;
  } else if (validB) {
    currentState = sB;
    counter = cB;
    currentSequence = qB;
    lastSaveTimeUs = tB;
    nextSlot = SLOT_A;
  } else {
    currentState = 0;
    counter = 0;
    currentSequence = 0;
    lastSaveTimeUs = 0;
    nextSlot = SLOT_A;
  }
}

// ===== RESET COUNTER =====
void resetCounterNow() {
  counter = 0;
  currentState = 0;
  digitalWrite(LED_PIN, LOW);
}

void handleResetButton() {
  bool reading = digitalRead(RESET_BUTTON_PIN);
  unsigned long now = millis();

  if (reading != lastButtonReading) {
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) > debounceMs) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW && mode == MODE_RUNNING) {
        resetCounterNow();
      }
    }
  }

  lastButtonReading = reading;
}

// ===== OLED =====
void updateDisplay() {
  if (!oledReady) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Brownout Survival");

  display.setCursor(0, 12);
  display.print("Count: ");
  display.print(counter);

  display.setCursor(0, 22);
  display.print("State: ");
  display.print(currentState);

  display.setCursor(0, 32);
  display.print("Save us: ");
  display.print(lastSaveTimeUs);

  display.setCursor(0, 42);
  display.print("Seq: ");
  display.print(currentSequence);

  display.setCursor(0, 52);
  if (mode == MODE_RUNNING) {
    display.print("RUNNING");
  } else {
    display.print("BROWNOUT HOLD");
  }

  display.display();
}

// ===== SETUP =====
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BROWNOUT_PIN, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);   // fast I2C for FRAM/OLED

  framReady = fram.begin();

  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledReady) {
    display.clearDisplay();
    display.display();
    // display.setRotation(2); // uncomment if the OLED is upside down
  }

  restoreState();
  digitalWrite(LED_PIN, currentState);

  attachInterrupt(digitalPinToInterrupt(BROWNOUT_PIN), brownoutISR, FALLING);

  updateDisplay();
}

// ===== LOOP =====
void loop() {
  static unsigned long lastCounterUpdate = 0;
  static unsigned long lastStateToggle = 0;
  static unsigned long lastDisplayUpdate = 0;

  unsigned long nowMs = millis();

  // ----- RUNNING MODE -----
  if (mode == MODE_RUNNING) {
    handleResetButton();

    if (nowMs - lastCounterUpdate >= COUNTER_INTERVAL_MS) {
      lastCounterUpdate = nowMs;
      counter++;
    }

    if (nowMs - lastStateToggle >= STATE_TOGGLE_MS) {
      lastStateToggle = nowMs;
      currentState ^= 1;
      digitalWrite(LED_PIN, currentState);
    }

    // First brownout event: save once, then freeze everything
    if (brownoutTriggered || powerIsBad()) {
      brownoutTriggered = false;

      saveFast();

      mode = MODE_BROWNOUT_HOLD;
      recoveryStartMs = 0;

      // Keep LED in saved state while waiting for stable recovery
      digitalWrite(LED_PIN, currentState);

      updateDisplay();
    }
  }

  // ----- BROWNOUT HOLD MODE -----
  else if (mode == MODE_BROWNOUT_HOLD) {
    // No counting, no blinking, no re-saving during unstable power

    if (powerIsGood()) {
      if (recoveryStartMs == 0) {
        recoveryStartMs = nowMs;
      }

      if (nowMs - recoveryStartMs >= RECOVERY_STABLE_MS) {
        restoreState();
        digitalWrite(LED_PIN, currentState);

        brownoutTriggered = false;
        mode = MODE_RUNNING;

        // Reset timers so behavior resumes smoothly
        lastCounterUpdate = nowMs;
        lastStateToggle = nowMs;
        lastDisplayUpdate = nowMs;

        updateDisplay();
      }
    } else {
      recoveryStartMs = 0;
    }
  }

  // OLED refresh
    if (mode == MODE_RUNNING && (nowMs - lastDisplayUpdate >= DISPLAY_UPDATE_MS)) {
    lastDisplayUpdate = nowMs;
    updateDisplay();
  }
}