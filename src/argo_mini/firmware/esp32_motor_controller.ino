#include "driver/dac.h"

// ── Hall sensors ───────────────────────────────────────────────────────────
#define HALL_LA 32
#define HALL_LB 34
#define HALL_LC 35
#define HALL_RA 13
#define HALL_RB 14
#define HALL_RC 27

// ── DAC channels ───────────────────────────────────────────────────────────
#define THROTTLE_L DAC_CHANNEL_1   // GPIO25
#define THROTTLE_R DAC_CHANNEL_2   // GPIO26

// ── Direction pins (active-LOW: LOW = reverse) ─────────────────────────────
#define DIR_L 2
#define DIR_R 4

// ── DAC range ───────────────────────────────────────────────────────────────
#define DAC_MIN       100
#define DAC_MAX       120
#define POLE_PAIRS     15
#define TICKS_PER_REV (POLE_PAIRS * 6)   // 90 ticks / rev

// ── Ramp ────────────────────────────────────────────────────────────────────
#define RAMP_STEP  1
#define RAMP_MS   20

// ── Direction flags ─────────────────────────────────────────────────────────
// Updated IMMEDIATELY when a "V" command is received (not in the ramp loop).
// This eliminates the 20 ms race window that previously caused reverse ticks
// to be signed as forward during the ramp's first cycle.
volatile bool leftReverse  = false;
volatile bool rightReverse = false;

// ── Odometry counters ───────────────────────────────────────────────────────
volatile long     leftTicks   = 0;
volatile long     rightTicks  = 0;
volatile uint32_t leftPulses  = 0;
volatile uint32_t rightPulses = 0;

void IRAM_ATTR leftISR() {
  if (leftReverse) leftTicks--; else leftTicks++;
  leftPulses++;
}
void IRAM_ATTR rightISR() {
  if (rightReverse) rightTicks--; else rightTicks++;
  rightPulses++;
}

// ── Motor drive ─────────────────────────────────────────────────────────────
int targetL  = 0, targetR  = 0;
int currentL = 0, currentR = 0;

void setDAC(int l, int r) {
  digitalWrite(DIR_L, (l < 0) ? LOW : HIGH);
  digitalWrite(DIR_R, (r < 0) ? LOW : HIGH);
  dac_output_voltage(THROTTLE_L, (l == 0) ? 0 : constrain(abs(l), DAC_MIN, DAC_MAX));
  dac_output_voltage(THROTTLE_R, (r == 0) ? 0 : constrain(abs(r), DAC_MIN, DAC_MAX));
}

int rampToward(int current, int target) {
  if (target == 0)                 return 0;
  if (current == 0 && target > 0)  return  DAC_MIN;
  if (current == 0 && target < 0)  return -DAC_MIN;
  if (current < target)            return min(current + RAMP_STEP, target);
  if (current > target)            return max(current - RAMP_STEP, target);
  return current;
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(HALL_LA, INPUT); pinMode(HALL_LB, INPUT); pinMode(HALL_LC, INPUT);
  pinMode(HALL_RA, INPUT); pinMode(HALL_RB, INPUT); pinMode(HALL_RC, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_LA), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_LB), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_LC), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RA), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RB), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RC), rightISR, CHANGE);

  pinMode(DIR_L, OUTPUT); digitalWrite(DIR_L, HIGH);
  pinMode(DIR_R, OUTPUT); digitalWrite(DIR_R, HIGH);

  dac_output_enable(THROTTLE_L);
  dac_output_enable(THROTTLE_R);
  setDAC(0, 0);

  Serial.println("ARGO MINI READY");
}

// ── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // ── Serial command parser ─────────────────────────────────────────────────
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("V ")) {
      int spaceIdx = line.indexOf(' ', 2);
      if (spaceIdx > 0) {
        targetL = line.substring(2, spaceIdx).toInt();
        targetR = line.substring(spaceIdx + 1).toInt();

        // ── KEY FIX: update direction flags immediately on command arrival ──
        // The old code updated these inside setDAC() which is called from the
        // ramp loop (every RAMP_MS = 20 ms).  ISR ticks fire instantly on
        // every Hall edge, so there was a 20 ms window where ticks fired with
        // the wrong direction flag.  Setting flags here closes that window.
        leftReverse  = (targetL < 0);
        rightReverse = (targetR < 0);
        // Also flip the DIR pin immediately so motor controller sees the new
        // direction before the ramp starts increasing throttle.
        digitalWrite(DIR_L, leftReverse  ? LOW : HIGH);
        digitalWrite(DIR_R, rightReverse ? LOW : HIGH);
      }
    } else if (line == "S") {
      targetL = 0; targetR = 0;
      currentL = 0; currentR = 0;
      leftReverse  = false;
      rightReverse = false;
      setDAC(0, 0);
      Serial.println("STOP");
    }
  }

  // ── Smooth ramp ───────────────────────────────────────────────────────────
  static uint32_t lastRamp = 0;
  if (now - lastRamp >= RAMP_MS) {
    currentL = rampToward(currentL, targetL);
    currentR = rampToward(currentR, targetR);
    setDAC(currentL, currentR);
    lastRamp = now;
  }

  // ── Odometry at 20 Hz ─────────────────────────────────────────────────────
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 50) {
    float elapsed = (now - lastPrint) / 1000.0f;

    noInterrupts();
    uint32_t lp = leftPulses;  leftPulses  = 0;
    uint32_t rp = rightPulses; rightPulses = 0;
    long lt = leftTicks;
    long rt = rightTicks;
    interrupts();

    Serial.printf("O %ld %ld\n", lt, rt);

    if (now % 500 < 50) {
      float lRPM = (lp / elapsed) * 60.0f / TICKS_PER_REV;
      float rRPM = (rp / elapsed) * 60.0f / TICKS_PER_REV;
      Serial.printf("R %.1f %.1f\n", lRPM, rRPM);
    }

    lastPrint = now;
  }
}
