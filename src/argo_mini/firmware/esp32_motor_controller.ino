#include "driver/dac.h"

// ── Hall sensors ───────────────────────────────────────────────────────────
#define HALL_LA 32
#define HALL_LB 34
#define HALL_LC 35
#define HALL_RA 13
#define HALL_RB 14
#define HALL_RC 27

// ── DAC channels ───────────────────────────────────────────────────────────
// GPIO25 = DAC_CHANNEL_1 → LEFT  motor throttle
// GPIO26 = DAC_CHANNEL_2 → RIGHT motor throttle
#define THROTTLE_L DAC_CHANNEL_1
#define THROTTLE_R DAC_CHANNEL_2

// ── Direction pins ─────────────────────────────────────────────────────────
// Active-LOW: pulling to GND activates reverse
#define DIR_L 2
#define DIR_R 4

// ── DAC range ───────────────────────────────────────────────────────────────
#define DAC_MIN       100
#define DAC_MAX       120
#define POLE_PAIRS     15
#define TICKS_PER_REV (POLE_PAIRS * 6)

// ── Ramp ────────────────────────────────────────────────────────────────────
#define RAMP_STEP  1
#define RAMP_MS   20

// ── Hall state-machine direction lookup ────────────────────────────────────
//
// BLDC hub motors with 3 Hall sensors produce a deterministic 6-step
// commutation sequence.  Forward and reverse traverse it in opposite order:
//
//   Forward: 5 → 4 → 6 → 2 → 3 → 1 → 5 → ...
//   Reverse: 5 → 1 → 3 → 2 → 6 → 4 → 5 → ...
//
// HALL_DIR[prev][curr] encodes the direction of that state transition:
//   +1 = forward tick,  -1 = reverse tick,  0 = invalid / noise (skip)
//
// This replaces the old command-flag approach, which had a 20 ms race
// window (ramp period) during which ticks fired with the wrong sign.
// The Hall state machine determines direction from what the motor *actually
// does*, not from what was commanded — no race, no timing dependency.
//
// ── Calibration note ───────────────────────────────────────────────────────
// The sequence above (5→4→6→2→3→1) is standard for most hub motors, but
// polarity depends on motor winding and Hall placement.  After flashing:
//   1. Drive slowly FORWARD and read `ros2 topic echo /odom | grep linear`
//   2. linear.x must be positive.  If it is negative, set HALL_FLIP -1 below.
//
#define HALL_FLIP  1     // set to -1 if forward odom reads negative after flash

// clang-format off
//                        curr→  0   1   2   3   4   5   6   7
const int8_t HALL_DIR[8][8] = {
  /* prev=0 invalid */ {  0,  0,  0,  0,  0,  0,  0,  0 },
  /* prev=1          */ {  0,  0,  0, -1,  0,  1,  0,  0 },  // fwd→5  rev→3
  /* prev=2          */ {  0,  0,  0,  1,  0,  0, -1,  0 },  // fwd→3  rev→6
  /* prev=3          */ {  0,  1, -1,  0,  0,  0,  0,  0 },  // fwd→1  rev→2
  /* prev=4          */ {  0,  0,  0,  0,  0, -1,  1,  0 },  // fwd→6  rev→5
  /* prev=5          */ {  0, -1,  0,  0,  1,  0,  0,  0 },  // fwd→4  rev→1
  /* prev=6          */ {  0,  0,  1,  0, -1,  0,  0,  0 },  // fwd→2  rev→4
  /* prev=7 invalid  */ {  0,  0,  0,  0,  0,  0,  0,  0 },
};
// clang-format on

// ── Odometry counters ───────────────────────────────────────────────────────
volatile long     leftTicks    = 0;
volatile long     rightTicks   = 0;
volatile uint32_t leftPulses   = 0;   // magnitude only, for RPM
volatile uint32_t rightPulses  = 0;

volatile int      lastLeftState  = 0;
volatile int      lastRightState = 0;

// ── ISRs ────────────────────────────────────────────────────────────────────
// Called on any CHANGE of any Hall pin.
// Reads all 3 pins to get current state, compares with previous state,
// and uses HALL_DIR to determine actual rotation direction.
// No command flags involved — purely sensor-derived.

void IRAM_ATTR leftISR() {
  int state = (digitalRead(HALL_LA) << 2) |
              (digitalRead(HALL_LB) << 1) |
               digitalRead(HALL_LC);

  if (lastLeftState != 0) {
    int8_t dir = HALL_DIR[lastLeftState][state];
    if (dir != 0) {
      leftTicks += (long)(dir * HALL_FLIP);
      leftPulses++;
    }
    // dir == 0 means invalid transition (glitch/noise) — ignore
  }
  lastLeftState = state;
}

void IRAM_ATTR rightISR() {
  int state = (digitalRead(HALL_RA) << 2) |
              (digitalRead(HALL_RB) << 1) |
               digitalRead(HALL_RC);

  if (lastRightState != 0) {
    int8_t dir = HALL_DIR[lastRightState][state];
    if (dir != 0) {
      rightTicks += (long)(dir * HALL_FLIP);
      rightPulses++;
    }
  }
  lastRightState = state;
}

// ── Motor drive ─────────────────────────────────────────────────────────────
int targetL  = 0, targetR  = 0;
int currentL = 0, currentR = 0;

// setDAC: controls motor direction pin and throttle DAC.
// Direction flags are now ONLY for the hardware DIR pin — they are no longer
// read inside ISRs.
void setDAC(int l, int r) {
  bool lRev = (l < 0);
  bool rRev = (r < 0);

  digitalWrite(DIR_L, lRev ? LOW : HIGH);
  digitalWrite(DIR_R, rRev ? LOW : HIGH);

  dac_output_voltage(THROTTLE_L, (l == 0) ? 0 : constrain(abs(l), DAC_MIN, DAC_MAX));
  dac_output_voltage(THROTTLE_R, (r == 0) ? 0 : constrain(abs(r), DAC_MIN, DAC_MAX));
}

int rampToward(int current, int target) {
  if (target == 0)                  return 0;
  if (current == 0 && target > 0)   return  DAC_MIN;
  if (current == 0 && target < 0)   return -DAC_MIN;
  if (current < target)             return min(current + RAMP_STEP, target);
  if (current > target)             return max(current - RAMP_STEP, target);
  return current;
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);

  // Hall sensor inputs — no pull required (external pull-ups on PCB)
  pinMode(HALL_LA, INPUT); pinMode(HALL_LB, INPUT); pinMode(HALL_LC, INPUT);
  pinMode(HALL_RA, INPUT); pinMode(HALL_RB, INPUT); pinMode(HALL_RC, INPUT);

  // Read initial Hall states so the first ISR call has a valid previous state
  lastLeftState  = (digitalRead(HALL_LA) << 2) |
                   (digitalRead(HALL_LB) << 1) |
                    digitalRead(HALL_LC);
  lastRightState = (digitalRead(HALL_RA) << 2) |
                   (digitalRead(HALL_RB) << 1) |
                    digitalRead(HALL_RC);

  // Attach interrupts — CHANGE mode fires on every Hall edge
  attachInterrupt(digitalPinToInterrupt(HALL_LA), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_LB), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_LC), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RA), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RB), rightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RC), rightISR, CHANGE);

  // Direction pins — default forward
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
      }
    } else if (line == "S") {
      targetL = 0; targetR = 0;
      currentL = 0; currentR = 0;
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
  // Signed cumulative tick counts — positive = forward, negative = reverse.
  // Direction determined by Hall state machine, not by command.
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

    // RPM debug line every ~500 ms
    if (now % 500 < 50) {
      float lRPM = (lp / elapsed) * 60.0f / TICKS_PER_REV;
      float rRPM = (rp / elapsed) * 60.0f / TICKS_PER_REV;
      Serial.printf("R %.1f %.1f\n", lRPM, rRPM);
    }

    lastPrint = now;
  }
}
