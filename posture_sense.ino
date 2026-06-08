// ============================================================
//  Posture Sense — Main Firmware
//  Board  : Heltec Wireless Tracker (ESP32-S3)
//  I2C    : SDA = GPIO 46 | SCL = GPIO 45
//  TCA    : 0x70  |  IMUs on channels 1, 2, 7
// ============================================================

#include <Wire.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>
#include <vector>

// ─────────────────────────────────────────
// Pin & bus constants
// ─────────────────────────────────────────
#define I2C_SDA       46
#define I2C_SCL       45
#define TCA_ADDR      0x70

#define CH_UPPER      1    // TCA channel → upper-back IMU
#define CH_MID        2    // TCA channel → mid-back IMU
#define CH_LOWER      7    // TCA channel → lower-back IMU

// ─────────────────────────────────────────
// Timing
// ─────────────────────────────────────────
#define SAMPLE_INTERVAL_US   250000UL   // 250 ms = 4 Hz
#define COUNTDOWN_MS         3000UL     // 3 s before calibration starts
#define CAL_SECONDS          5          // calibration collection window
#define CAL_SAMPLES          (CAL_SECONDS * 4)   // 20 samples @ 4 Hz
#define CAL_MIN_VALID        15         // accept cal if ≥15 of 20 samples valid
#define FROZEN_TIMEOUT_MS    300000UL   // 5 min — auto-resume if no ACK
#define ALERT_INTERVAL_MS    10000UL    // minimum gap between repeated alerts

// ─────────────────────────────────────────
// EMA & alert thresholds
// ─────────────────────────────────────────
#define EMA_ALPHA            0.05f
#define ALERT_THRESHOLD      0.65f      // EMA score that fires an alert
#define MAX_CONSECUTIVE_BAD  3          // invalid IMU readings before ERROR

// ─────────────────────────────────────────
// Posture detection thresholds (degrees)
// ─────────────────────────────────────────
// All values are research-informed starting points.
// Empirical calibration required after final IMU mounting.
//
//  Thoracic Slouch      : upper–mid pitch delta > 10°
//  Forward Flexion      : upper pitch alone     > 20°  (forward-head proxy)
//  Lateral Lean         : max roll spread       > 10°  (clinical scoliosis cutoff)
//  Lumbar Hyperlordosis : mid–lower pitch delta > 10°  (excessive inward arch)
//  Lumbar Flattening    : mid–lower pitch delta < −10° (posterior pelvic tilt)
#define THR_THORACIC_SLOUCH       10.0f
#define THR_FORWARD_FLEXION       20.0f
#define THR_LATERAL_LEAN          10.0f
#define THR_LUMBAR_HYPERLORDOSIS  10.0f
#define THR_LUMBAR_FLATTENING    -10.0f

// ─────────────────────────────────────────
// Posture flag bitmask (uint8_t)
// ─────────────────────────────────────────
#define POST_GOOD                 0x00
#define POST_THORACIC_SLOUCH      0x01
#define POST_FORWARD_FLEXION      0x02
#define POST_LATERAL_LEAN         0x04
#define POST_LUMBAR_HYPERLORDOSIS 0x08
#define POST_LUMBAR_FLATTENING    0x10

// ─────────────────────────────────────────
// Calibration stability
// ─────────────────────────────────────────
#define CAL_VARIANCE_LIMIT   2.0f   // degrees² — max acceptable variance

// ─────────────────────────────────────────
// Structs
// ─────────────────────────────────────────

// Raw orientation from one IMU after Madgwick fusion
struct IMUReading {
  float pitch, roll, yaw;
  bool  valid;  // false if IMU was not ready this tick
};

// All three IMU readings bundled together
struct SpineReading {
  IMUReading upper, mid, lower;
  bool allValid;  // true only when all three valid
};

// Calibrated baseline angles (recorded during CALIBRATING state)
struct Baseline {
  float pitchUpper, rollUpper;
  float pitchMid,   rollMid;
  float pitchLower, rollLower;
};

// ─────────────────────────────────────────
// State machine
// ─────────────────────────────────────────
enum SystemState {
  WAITING,           // idle — waiting for webapp to send START_CAL
  COUNTDOWN,         // 3-second countdown before collection begins
  CALIBRATING,       // collecting baseline samples
  WAITING_FOR_START, // calibration done — waiting for START_MON
  MONITORING,        // active posture detection
  FROZEN             // alert fired — waiting for ACK from webapp
};

// ─────────────────────────────────────────
// Globals
// ─────────────────────────────────────────
ICM_20948_I2C imuUpper, imuMid, imuLower;
Madgwick      filterUpper, filterMid, filterLower;

SystemState   currentState    = WAITING;
Baseline      baseline;
bool          baselineSet     = false;

// Per-posture EMA scores (one float per condition)
float emaScores[5] = {0, 0, 0, 0, 0};

// Consecutive invalid reading counter (shared across MONITORING + CALIBRATING)
int consecutiveBadReadings = 0;

// Timestamp of the last alert sent (prevents alert spam)
unsigned long lastAlertTime = 0;

// Timestamp when FROZEN state was entered (for timeout)
unsigned long frozenEnteredAt = 0;

// ─────────────────────────────────────────
// TCA helpers
// ─────────────────────────────────────────

// Select a single TCA channel; all others are disabled
void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Deselect all TCA channels (good practice between reads)
void tcaDisableAll() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ─────────────────────────────────────────
// IMU initialisation helper
// ─────────────────────────────────────────

// Tries address 0x68 first, then 0x69.
// Returns true if the IMU responds on either address.
bool initIMU(ICM_20948_I2C &imu, uint8_t channel, const char *name) {
  tcaSelect(channel);
  delay(50);

  imu.begin(Wire, 0);  // AD0 LOW → 0x68
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(name); Serial.println(" found at 0x68 ✅");
    return true;
  }

  imu.begin(Wire, 1);  // AD0 HIGH → 0x69
  if (imu.status == ICM_20948_Stat_Ok) {
    Serial.print(name); Serial.println(" found at 0x69 ✅");
    return true;
  }

  Serial.print(name); Serial.println(" NOT FOUND ❌");
  return false;
}

// ─────────────────────────────────────────
// IMU reading
// ─────────────────────────────────────────

// Read one IMU, run it through its Madgwick filter, return pitch/roll/yaw.
// Returns valid=false if the IMU has no new data this tick.
IMUReading readOneIMU(ICM_20948_I2C &imu, Madgwick &filter, uint8_t channel) {
  IMUReading result = {0, 0, 0, false};
  tcaSelect(channel);

  if (!imu.dataReady()) return result;

  imu.getAGMT();
  filter.update(
    imu.gyrX(), imu.gyrY(), imu.gyrZ(),
    imu.accX(), imu.accY(), imu.accZ(),
    imu.magX(), imu.magY(), imu.magZ()
  );

  result.pitch = filter.getPitch();
  result.roll  = filter.getRoll();
  result.yaw   = filter.getYaw();
  result.valid = true;

  tcaDisableAll();  // deselect after each read to prevent bus conflicts
  return result;
}

// Read all three IMUs.
// NOTE: reads are sequential (not async). If any IMU is not ready,
// that sample is marked invalid. allValid is only true when all three succeed.
// The Madgwick filter for each IMU is only updated when that IMU has fresh data,
// so filters may accumulate at slightly different rates over time — acceptable at 4 Hz.
SpineReading readAllIMUs() {
  SpineReading tr;
  tr.upper = readOneIMU(imuUpper, filterUpper, CH_UPPER);
  tr.mid   = readOneIMU(imuMid,   filterMid,   CH_MID);
  tr.lower = readOneIMU(imuLower, filterLower,  CH_LOWER);
  tr.allValid = tr.upper.valid && tr.mid.valid && tr.lower.valid;
  return tr;
}

// ─────────────────────────────────────────
// Calibration helpers
// ─────────────────────────────────────────

// Returns true if the variance of samples is within CAL_VARIANCE_LIMIT.
// Called after collection is complete; the size guard is a safety fallback.
bool isStable(std::vector<float> &samples) {
  if ((int)samples.size() < CAL_MIN_VALID) return false;
  float mean = 0;
  for (float s : samples) mean += s;
  mean /= samples.size();
  float variance = 0;
  for (float s : samples) variance += (s - mean) * (s - mean);
  variance /= samples.size();
  return variance < CAL_VARIANCE_LIMIT;
}

// ─────────────────────────────────────────
// EMA helper
// ─────────────────────────────────────────

// Updates and returns the new EMA score.
// isBad should be true (1.0) when that posture condition is detected.
float updateEMA(float currentScore, bool isBad) {
  float sample = isBad ? 1.0f : 0.0f;
  return (EMA_ALPHA * sample) + (1.0f - EMA_ALPHA) * currentScore;
}

// ─────────────────────────────────────────
// Posture detection
// ─────────────────────────────────────────

// Compares delta angles against research-derived thresholds.
// Returns a bitmask of all active posture flags (POST_* constants).
// All deltas are relative to the calibrated baseline.
//
// Axis orientation note: pitch/roll polarity depends on how each IMU
// is physically mounted. Validate empirically after mounting on spine.
uint8_t detectPostures(SpineReading &r) {
  uint8_t flags = POST_GOOD;

  // Subtract baseline offsets so all comparisons are relative to "good posture"
  float dPitchUpper = r.upper.pitch - baseline.pitchUpper;
  float dPitchMid   = r.mid.pitch   - baseline.pitchMid;
  float dPitchLower = r.lower.pitch - baseline.pitchLower;
  float dRollUpper  = r.upper.roll  - baseline.rollUpper;
  float dRollMid    = r.mid.roll    - baseline.rollMid;
  float dRollLower  = r.lower.roll  - baseline.rollLower;

  // Thoracic Slouch: upper–mid pitch delta > 10°
  // Excessive forward rounding of the upper back relative to mid back
  if ((dPitchUpper - dPitchMid) > THR_THORACIC_SLOUCH)
    flags |= POST_THORACIC_SLOUCH;

  // Forward Flexion (forward-head proxy): upper pitch alone > 20°
  // Upper thoracic region bending forward, associated with head-forward posture
  if (dPitchUpper > THR_FORWARD_FLEXION)
    flags |= POST_FORWARD_FLEXION;

  // Lateral Lean: max roll spread across all three sensors > 10°
  // Person leaning left or right; uses spread rather than absolute value
  float maxRoll = max({dRollUpper, dRollMid, dRollLower});
  float minRoll = min({dRollUpper, dRollMid, dRollLower});
  if ((maxRoll - minRoll) > THR_LATERAL_LEAN)
    flags |= POST_LATERAL_LEAN;

  // Lumbar Hyperlordosis: mid–lower pitch delta > 10°
  // Excessive inward arching of the lower back
  if ((dPitchMid - dPitchLower) > THR_LUMBAR_HYPERLORDOSIS)
    flags |= POST_LUMBAR_HYPERLORDOSIS;

  // Lumbar Flattening: mid–lower pitch delta < −10°
  // Reduction of the natural lumbar curve (posterior pelvic tilt)
  // Uses the same sensor pair as Hyperlordosis but in the opposite direction
  if ((dPitchMid - dPitchLower) < THR_LUMBAR_FLATTENING)
    flags |= POST_LUMBAR_FLATTENING;

  return flags;
}

// ─────────────────────────────────────────
// Invalid reading guard
// ─────────────────────────────────────────

// Call this whenever a SpineReading is invalid (not allValid).
// After MAX_CONSECUTIVE_BAD consecutive failures, sends an error signal
// and transitions to FROZEN so the webapp can alert the user.
void handleInvalidReading() {
  consecutiveBadReadings++;
  if (consecutiveBadReadings >= MAX_CONSECUTIVE_BAD) {
    Serial.println("ERROR:IMU_MALFUNCTION");
    consecutiveBadReadings = 0;
    currentState = FROZEN;
    frozenEnteredAt = millis();
  }
}

// ─────────────────────────────────────────
// STATE HANDLERS
// ─────────────────────────────────────────

// WAITING ────────────────────────────────
// Idle. Blocks on Serial waiting for "START_CAL" from webapp.
// Blocking is intentional here — timing precision is not needed in this state.
void handleWaiting() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "START_CAL") {
      currentState = COUNTDOWN;
    }
  }
}

// COUNTDOWN ──────────────────────────────
// Non-blocking 3-second countdown. Sends tick updates to webapp.
// Transitions to CALIBRATING when countdown expires.
void handleCountdown() {
  static unsigned long countdownStart = 0;
  static unsigned long lastTick       = 0;

  if (countdownStart == 0) {
    countdownStart = millis();
    lastTick       = millis();
    Serial.println("COUNTDOWN_START");
  }

  unsigned long elapsed = millis() - countdownStart;

  // Send a tick every second so the webapp can update its UI timer
  if (elapsed - lastTick >= 1000) {
    lastTick = elapsed;
    Serial.print("COUNTDOWN_TICK:");
    Serial.println(3 - (int)(elapsed / 1000));
  }

  if (elapsed >= COUNTDOWN_MS) {
    countdownStart = 0;  // reset for next calibration
    lastTick       = 0;
    currentState   = CALIBRATING;
  }
}

// CALIBRATING ────────────────────────────
// Collects CAL_SAMPLES readings at 4 Hz and computes baseline angles.
// Accepts the calibration if ≥ CAL_MIN_VALID samples are valid AND variance
// is within CAL_VARIANCE_LIMIT. Otherwise sends UNSTABLE_CAL and retries
// from COUNTDOWN. Consecutive invalid reading guard is active here too.
void handleCalibrating() {
  static std::vector<float> pU, rU, pM, rM, pL, rL;
  static int  collected = 0;
  static bool started   = false;

  if (!started) {
    pU.clear(); rU.clear();
    pM.clear(); rM.clear();
    pL.clear(); rL.clear();
    collected = 0;
    started   = true;
    Serial.println("CAL_STARTED");
  }

  SpineReading reading = readAllIMUs();

  if (reading.allValid) {
    consecutiveBadReadings = 0;  // reset on any good reading

    pU.push_back(reading.upper.pitch); rU.push_back(reading.upper.roll);
    pM.push_back(reading.mid.pitch);   rM.push_back(reading.mid.roll);
    pL.push_back(reading.lower.pitch); rL.push_back(reading.lower.roll);
    collected++;

    // Report progress every 10% so webapp can show a progress bar
    int pct = (collected * 100) / CAL_SAMPLES;
    static int lastReported = -1;
    if (pct / 10 != lastReported / 10) {
      lastReported = pct;
      Serial.print("CAL_PROGRESS:"); Serial.println(pct);
    }
  } else {
    // Invalid reading — check for IMU malfunction
    handleInvalidReading();
    // Do not increment collected; just skip this tick
  }

  // Check if we have enough samples to evaluate
  if (collected >= CAL_SAMPLES) {
    started = false;  // reset static state for next calibration

    // Require minimum valid samples AND stability on all six axes
    bool enoughSamples = (int)pU.size() >= CAL_MIN_VALID;
    bool stable = enoughSamples &&
                  isStable(pU) && isStable(rU) &&
                  isStable(pM) && isStable(rM) &&
                  isStable(pL) && isStable(rL);

    if (stable) {
      // Compute averages and store as baseline
      auto avg = [](std::vector<float> &v) {
        float s = 0; for (float x : v) s += x; return s / v.size();
      };
      baseline.pitchUpper = avg(pU); baseline.rollUpper = avg(rU);
      baseline.pitchMid   = avg(pM); baseline.rollMid   = avg(rM);
      baseline.pitchLower = avg(pL); baseline.rollLower = avg(rL);
      baselineSet = true;

      Serial.println("CAL_COMPLETE");
      currentState = WAITING_FOR_START;
    } else {
      // Not enough valid samples or too much movement during calibration
      Serial.println("UNSTABLE_CAL");
      currentState = COUNTDOWN;  // retry from countdown
    }
  }
}

// WAITING_FOR_START ──────────────────────
// Blocks on Serial waiting for "START_MON" from webapp.
// Blocking is intentional — timing precision not needed here.
void handleWaitingForStart() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "START_MON") {
      // Reset all EMA scores and counters before entering monitoring
      for (int i = 0; i < 5; i++) emaScores[i] = 0.0f;
      consecutiveBadReadings = 0;
      currentState = MONITORING;
    }
  }
}

// MONITORING ─────────────────────────────
// Core posture detection loop. Runs at 4 Hz via busy-compensation timing
// in the main loop(). Each tick: read IMUs → detect postures → update EMA
// → send alert if any EMA score crosses ALERT_THRESHOLD.
// Consecutive invalid reading guard is active here.
void handleMonitoring() {
  if (!baselineSet) {
    // Safety guard — should never reach here without a baseline
    Serial.println("ERROR:NO_BASELINE");
    currentState = WAITING;
    return;
  }

  SpineReading reading = readAllIMUs();

  if (!reading.allValid) {
    handleInvalidReading();
    return;
  }

  consecutiveBadReadings = 0;  // reset on any good reading

  // Detect which posture conditions are currently active
  uint8_t flags = detectPostures(reading);

  // Update EMA for each condition independently.
  // Each EMA score represents a smoothed probability (0–1) that
  // the condition is currently active. A score > ALERT_THRESHOLD triggers alert.
  emaScores[0] = updateEMA(emaScores[0], (flags & POST_THORACIC_SLOUCH)      != 0);
  emaScores[1] = updateEMA(emaScores[1], (flags & POST_FORWARD_FLEXION)      != 0);
  emaScores[2] = updateEMA(emaScores[2], (flags & POST_LATERAL_LEAN)         != 0);
  emaScores[3] = updateEMA(emaScores[3], (flags & POST_LUMBAR_HYPERLORDOSIS) != 0);
  emaScores[4] = updateEMA(emaScores[4], (flags & POST_LUMBAR_FLATTENING)    != 0);

  // Check if any condition has crossed the alert threshold.
  // ALERT_INTERVAL_MS prevents repeated alerts for the same ongoing issue.
  unsigned long now = millis();
  if (now - lastAlertTime >= ALERT_INTERVAL_MS) {
    if (emaScores[0] > ALERT_THRESHOLD) { Serial.println("ALERT:THORACIC_SLOUCH");      lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[1] > ALERT_THRESHOLD) { Serial.println("ALERT:FORWARD_FLEXION"); lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[2] > ALERT_THRESHOLD) { Serial.println("ALERT:LATERAL_LEAN");    lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[3] > ALERT_THRESHOLD) { Serial.println("ALERT:LUMBAR_HYPERLORDOSIS"); lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
    else if (emaScores[4] > ALERT_THRESHOLD) { Serial.println("ALERT:LUMBAR_FLATTENING");    lastAlertTime = now; currentState = FROZEN; frozenEnteredAt = now; }
  }
}

// FROZEN ─────────────────────────────────
// Waiting for "ACK_ALERT" from webapp (user confirmed they corrected posture).
// After FROZEN_TIMEOUT_MS (5 min) with no ACK, auto-resumes monitoring
// and notifies the webapp so it can also resume its UI state.
// Blocking read is intentional — timing precision not needed here.
void handleFrozen() {
  // Check for ACK from webapp
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "ACK_ALERT") {
      // Reset EMA scores so old alert doesn't immediately re-trigger
      for (int i = 0; i < 5; i++) emaScores[i] = 0.0f;
      consecutiveBadReadings = 0;
      Serial.println("MONITORING_RESUMED");
      currentState = MONITORING;
      return;
    }
  }

  // Timeout: auto-resume after 5 minutes with no ACK
  if (millis() - frozenEnteredAt >= FROZEN_TIMEOUT_MS) {
    for (int i = 0; i < 5; i++) emaScores[i] = 0.0f;
    consecutiveBadReadings = 0;
    // Notify webapp so it can also exit its "waiting for ACK" UI state
    Serial.println("FROZEN_TIMEOUT");
    Serial.println("MONITORING_RESUMED");
    currentState = MONITORING;
  }
}

// ─────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  // Verify TCA is present before attempting IMU init
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR:TCA_NOT_FOUND");
    while (true) delay(1000);  // halt — nothing works without the mux
  }
  Serial.println("TCA9548A found ✅");

  // Initialise all three IMUs
  bool ok1 = initIMU(imuUpper, CH_UPPER, "IMU_UPPER (ch1)"); 
  bool ok2 = initIMU(imuMid,   CH_MID,   "IMU_MID   (ch2)");
  bool ok3 = initIMU(imuLower, CH_LOWER, "IMU_LOWER (ch7)");

  if (!ok1 || !ok2 || !ok3) {
    Serial.println("ERROR:IMU_INIT_FAILED");
    // Do not halt — partial operation may still be useful during development 
  }

  // Initialise all three Madgwick filters at the chosen sample rate
  filterUpper.begin(4);  // 4 Hz — must match actual loop rate
  filterMid.begin(4);
  filterLower.begin(4);

  Serial.println("SYSTEM_READY");
  Serial.println("Waiting for START_CAL from webapp...");
}

// ─────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────
void loop() {
  unsigned long start = micros();

  switch (currentState) {
    case WAITING:            handleWaiting();          break;
    case COUNTDOWN:          handleCountdown();         break;
    case CALIBRATING:        handleCalibrating();       break;
    case WAITING_FOR_START:  handleWaitingForStart();   break;
    case MONITORING:         handleMonitoring();        break;
    case FROZEN:             handleFrozen();            break;
  }

  // Busy-compensation timing — only applied during states that need
  // precise 4 Hz sampling (CALIBRATING and MONITORING).
  // Other states block on Serial or idle, so timing there doesn't matter.
  if (currentState == MONITORING || currentState == CALIBRATING) {
    unsigned long elapsed = micros() - start;
    if (elapsed < SAMPLE_INTERVAL_US) {
      delayMicroseconds(SAMPLE_INTERVAL_US - elapsed);
    }
  }
}
