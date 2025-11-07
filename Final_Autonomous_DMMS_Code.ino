#include <Arduino.h>
 
/* ===================== L298N DC MOTORS (PWM) =====================
   Wiring:
     ENA -> D2   (Right motor enable, PWM)
     ENB -> D3   (Left  motor enable, PWM)
     IN1 -> D34  (Right dir 1)
     IN2 -> D35  (Right dir 2)
     IN3 -> D36  (Left  dir 1)
     IN4 -> D37  (Left  dir 2)
*/
#define ENA 2
#define ENB 3
#define IN1 34
#define IN2 35
#define IN3 36
#define IN4 37

// ---- TUNING CHEATSHEET ----
// BASE_FWD_PWML/R : forward cruise speed (balance straightness by offsetting L/R)
// BASE_IDLE_*     : brief dip during sampling to reduce echo noise (no full stop)
// BASE_TURN_*     : turning power (raise for snappier pivot; lower to avoid skid)
// STOP_CM         : “too close” threshold for front trip (increase to be cautious)
// CLEAR_MARGIN_CM : extra clearance needed before leaving a turn/pause
// DIFF_MIN_CM     : minimum left/right difference before favouring a side
// CASTER_MIN/MAX  : filter band for false echoes from front casters
// TURN_MIN_MS     : min time before re-checking + absolute timeout in a turn
// STUCK_FWD_MS    : how long the scene can look “unchanged” in FORWARD before backup
// BACKUP_MS       : reverse time when stuck, then SCAN_SPOT picks a new heading

/* ===================== BASE SPEEDS (0..255) ===================== */
const uint8_t BASE_FWD_PWML   = 75;
const uint8_t BASE_FWD_PWMR   = 78;
const uint8_t BASE_IDLE_PWML  = 30;
const uint8_t BASE_IDLE_PWMR  = 30;
const uint8_t BASE_TURN_PWML  = 75;
const uint8_t BASE_TURN_PWMR  = 78;
 
/* ===== Sampling ===== */
const unsigned long SAMPLE_PERIOD_MS = 180;
const unsigned long SAMPLE_PAUSE_MS  = 25;
 
/* ===================== 5x ULTRASONICS ===================== */
struct Sonar { uint8_t trig; uint8_t echo; const char *name; };
Sonar sensors[5] = {
  {22, 23, "L-OUT"},
  {24, 25, "L-IN "},
  {26, 27, "CTR  "},
  {28, 29, "R-IN "},
  {30, 31, "R-OUT"}
};
 
/* ===================== TUNABLES ===================== */
const int   TURN_CHECK_MS    = 300;
const int   TURN_MIN_MS      = 900;
const int   TURN_TIMEOUT_MS  = 5000;
const int   COOLDOWN_MS      = 60;
const int   STOP_CM          = 62;
const int   CLEAR_MARGIN_CM  = 23;
const int   DIFF_MIN_CM      = 8;
const int   CASTER_MIN_CM    = 34;

const int   CASTER_MAX_CM    = 42;

/* ======== STUCK RECOVERY ======== */
/* triggers if PAUSE_BLOCKED lasts too long */
const int   STUCK_TIMEOUT_MS = 4000;   // pause-blocked -> consider stuck after 4s
/* reverse parameters */
const int   BACKUP_MS        = 2000;    // reverse duration
const int   BACKUP_PWML      = 54;     // reverse speed L
const int   BACKUP_PWMR      = 56;     // reverse speed R
/* forward stall detector (NEW) */
const int   STUCK_FWD_MS     = 4000;   // if readings hardly change for this long while in FORWARD
const int   STALL_CHECK_MS   = 400;    // how often to re-check stall
const int   STALL_DELTA_CM   = 6;      // “hardly change” threshold on avg of 5 sensors
 
/* ===================== STATE MACHINES ===================== */
enum ModeAuto { FORWARD, TURN_LEFT, TURN_RIGHT, PAUSE_BLOCKED, BACKUP, SCAN_SPOT };
ModeAuto autoMode = FORWARD;
 
enum ControlMode { MODE_AUTO, MODE_TELEOP, MODE_ESTOP };
ControlMode controlMode = MODE_AUTO;
 
/* teleop timeout (backup) */
unsigned long lastTeleopCmdMs = 0;
const unsigned long TELEOP_TIMEOUT_MS = 1500;
 
/* teleop speed (from S###) */
uint8_t teleopSpeedPct = 60;
 
/* auto timing vars */
unsigned long autoStateStart = 0;
unsigned long autoLastTurnCheck = 0;
 
/* when ESP presses emergency, we latch this */
bool remoteHold = false;
 
/* ===================== MOTOR HELPERS ===================== */
inline void driveSide(bool right, int16_t pwm) {
  uint8_t ena  = right ? ENA : ENB;
  uint8_t inA  = right ? IN1 : IN3;
  uint8_t inB  = right ? IN2 : IN4;
 
  if (pwm == 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
    analogWrite(ena, 0);
    return;
  }
 
  if (pwm > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
    pwm = -pwm;
  }
 
  if (pwm > 255) pwm = 255;
  analogWrite(ena, (uint8_t)pwm);
}
 
inline void driveLR(int16_t left_pwm, int16_t right_pwm) {
  driveSide(false, left_pwm);
  driveSide(true,  right_pwm);
}
 
inline void stopMotors() {
  driveLR(0, 0);
}
 
/* ====== AUTO TURN HELPERS ====== */
inline void autoTurnLeftContinuous(uint8_t turnL, uint8_t turnR) {
  driveLR(-turnL, +turnR);
}
inline void autoTurnRightContinuous(uint8_t turnL, uint8_t turnR) {
  driveLR(+turnL, -turnR);
}
 
/* ===================== SENSOR READ ===================== */
static inline long readOnceCM(const Sonar &s) {
  digitalWrite(s.trig, LOW);  delayMicroseconds(2);
  digitalWrite(s.trig, HIGH); delayMicroseconds(10);
  digitalWrite(s.trig, LOW);
  unsigned long us = pulseIn(s.echo, HIGH, 20000UL);
  if (!us) return 999;
  long cm = (long)(us * 0.034 / 2);
  if (cm < 3) cm = 999;
  return cm;
}
static inline long median3(long a, long b, long c) {
  if (a>b) { long t=a; a=b; b=t; }
  if (b>c) { long t=b; b=c; c=t; }
  if (a>b) { long t=a; a=b; b=t; }
  return b;
}
long readSmoothedCM(const Sonar &s) {
  long a = readOnceCM(s); delay(8);
  long b = readOnceCM(s); delay(8);
  long c = readOnceCM(s);
  return median3(a,b,c);
}
long readQuickCM(const Sonar &s) {
  digitalWrite(s.trig, LOW);  delayMicroseconds(2);
  digitalWrite(s.trig, HIGH); delayMicroseconds(10);
  digitalWrite(s.trig, LOW);
  unsigned long us = pulseIn(s.echo, HIGH, 20000UL);
  if (!us) return 999;
  long cm = (long)(us * 0.034 / 2);
  if (cm < 3) cm = 999;
  return cm;
}
bool isCasterReading(long distance) {
  return (distance >= CASTER_MIN_CM && distance <= CASTER_MAX_CM);
}
 
/* ===================== AUTO STATE UTIL ===================== */
inline void enterAutoState(ModeAuto m) {
  autoMode = m;
  autoStateStart = millis();
  autoLastTurnCheck = millis();
}
 
/* ===================== TELEOP HELPERS ===================== */
void teleopForward()  { driveLR(BASE_FWD_PWML,  BASE_FWD_PWMR); }
void teleopBackward() { driveLR(-BASE_FWD_PWML, -BASE_FWD_PWMR); }
void teleopTurnLeft() { driveLR(-BASE_TURN_PWML, +BASE_TURN_PWMR); }
void teleopTurnRight(){ driveLR(+BASE_TURN_PWML, -BASE_TURN_PWMR); }
 
/* ===================== SERIAL PARSER ===================== */
void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') continue;
 
    // Teleop ON from ESP
    if (c == 'M') {
      controlMode = MODE_TELEOP;
      remoteHold = false;             // user is taking over, clear hold
      stopMotors();
      lastTeleopCmdMs = millis();
      Serial.println("[MEGA] Teleop MODE ON (from ESP)");
      continue;
    }
 
    // Teleop OFF / back to auto from ESP
    if (c == 'A') {
      // we now treat A as "release hold + go to auto"
      remoteHold = false;
      controlMode = MODE_AUTO;
      stopMotors();
      enterAutoState(PAUSE_BLOCKED);  // safe start
      Serial.println("[MEGA] AUTO MODE ON (from ESP, paused)");
      continue;
    }
 
    // speed packet
    if (c == 'S') {
      char d1 = Serial.read();
      char d2 = Serial.read();
      char d3 = Serial.read();
      if (isdigit(d1) && isdigit(d2) && isdigit(d3)) {
        int v = (d1 - '0') * 100 + (d2 - '0') * 10 + (d3 - '0');
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        teleopSpeedPct = (uint8_t)v;
        lastTeleopCmdMs = millis();
        Serial.print("[MEGA] Teleop speed = "); Serial.println(teleopSpeedPct);
      }
      continue;
    }
 
    // EMERGENCY from ESP (latch)
    if (c == 'E') {
      controlMode = MODE_ESTOP;
      remoteHold  = true;
      stopMotors();
      Serial.println("[MEGA] E-STOP triggered (latched)");
      continue;
    }
 
    // normal teleop keys
    switch (c) {
      case 'f':
        controlMode = MODE_TELEOP;
        remoteHold = false;
        lastTeleopCmdMs = millis();
        teleopForward();
        break;
      case 'b':
        controlMode = MODE_TELEOP;
        remoteHold = false;
        lastTeleopCmdMs = millis();
        teleopBackward();
        break;
      case 'l':
        controlMode = MODE_TELEOP;
        remoteHold = false;
        lastTeleopCmdMs = millis();
        teleopTurnLeft();
        break;
      case 'r':
        controlMode = MODE_TELEOP;
        remoteHold = false;
        lastTeleopCmdMs = millis();
        teleopTurnRight();
        break;
      case 's':
        controlMode = MODE_TELEOP;
        remoteHold = false;
        lastTeleopCmdMs = millis();
        stopMotors();
        break;
      default:
        break;
    }
  }
}
 
/* ===================== AUTO OBSTACLE LOGIC ===================== */
void runAutoObstacle() {
  long dLout = readSmoothedCM(sensors[0]);
  long dLin  = readSmoothedCM(sensors[1]);
  long dCtr  = readSmoothedCM(sensors[2]);
  long dRin  = readSmoothedCM(sensors[3]);
  long dRout = readSmoothedCM(sensors[4]);
 
  if (isCasterReading(dLout)) dLout = 999;
  if (isCasterReading(dRout)) dRout = 999;
 
  bool frontBlocked = (dCtr <= STOP_CM) || (dLin <= STOP_CM) || (dRin <= STOP_CM);
  bool rightSideBlocked = (dRin <= STOP_CM) || (dRout <= STOP_CM);
  bool leftSideBlocked  = (dLin <= STOP_CM) || (dLout <= STOP_CM);
 
  const int EXTREMELY_CLOSE_CM = 8;
  bool bothVeryClose = (dLout > 0 && dLout < EXTREMELY_CLOSE_CM) &&
                       (dRout > 0 && dRout < EXTREMELY_CLOSE_CM);
  bool bothSidesBlocked = (dLout <= STOP_CM + 2 * CLEAR_MARGIN_CM) &&
                          (dRout <= STOP_CM + 2 * CLEAR_MARGIN_CM);

  /* ====== FORWARD stall detector state (NEW) ====== */
  static unsigned long stallLastCheck = 0;
  static unsigned long stallStart = 0;
  static long          stallLastAvg = 0;
  auto avg5 = [&](long a,long b,long c,long d,long e)->long{
    long s = a + b + c + d + e;
    return s / 5;
  };
 
  switch (autoMode) {
    case FORWARD: {
      static unsigned long lastSample = 0;
      if (!frontBlocked) {
        driveLR(BASE_FWD_PWML, BASE_FWD_PWMR);

        // Sample-idle for cleaner sonar reads
        if (millis() - lastSample >= SAMPLE_PERIOD_MS) {
          driveLR(BASE_IDLE_PWML, BASE_IDLE_PWMR);
          delay(SAMPLE_PAUSE_MS);
          lastSample = millis();
          driveLR(BASE_FWD_PWML, BASE_FWD_PWMR);
        }

        // -------- NEW: detect being stuck while still in FORWARD --------
        if (millis() - stallLastCheck >= (unsigned long)STALL_CHECK_MS) {
          stallLastCheck = millis();
          long avgNow = avg5(dLout, dLin, dCtr, dRin, dRout);

          // initialize on first run
          if (stallStart == 0) {
            stallStart = millis();
            stallLastAvg = avgNow;
          }

          // if the environment "looks the same" (within a few cm), keep counting
          if (labs(avgNow - stallLastAvg) <= STALL_DELTA_CM) {
            if (millis() - stallStart >= (unsigned long)STUCK_FWD_MS) {
              // Consider this a stall: back up and try again
              stopMotors();
              enterAutoState(BACKUP);
              delay(COOLDOWN_MS);
              // reset stall tracker for next time
              stallStart = 0;
              stallLastAvg = avgNow;
              return;
            }
          } else {
            // something changed → reset the stall timer/baseline
            stallStart = millis();
            stallLastAvg = avgNow;
          }
        }
        // ---------------------------------------------------------------

        // normal side nudges
        if (rightSideBlocked && !leftSideBlocked) {
          enterAutoState(TURN_LEFT);
          delay(COOLDOWN_MS);
          return;
        }
        if (leftSideBlocked && !rightSideBlocked) {
          enterAutoState(TURN_RIGHT);
          delay(COOLDOWN_MS);
          return;
        }
        if (leftSideBlocked && rightSideBlocked) {
          if (dLout > dRout) enterAutoState(TURN_LEFT);
          else               enterAutoState(TURN_RIGHT);
          delay(COOLDOWN_MS);
          return;
        }
        return;
      }
 
      // front blocked
      stopMotors();
      // reset stall detector whenever we stop forward motion
      stallStart = 0;
      if (bothVeryClose || bothSidesBlocked) {
        enterAutoState(PAUSE_BLOCKED);
        delay(COOLDOWN_MS);
        return;
      }
      if (abs(dLout - dRout) >= DIFF_MIN_CM) {
        if (dLout > dRout) enterAutoState(TURN_LEFT);
        else               enterAutoState(TURN_RIGHT);
      } else {
        enterAutoState(TURN_RIGHT);
      }
      delay(COOLDOWN_MS);
    } break;
 
    case TURN_LEFT: {
      autoTurnLeftContinuous(BASE_TURN_PWML, BASE_TURN_PWMR);
      if (millis() - autoLastTurnCheck >= TURN_CHECK_MS) {
        autoLastTurnCheck = millis();
        if (millis() - autoStateStart >= TURN_MIN_MS) {
          long dCtrNow = readQuickCM(sensors[2]);
          long dLinNow = readQuickCM(sensors[1]);
          long dRinNow = readQuickCM(sensors[3]);
          bool frontClear = (dCtrNow > STOP_CM + CLEAR_MARGIN_CM) &&
                            ((dLinNow > STOP_CM + CLEAR_MARGIN_CM) ||
                             (dRinNow > STOP_CM + CLEAR_MARGIN_CM));
          if (frontClear) {
            stopMotors();
            delay(COOLDOWN_MS);
            enterAutoState(FORWARD);
            return;
          }
        }
        if (millis() - autoStateStart > TURN_TIMEOUT_MS) {
          stopMotors();
          delay(COOLDOWN_MS);
          enterAutoState(PAUSE_BLOCKED);
          return;
        }
      }
    } break;
 
    case TURN_RIGHT: {
      autoTurnRightContinuous(BASE_TURN_PWML, BASE_TURN_PWMR);
      if (millis() - autoLastTurnCheck >= TURN_CHECK_MS) {
        autoLastTurnCheck = millis();
        if (millis() - autoStateStart >= TURN_MIN_MS) {
          long dCtrNow = readQuickCM(sensors[2]);
          long dLinNow = readQuickCM(sensors[1]);
          long dRinNow = readQuickCM(sensors[3]);
          bool frontClear = (dCtrNow > STOP_CM + CLEAR_MARGIN_CM) &&
                            ((dLinNow > STOP_CM + CLEAR_MARGIN_CM) ||
                             (dRinNow > STOP_CM + CLEAR_MARGIN_CM));
          if (frontClear) {
            stopMotors();
            delay(COOLDOWN_MS);
            enterAutoState(FORWARD);
            return;
          }
        }
        if (millis() - autoStateStart > TURN_TIMEOUT_MS) {
          stopMotors();
          delay(COOLDOWN_MS);
          enterAutoState(PAUSE_BLOCKED);
          return;
        }
      }
    } break;
 
    case PAUSE_BLOCKED: {
      static unsigned long pauseStart = 0;
      if (pauseStart == 0) pauseStart = millis();
      if (millis() - pauseStart < 1500) {
        delay(COOLDOWN_MS);
        return;
      }

      // If we've been paused too long, treat as stuck -> backup then scan
      if (millis() - pauseStart >= (unsigned long)STUCK_TIMEOUT_MS) {
        pauseStart = 0;                 // reset for next time
        enterAutoState(BACKUP);         // begin recovery
        delay(COOLDOWN_MS);
        return;
      }

      delay(200);
      long dLoutNow = readSmoothedCM(sensors[0]);
      long dRoutNow = readSmoothedCM(sensors[4]);
      if (isCasterReading(dLoutNow)) dLoutNow = 999;
      if (isCasterReading(dRoutNow)) dRoutNow = 999;
      bool leftClear  = (dLoutNow > STOP_CM + 2 * CLEAR_MARGIN_CM);
      bool rightClear = (dRoutNow > STOP_CM + 2 * CLEAR_MARGIN_CM);
 
      if (leftClear && !rightClear) {
        pauseStart = 0;
        enterAutoState(TURN_LEFT);
        delay(COOLDOWN_MS);
        return;
      }
      if (rightClear && !leftClear) {
        pauseStart = 0;
        enterAutoState(TURN_RIGHT);
        delay(COOLDOWN_MS);
        return;
      }
      if (leftClear && rightClear) {
        pauseStart = 0;
        enterAutoState(FORWARD);
        delay(COOLDOWN_MS);
        return;
      }
      delay(COOLDOWN_MS);
    } break;

    /* ====== BACKUP when stuck ====== */
    case BACKUP: {
      driveLR(-BACKUP_PWML, -BACKUP_PWMR);
      if (millis() - autoStateStart >= (unsigned long)BACKUP_MS) {
        stopMotors();
        delay(COOLDOWN_MS);
        enterAutoState(SCAN_SPOT);  // then scan for new direction
        return;
      }
    } break;

    /* ====== SCAN current spot and pick a new heading ====== */
    case SCAN_SPOT: {
      long lOut = readSmoothedCM(sensors[0]);
      long lIn  = readSmoothedCM(sensors[1]);
      long ctr  = readSmoothedCM(sensors[2]);
      long rIn  = readSmoothedCM(sensors[3]);
      long rOut = readSmoothedCM(sensors[4]);

      if (isCasterReading(lOut)) lOut = 999;
      if (isCasterReading(rOut)) rOut = 999;

      bool frontGood = (ctr > STOP_CM + CLEAR_MARGIN_CM) &&
                       ((lIn > STOP_CM + CLEAR_MARGIN_CM) || (rIn > STOP_CM + CLEAR_MARGIN_CM));
      if (frontGood) {
        enterAutoState(FORWARD);
        delay(COOLDOWN_MS);
        return;
      }

      long leftScore  = max(lOut, lIn);
      long rightScore = max(rOut, rIn);

      if (abs(leftScore - rightScore) < DIFF_MIN_CM) {
        if (lIn >= rIn) enterAutoState(TURN_LEFT);
        else            enterAutoState(TURN_RIGHT);
      } else {
        if (leftScore > rightScore) enterAutoState(TURN_LEFT);
        else                        enterAutoState(TURN_RIGHT);
      }
      delay(COOLDOWN_MS);
      return;
    } break;
  }
}
 
/* ===================== SETUP ===================== */
void setup() {
  Serial.begin(115200);   // must match ESP8266
 
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();
 
  for (int i = 0; i < 5; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
    digitalWrite(sensors[i].trig, LOW);
  }
 
  Serial.println("[MEGA] Robot ready (AUTO + TELEOP + latched E-STOP)");
  enterAutoState(FORWARD);
}
 
/* ===================== LOOP ===================== */
void loop() {
  handleSerial();
 
  // hard e-stop: stay stopped
  if (controlMode == MODE_ESTOP) {
    stopMotors();
    return;
  }
 
  // teleop: user is driving
  if (controlMode == MODE_TELEOP) {
    return;
  }
 
  // MODE_AUTO
  runAutoObstacle();
}
