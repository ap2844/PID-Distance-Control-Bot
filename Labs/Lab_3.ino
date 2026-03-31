#include <Arduino.h>

/* ===================== Pins (Raspberry Pi Pico + L298N) ===================== */
#define ENA 0
#define ENB 1
#define IN1 3
#define IN2 2
#define IN3 5
#define IN4 4
#define ENC_L_A 7
#define ENC_L_B 6
#define ENC_R_A 17
#define ENC_R_B 16

/* ===================== Ultrasonic ===================== */
#define US_ECHO 8
#define US_TRIG 9

/* ===================== Geometry / Encoder ===================== */
const float WHEEL_DIAMETER_M = 0.07070f;
const long  TICKS_PER_REV    = 1125;

inline float ticksPerMeter() {
  return (float)TICKS_PER_REV / (PI * WHEEL_DIAMETER_M);
}

/* =============================== Timing =============================== */
const int CTRL_DT_MS  = 20;
const int PRINT_MS    = 100;
#define DEBUG 1

/* =============================== Follow Goal =============================== */
float DIST_SET_CM        = 25.0f;   // desired distance from object
float DIST_DEADBAND_CM   = 2.50f;    // stop band
float DIST_REVERSE_AT_CM = 22.50f;   // definitely too close => reverse

/* =============================== Limits =============================== */
int MAX_PWM        = 255;
int MIN_PWM_MOVE   = 150;   // raised to overcome reverse hesitation
int MAX_FWD_PWM    = 255;
int MAX_REV_PWM    = 255;   // reverse still capped slightly lower than full

/* =============================== Safety =============================== */
float MAX_VALID_US_CM  = 300.0f;
int   ULTRA_TIMEOUT_US = 30000;
int   LOST_TARGET_MAX  = 5;

/* =============================== Braking =============================== */
bool ENABLE_DIRECTION_BRAKE = true;
int  BRAKE_PWM              = 255;
int  BRAKE_TIME_MS          = 200;   // longer brake before reverse

/* =============================== Reverse kick =============================== */
int REVERSE_KICK_PWM = 255;   // short shove when reverse begins
int REVERSE_KICK_MS  = 50;

/* =============================== Distance PID =============================== */
/*
   err = measured_distance - desired_distance

   err > 0 => too far  => forward
   err < 0 => too close => reverse
*/
float DIST_KP = 12.0f;
float DIST_KI = 0.0f;
float DIST_KD = 4.0f;

/* =============================== K-ratio Balance =============================== */
float K_RATIO   = 150.0f;
float REL_CLAMP = 0.20f;
int   BAL_CLAMP = 40;

const int MA_N = 5;

/* =============================== State =============================== */
volatile long encL = 0;
volatile long encR = 0;

bool MOTOR_INV_LEFT    = true;
bool MOTOR_INV_RIGHT   = true;
bool ENCODER_INV_LEFT  = false;
bool ENCODER_INV_RIGHT = false;

float distFilteredCm = -1.0f;
int   lostTargetCount = 0;

int baseCmdPWM = 0;
int pwmCmdL    = 0;
int pwmCmdR    = 0;

/* ========================== Brake State Machine ========================== */
bool brakeActive = false;
unsigned long brakeStartMs = 0;
int pendingPwmL = 0;
int pendingPwmR = 0;

#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

/* =============================== PID Struct =============================== */
struct PID {
  float kp, ki, kd;
  float integ;
  float prevErr;
  float outMin, outMax;
  bool first;

  PID() {
    kp = ki = kd = 0.0f;
    integ = 0.0f;
    prevErr = 0.0f;
    outMin = -255.0f;
    outMax = 255.0f;
    first = true;
  }

  void set(float p, float i, float d, float mn, float mx) {
    kp = p;
    ki = i;
    kd = d;
    outMin = mn;
    outMax = mx;
    reset();
  }

  void reset() {
    integ = 0.0f;
    prevErr = 0.0f;
    first = true;
  }

  float update(float err, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float deriv = 0.0f;
    if (!first) deriv = (err - prevErr) / dt;
    first = false;

    float newInteg = integ + err * dt;
    float out = kp * err + ki * newInteg + kd * deriv;

    if (out <= outMax && out >= outMin) {
      integ = newInteg;
    }

    out = kp * err + ki * integ + kd * deriv;

    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;

    prevErr = err;
    return out;
  }
};

PID pidDist;

/* ============================ Encoder ISRs ============================ */
void isrLeftA() {
  bool A = digitalReadFast(ENC_L_A);
  bool B = digitalReadFast(ENC_L_B);
  encL += (A == B) ? +1 : -1;
}

void isrLeftB() {
  bool A = digitalReadFast(ENC_L_A);
  bool B = digitalReadFast(ENC_L_B);
  encL += (A != B) ? +1 : -1;
}

void isrRightA() {
  bool A = digitalReadFast(ENC_R_A);
  bool B = digitalReadFast(ENC_R_B);
  encR += (A == B) ? +1 : -1;
}

void isrRightB() {
  bool A = digitalReadFast(ENC_R_A);
  bool B = digitalReadFast(ENC_R_B);
  encR += (A != B) ? +1 : -1;
}

/* ============================== Helpers ============================== */
inline int clamp255(int x) {
  if (x < 0) return 0;
  if (x > 255) return 255;
  return x;
}

long getRawL() {
  noInterrupts();
  long v = encL;
  interrupts();
  return v;
}

long getRawR() {
  noInterrupts();
  long v = encR;
  interrupts();
  return v;
}

long getCountL() {
  long v = getRawL();
  return ENCODER_INV_LEFT ? -v : v;
}

long getCountR() {
  long v = getRawR();
  return ENCODER_INV_RIGHT ? -v : v;
}

void setDirLeft(bool forward) {
  bool d = forward ^ MOTOR_INV_LEFT;
  digitalWrite(IN1, d ? HIGH : LOW);
  digitalWrite(IN2, d ? LOW  : HIGH);
}

void setDirRight(bool forward) {
  bool d = forward ^ MOTOR_INV_RIGHT;
  digitalWrite(IN3, d ? HIGH : LOW);
  digitalWrite(IN4, d ? LOW  : HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  pwmCmdL = 0;
  pwmCmdR = 0;
}

void activeBrake(int pwm) {
  pwm = clamp255(abs(pwm));

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);

  pwmCmdL = 0;
  pwmCmdR = 0;
}

void applyMotorSignedPWMNow(int pwmL, int pwmR) {
  static int prevSignL = 0;
  static int prevSignR = 0;

  bool dirL = (pwmL >= 0);
  bool dirR = (pwmR >= 0);

  int magL = abs(pwmL);
  int magR = abs(pwmR);

  int signL = (magL == 0) ? 0 : (dirL ? 1 : -1);
  int signR = (magR == 0) ? 0 : (dirR ? 1 : -1);

  if (magL > 0 && magL < MIN_PWM_MOVE) magL = MIN_PWM_MOVE;
  if (magR > 0 && magR < MIN_PWM_MOVE) magR = MIN_PWM_MOVE;

  magL = clamp255(magL);
  magR = clamp255(magR);

  setDirLeft(dirL);
  setDirRight(dirR);

  bool reverseStart =
      ((prevSignL >= 0 && signL < 0) || (prevSignR >= 0 && signR < 0));

  if (reverseStart) {
    analogWrite(ENA, REVERSE_KICK_PWM);
    analogWrite(ENB, REVERSE_KICK_PWM);
    delay(REVERSE_KICK_MS);
  }

  analogWrite(ENA, magL);
  analogWrite(ENB, magR);

  pwmCmdL = dirL ? magL : -magL;
  pwmCmdR = dirR ? magR : -magR;

  prevSignL = signL;
  prevSignR = signR;
}

void setMotorSignedPWM(int pwmL, int pwmR) {
  bool newDirL = (pwmL >= 0);
  bool newDirR = (pwmR >= 0);

  bool oldDirL = (pwmCmdL >= 0);
  bool oldDirR = (pwmCmdR >= 0);

  int magL = abs(pwmL);
  int magR = abs(pwmR);

  bool wantMoveL = (magL > 0);
  bool wantMoveR = (magR > 0);

  bool oldMoveL = (abs(pwmCmdL) > 0);
  bool oldMoveR = (abs(pwmCmdR) > 0);

  bool dirFlipL = oldMoveL && wantMoveL && (newDirL != oldDirL);
  bool dirFlipR = oldMoveR && wantMoveR && (newDirR != oldDirR);

  bool meaningfulFlipL = dirFlipL && (magL >= MIN_PWM_MOVE);
  bool meaningfulFlipR = dirFlipR && (magR >= MIN_PWM_MOVE);

  if (ENABLE_DIRECTION_BRAKE && (meaningfulFlipL || meaningfulFlipR)) {
    brakeActive = true;
    brakeStartMs = millis();
    pendingPwmL = pwmL;
    pendingPwmR = pwmR;
    activeBrake(BRAKE_PWM);
    return;
  }

  applyMotorSignedPWMNow(pwmL, pwmR);
}

/* ===================== Ultrasonic ===================== */
float readUltrasonicCmRaw() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long duration = pulseIn(US_ECHO, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  float cm = (duration * 0.0343f) / 2.0f;
  return cm;
}

float readUltrasonicCmFiltered() {
  float d = readUltrasonicCmRaw();

  if (d < 0.0f || d > MAX_VALID_US_CM) {
    lostTargetCount++;
    if (lostTargetCount > LOST_TARGET_MAX) {
      distFilteredCm = -1.0f;
    }
    return distFilteredCm;
  }

  lostTargetCount = 0;

  if (distFilteredCm < 0.0f) {
    distFilteredCm = d;
  } else {
    distFilteredCm = 0.4f * distFilteredCm + 0.6f * d;
  }

  return distFilteredCm;
}

/* ===================== Auto Encoder Polarity ===================== */
void autoConfigure() {
#if DEBUG
  Serial.println(F("\n[AUTOCONFIG] Probing encoder polarity..."));
#endif

  long l0 = getRawL();
  long r0 = getRawR();

  setDirLeft(true);
  setDirRight(true);
  analogWrite(ENA, 170);
  analogWrite(ENB, 170);
  delay(500);

  stopMotors();
  delay(120);

  long dL = getRawL() - l0;
  long dR = getRawR() - r0;

  ENCODER_INV_LEFT  = (dL < 0);
  ENCODER_INV_RIGHT = (dR < 0);

#if DEBUG
  Serial.print(F("[AUTOCONFIG] dL="));
  Serial.print(dL);
  Serial.print(F(" dR="));
  Serial.print(dR);
  Serial.print(F(" ENC_INV(L/R)="));
  Serial.print(ENCODER_INV_LEFT);
  Serial.print('/');
  Serial.println(ENCODER_INV_RIGHT);
#endif
}

/* ===================== Distance PID ===================== */
void updateDistanceControl(float dt) {
  float d = readUltrasonicCmFiltered();

  if (d < 0.0f) {
    baseCmdPWM = 0;
    pidDist.reset();
    return;
  }

  if (fabs(d - DIST_SET_CM) <= DIST_DEADBAND_CM) {
    baseCmdPWM = 0;
    pidDist.reset();
    return;
  }

  if (d <= DIST_REVERSE_AT_CM) {
    float err = d - DIST_REVERSE_AT_CM;
    int cmd = (int)roundf(pidDist.update(err, dt));

    // Force a meaningful reverse command
    if (cmd > -MIN_PWM_MOVE) cmd = -MIN_PWM_MOVE;
    if (cmd < -MAX_REV_PWM)  cmd = -MAX_REV_PWM;

    baseCmdPWM = cmd;
    return;
  }

  if (d > DIST_SET_CM + DIST_DEADBAND_CM) {
    float err = d - DIST_SET_CM;
    int cmd = (int)roundf(pidDist.update(err, dt));

    if (cmd < MIN_PWM_MOVE) cmd = MIN_PWM_MOVE;
    if (cmd > MAX_FWD_PWM)  cmd = MAX_FWD_PWM;

    baseCmdPWM = cmd;
    return;
  }

  baseCmdPWM = 0;
  pidDist.reset();
}

/* ===================== K-ratio Straight Balancer ===================== */
void updateStraightBalanceAndDrive() {
  static long lastL = 0;
  static long lastR = 0;
  static bool first = true;

  static long bufL[MA_N] = {0};
  static long bufR[MA_N] = {0};
  static long sumL = 0;
  static long sumR = 0;
  static int idx = 0;
  static int filled = 0;

  long nowL = getCountL();
  long nowR = getCountR();

  if (first) {
    lastL = nowL;
    lastR = nowR;
    first = false;
    setMotorSignedPWM(baseCmdPWM, baseCmdPWM);
    return;
  }

  long dL = nowL - lastL;
  long dR = nowR - lastR;
  lastL = nowL;
  lastR = nowR;

  sumL -= bufL[idx];
  sumR -= bufR[idx];
  bufL[idx] = dL;
  bufR[idx] = dR;
  sumL += bufL[idx];
  sumR += bufR[idx];

  idx = (idx + 1) % MA_N;
  if (filled < MA_N) filled++;

  int finalL = baseCmdPWM;
  int finalR = baseCmdPWM;

  if (baseCmdPWM != 0) {
    float aL = fabsf((float)sumL);
    float aR = fabsf((float)sumR);

    float rel;
    if (aL == 0.0f && aR == 0.0f) rel = 0.0f;
    else if (aL == 0.0f)          rel = REL_CLAMP;
    else {
      rel = (aR / aL) - 1.0f;
      if (rel >  REL_CLAMP) rel =  REL_CLAMP;
      if (rel < -REL_CLAMP) rel = -REL_CLAMP;
    }

    int balance = (int)roundf(K_RATIO * rel);
    if (balance >  BAL_CLAMP) balance =  BAL_CLAMP;
    if (balance < -BAL_CLAMP) balance = -BAL_CLAMP;

    finalL = baseCmdPWM + balance;
    finalR = baseCmdPWM - balance;

    if (baseCmdPWM > 0) {
      if (finalL < 0) finalL = 0;
      if (finalR < 0) finalR = 0;
      if (finalL > MAX_FWD_PWM) finalL = MAX_FWD_PWM;
      if (finalR > MAX_FWD_PWM) finalR = MAX_FWD_PWM;
    } else {
      if (finalL > 0) finalL = 0;
      if (finalR > 0) finalR = 0;
      if (finalL < -MAX_REV_PWM) finalL = -MAX_REV_PWM;
      if (finalR < -MAX_REV_PWM) finalR = -MAX_REV_PWM;
    }

#if DEBUG
    static unsigned long lastDbg = 0;
    unsigned long now = millis();
    if (now - lastDbg >= (unsigned long)PRINT_MS) {
      lastDbg = now;

      float dLma = (filled > 0) ? (float)sumL / (float)filled : 0.0f;
      float dRma = (filled > 0) ? (float)sumR / (float)filled : 0.0f;

      Serial.print(F("dist="));
      Serial.print(distFilteredCm, 2);
      Serial.print(F(" cm  base="));
      Serial.print(baseCmdPWM);

      Serial.print(F("  dL5="));
      Serial.print(dLma, 2);
      Serial.print(F("  dR5="));
      Serial.print(dRma, 2);

      Serial.print(F("  rel="));
      Serial.print(rel, 4);

      Serial.print(F("  pwmL="));
      Serial.print(finalL);
      Serial.print(F("  pwmR="));
      Serial.print(finalR);

      Serial.print(F("  lost="));
      Serial.print(lostTargetCount);

      Serial.print(F("  brake="));
      Serial.println(brakeActive);
    }
#endif
  }

  setMotorSignedPWM(finalL, finalR);
}

/* =============================== Setup =============================== */
void setup() {
  delay(2000);

#if DEBUG
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== Ultrasonic PID + K-Ratio Balance (reverse-fixed) ==="));
#endif

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrRightB, CHANGE);

  stopMotors();
  autoConfigure();

  pidDist.set(
    DIST_KP,
    DIST_KI,
    DIST_KD,
    -MAX_REV_PWM,
    MAX_FWD_PWM
  );

#if DEBUG
  Serial.print(F("ticks/m = "));
  Serial.println(ticksPerMeter());

  Serial.print(F("Distance setpoint (cm) = "));
  Serial.println(DIST_SET_CM);

  Serial.print(F("Reverse threshold (cm) = "));
  Serial.println(DIST_REVERSE_AT_CM);

  Serial.print(F("Distance deadband (cm) = "));
  Serial.println(DIST_DEADBAND_CM);

  Serial.print(F("Forward/Reverse max PWM = "));
  Serial.print(MAX_FWD_PWM);
  Serial.print(F(" / "));
  Serial.println(MAX_REV_PWM);

  Serial.print(F("MIN_PWM_MOVE = "));
  Serial.println(MIN_PWM_MOVE);

  Serial.print(F("K_RATIO = "));
  Serial.println(K_RATIO);

  Serial.print(F("Brake PWM / Time = "));
  Serial.print(BRAKE_PWM);
  Serial.print(F(" / "));
  Serial.println(BRAKE_TIME_MS);

  Serial.print(F("Reverse kick PWM / Time = "));
  Serial.print(REVERSE_KICK_PWM);
  Serial.print(F(" / "));
  Serial.println(REVERSE_KICK_MS);
#endif
}

/* ================================ Main ================================ */
void loop() {
  static unsigned long lastCtrlMs = 0;
  unsigned long now = millis();

  if (lastCtrlMs == 0) lastCtrlMs = now;

  if (brakeActive) {
    if (now - brakeStartMs >= (unsigned long)BRAKE_TIME_MS) {
      brakeActive = false;
      applyMotorSignedPWMNow(pendingPwmL, pendingPwmR);
    } else {
      activeBrake(BRAKE_PWM);
      return;
    }
  }

  if (now - lastCtrlMs >= (unsigned long)CTRL_DT_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;

    updateDistanceControl(dt);
    updateStraightBalanceAndDrive();
  }
}
