#include <Arduino.h>

/* ===================== Pins (Raspberry Pi Pico + L298N) ===================== */
#define ENA 14
#define ENB 15
#define IN1 4
#define IN2 5
#define IN3 2
#define IN4 3
#define ENC_L_A 6
#define ENC_L_B 7
#define ENC_R_A 17
#define ENC_R_B 16

/* ===================== Ultrasonic ===================== */
#define US_FRONT_ECHO 8
#define US_FRONT_TRIG 9

/* ===================== Geometry / Encoder ===================== */
const float WHEEL_DIAMETER_M = 0.07070f;
const long  TICKS_PER_REV    = 1125;

inline float ticksPerMeter() {
  return (float)TICKS_PER_REV / (PI * WHEEL_DIAMETER_M);
}

/* =============================== Timing =============================== */
const int CTRL_DT_MS = 15;
const int PRINT_MS   = 100;
#define DEBUG 1

/* =============================== Target Distance =============================== */
float DIST_SET_CM = 25.0f;

/* =============================== Limits =============================== */
int MIN_PWM_MOVE = 150;
int MAX_FWD_PWM  = 255;
int MAX_REV_PWM  = 255;

/* =============================== Safety =============================== */
float MAX_VALID_US_CM  = 300.0f;
int   ULTRA_TIMEOUT_US = 30000;

/* =============================== Braking =============================== */
bool ENABLE_DIRECTION_BRAKE = true;
int  BRAKE_PWM              = 255;
int  BRAKE_TIME_MS          = 200;

/* =============================== Reverse kick =============================== */
int REVERSE_KICK_PWM = 255;
int REVERSE_KICK_MS  = 50;

/* =============================== Distance PID =============================== */
float DIST_KP = 30.0f;
float DIST_KI = 0.0f;
float DIST_KD = 5.0f;

/* =============================== K-ratio Balance =============================== */
float K_RATIO   = 150.0f;
float REL_CLAMP = 0.01f;
int   BAL_CLAMP = 20;

const int MA_N = 5;

/* =============================== State =============================== */
volatile long encL = 0;
volatile long encR = 0;

bool MOTOR_INV_LEFT    = true;
bool MOTOR_INV_RIGHT   = true;
bool ENCODER_INV_LEFT  = false;
bool ENCODER_INV_RIGHT = false;

float distF = -1.0f;

int baseCmdPWM = 0;
int pwmCmdL    = 0;
int pwmCmdR    = 0;

float encVelL_mps    = 0.0f;
float encVelR_mps    = 0.0f;
float encVelAvg_mps  = 0.0f;

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
  digitalWrite(IN2, d ? LOW : HIGH);
}

void setDirRight(bool forward) {
  bool d = forward ^ MOTOR_INV_RIGHT;
  digitalWrite(IN3, d ? HIGH : LOW);
  digitalWrite(IN4, d ? LOW : HIGH);
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

/* ===================== Ultrasonic Read ===================== */
float readUSRaw(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  float cm = (duration * 0.0343f) / 2.0f;
  if (cm > MAX_VALID_US_CM) return -1.0f;

  return cm;
}

void updateSensors() {
  distF = readUSRaw(US_FRONT_TRIG, US_FRONT_ECHO);
}

/* ===================== Encoder Velocity ===================== */
void updateEncoderVelocity(float dt) {
  static long lastL = 0;
  static long lastR = 0;
  static bool first = true;

  long nowL = getCountL();
  long nowR = getCountR();

  if (first) {
    lastL = nowL;
    lastR = nowR;
    encVelL_mps = 0.0f;
    encVelR_mps = 0.0f;
    encVelAvg_mps = 0.0f;
    first = false;
    return;
  }

  long dL = nowL - lastL;
  long dR = nowR - lastR;

  lastL = nowL;
  lastR = nowR;

  float tpm = ticksPerMeter();

  encVelL_mps = ((float)dL / tpm) / dt;
  encVelR_mps = ((float)dR / tpm) / dt;
  encVelAvg_mps = 0.5f * (encVelL_mps + encVelR_mps);
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

/* ===================== Distance Control ===================== */
void updateDistancePID(float dt) {
  if (distF < 0.0f) {
    baseCmdPWM = 0;
    pidDist.reset();
    return;
  }

  float err = distF - DIST_SET_CM;   // positive -> too far -> move forward
  int cmd = (int)roundf(pidDist.update(err, dt));

  if (cmd > 0) {
    if (cmd < MIN_PWM_MOVE) cmd = MIN_PWM_MOVE;
    if (cmd > MAX_FWD_PWM)  cmd = MAX_FWD_PWM;
  } else if (cmd < 0) {
    if (cmd > -MIN_PWM_MOVE) cmd = -MIN_PWM_MOVE;
    if (cmd < -MAX_REV_PWM)  cmd = -MAX_REV_PWM;
  }

  baseCmdPWM = cmd;
}

/* ===================== Straight Balance ===================== */
void updateStraightBalanceAndDrive() {
  static float bufL[MA_N] = {0};
  static float bufR[MA_N] = {0};
  static float sumL = 0.0f;
  static float sumR = 0.0f;
  static int idx = 0;
  static int filled = 0;

  sumL -= bufL[idx];
  sumR -= bufR[idx];

  bufL[idx] = encVelL_mps;
  bufR[idx] = encVelR_mps;

  sumL += bufL[idx];
  sumR += bufR[idx];

  idx = (idx + 1) % MA_N;
  if (filled < MA_N) filled++;

  int finalL = baseCmdPWM;
  int finalR = baseCmdPWM;

  if (baseCmdPWM != 0) {
    float aL = fabsf(sumL / (float)filled);
    float aR = fabsf(sumR / (float)filled);

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
  }

  setMotorSignedPWM(finalL, finalR);
}

/* ===================== Debug Print ===================== */
void printSensors() {
#if DEBUG
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= (unsigned long)PRINT_MS) {
    lastPrint = now;

    Serial.print(F("F="));
    Serial.print(distF, 2);

    Serial.print(F("  err="));
    if (distF > 0.0f) Serial.print(distF - DIST_SET_CM, 2);
    else              Serial.print(-999.0f, 2);

    Serial.print(F("  vL="));
    Serial.print(encVelL_mps, 3);

    Serial.print(F("  vR="));
    Serial.print(encVelR_mps, 3);

    Serial.print(F("  vAvg="));
    Serial.print(encVelAvg_mps, 3);

    Serial.print(F("  base="));
    Serial.print(baseCmdPWM);

    Serial.print(F("  brake="));
    Serial.println(brakeActive);
  }
#endif
}

/* =============================== Setup =============================== */
void setup() {
  delay(2000);

#if DEBUG
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== Front Ultrasonic Constant Distance Control ==="));
#endif

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(US_FRONT_TRIG, OUTPUT);
  pinMode(US_FRONT_ECHO, INPUT);
  digitalWrite(US_FRONT_TRIG, LOW);

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

  pidDist.set(DIST_KP, DIST_KI, DIST_KD, -MAX_REV_PWM, MAX_FWD_PWM);

#if DEBUG
  Serial.print(F("DIST_SET_CM = "));
  Serial.println(DIST_SET_CM);

  Serial.print(F("DIST PID = "));
  Serial.print(DIST_KP);
  Serial.print(F(", "));
  Serial.print(DIST_KI);
  Serial.print(F(", "));
  Serial.println(DIST_KD);

  Serial.print(F("MIN_PWM_MOVE = "));
  Serial.println(MIN_PWM_MOVE);
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
      printSensors();
      return;
    }
  }

  if (now - lastCtrlMs >= (unsigned long)CTRL_DT_MS) {
    float dt = (now - lastCtrlMs) / 1000.0f;
    lastCtrlMs = now;

    updateSensors();
    updateEncoderVelocity(dt);
    updateDistancePID(dt);
    updateStraightBalanceAndDrive();
    printSensors();
  }
}
