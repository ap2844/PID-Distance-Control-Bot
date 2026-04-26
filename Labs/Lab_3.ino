#include <Arduino.h>

/* ===================== Debug / Mode Toggles ===================== */
#define DEBUG 1

#define DEBUG_SENSORS 1
#define DEBUG_ACTIONS 1

#define ENABLE_FORWARD_BACKWARD 1
// 0 = only test left/right turning using side ultrasonic sensors
// 1 = enable normal front distance-following forward/backward motion

/* ===================== Pins ===================== */
#define ENA 14
#define ENB 15

#define IN1 4
#define IN2 5
#define IN3 0
#define IN4 3

#define ENC_L_A 6
#define ENC_L_B 7
#define ENC_R_A 17
#define ENC_R_B 16

/* ===================== Ultrasonic Sensors ===================== */
#define US_FRONT_ECHO 8
#define US_FRONT_TRIG 9

#define US_LEFT_ECHO 10
#define US_LEFT_TRIG 11

#define US_RIGHT_ECHO 12
#define US_RIGHT_TRIG 13

/* ===================== Constants ===================== */
const int CTRL_DT_MS = 20;
const int PRINT_MS = 100;

float DIST_SET_CM = 25.0f;
float DIST_DEADBAND_CM = 2.50f;
float DIST_REVERSE_AT_CM = 22.50f;

float SIDE_DETECT_CM = 50.0f;
int TURN_PWM = 180;

int MIN_PWM_MOVE = 150;
int MAX_FWD_PWM = 255;
int MAX_REV_PWM = 255;

float MAX_VALID_US_CM = 300.0f;
int ULTRA_TIMEOUT_US = 30000;
int LOST_TARGET_MAX = 5;

bool ENABLE_DIRECTION_BRAKE = true;
int BRAKE_PWM = 255;
int BRAKE_TIME_MS = 200;

int REVERSE_KICK_PWM = 255;
int REVERSE_KICK_MS = 50;

float DIST_KP = 30.0f;
float DIST_KI = 0.0f;
float DIST_KD = 10.0f;

/* ===================== State ===================== */
volatile long encL = 0;
volatile long encR = 0;

bool MOTOR_INV_LEFT = true;
bool MOTOR_INV_RIGHT = true;
bool ENCODER_INV_LEFT = false;
bool ENCODER_INV_RIGHT = false;

float distFilteredCm = -1.0f;
int lostTargetCount = 0;

int baseCmdPWM = 0;
int pwmCmdL = 0;
int pwmCmdR = 0;

bool brakeActive = false;
unsigned long brakeStartMs = 0;
int pendingPwmL = 0;
int pendingPwmR = 0;

int latchedSide = 0;
// 0  = no latch
// -1 = locked LEFT
// +1 = locked RIGHT

#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

/* ===================== PID ===================== */
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

    float deriv = first ? 0.0f : (err - prevErr) / dt;
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

/* ===================== Encoder ISRs ===================== */
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

/* ===================== Motor Helpers ===================== */
int clamp255(int x) {
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

  bool dirL = pwmL >= 0;
  bool dirR = pwmR >= 0;

  int magL = abs(pwmL);
  int magR = abs(pwmR);

  int signL = magL == 0 ? 0 : dirL ? 1 : -1;
  int signR = magR == 0 ? 0 : dirR ? 1 : -1;

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
  bool newDirL = pwmL >= 0;
  bool newDirR = pwmR >= 0;

  bool oldDirL = pwmCmdL >= 0;
  bool oldDirR = pwmCmdR >= 0;

  int magL = abs(pwmL);
  int magR = abs(pwmR);

  bool wantMoveL = magL > 0;
  bool wantMoveR = magR > 0;

  bool oldMoveL = abs(pwmCmdL) > 0;
  bool oldMoveR = abs(pwmCmdR) > 0;

  bool dirFlipL = oldMoveL && wantMoveL && (newDirL != oldDirL);
  bool dirFlipR = oldMoveR && wantMoveR && (newDirR != oldDirR);

  if (ENABLE_DIRECTION_BRAKE && (dirFlipL || dirFlipR)) {
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
float readUltrasonicCmRaw(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);

  if (duration == 0) return -1.0f;

  float cm = duration * 0.0343f / 2.0f;

  if (cm <= 0.0f || cm > MAX_VALID_US_CM) return -1.0f;

  return cm;
}

float readUltrasonicCmFiltered() {
  float d = readUltrasonicCmRaw(US_FRONT_TRIG, US_FRONT_ECHO);

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

/* ===================== Debug Printing ===================== */
void printSensorDebug(float front, float left, float right) {
#if DEBUG_SENSORS
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint < (unsigned long)PRINT_MS) return;
  lastPrint = now;

  Serial.print(F("FRONT="));
  Serial.print(front, 2);
  Serial.print(F(" cm | LEFT="));
  Serial.print(left, 2);
  Serial.print(F(" cm | RIGHT="));
  Serial.print(right, 2);

  Serial.print(F(" | LATCH="));

  if (latchedSide == -1) Serial.print(F("LEFT"));
  else if (latchedSide == +1) Serial.print(F("RIGHT"));
  else Serial.print(F("NONE"));

  Serial.print(F(" | frontValid="));
  Serial.print(front > 0.0f);

  Serial.print(F(" | leftDetect="));
  Serial.print(left > 0.0f && left < SIDE_DETECT_CM);

  Serial.print(F(" | rightDetect="));
  Serial.print(right > 0.0f && right < SIDE_DETECT_CM);

  Serial.println();
#endif
}

void printActionDebug(const char *msg) {
#if DEBUG_ACTIONS
  Serial.println(msg);
#endif
}

/* ===================== Side Sensor Latch Turn Logic ===================== */
bool acquireFrontFromSideSensors() {
  float front = readUltrasonicCmRaw(US_FRONT_TRIG, US_FRONT_ECHO);
  float left  = readUltrasonicCmRaw(US_LEFT_TRIG, US_LEFT_ECHO);
  float right = readUltrasonicCmRaw(US_RIGHT_TRIG, US_RIGHT_ECHO);

  bool frontDetect = front > 0.0f;
  bool leftDetect  = left > 0.0f && left < SIDE_DETECT_CM;
  bool rightDetect = right > 0.0f && right < SIDE_DETECT_CM;

  printSensorDebug(front, left, right);

  if (frontDetect) {
    latchedSide = 0;
    printActionDebug("ACTION: front acquired -> releasing latch");
    return false;
  }

  baseCmdPWM = 0;
  pidDist.reset();
  distFilteredCm = -1.0f;
  lostTargetCount = 0;

  if (latchedSide == 0) {
    if (leftDetect && rightDetect) {
      latchedSide = (left <= right) ? -1 : +1;

      if (latchedSide == -1) {
        printActionDebug("LATCH: both detect, LEFT closer -> locked LEFT");
      } else {
        printActionDebug("LATCH: both detect, RIGHT closer -> locked RIGHT");
      }
    } else if (leftDetect) {
      latchedSide = -1;
      printActionDebug("LATCH: locked LEFT");
    } else if (rightDetect) {
      latchedSide = +1;
      printActionDebug("LATCH: locked RIGHT");
    } else {
      printActionDebug("ACTION: no side detection -> STOP");
      stopMotors();
      return true;
    }
  }

  if (latchedSide == -1) {
    printActionDebug("ACTION: latched LEFT -> turning LEFT");
    setMotorSignedPWM(-TURN_PWM, TURN_PWM);
    return true;
  }

  if (latchedSide == +1) {
    printActionDebug("ACTION: latched RIGHT -> turning RIGHT");
    setMotorSignedPWM(TURN_PWM, -TURN_PWM);
    return true;
  }

  stopMotors();
  return true;
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

  ENCODER_INV_LEFT = dL < 0;
  ENCODER_INV_RIGHT = dR < 0;

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

/* ===================== Front Distance Control ===================== */
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

    if (cmd > -MIN_PWM_MOVE) cmd = -MIN_PWM_MOVE;
    if (cmd < -MAX_REV_PWM) cmd = -MAX_REV_PWM;

    baseCmdPWM = cmd;
    return;
  }

  if (d > DIST_SET_CM + DIST_DEADBAND_CM) {
    float err = d - DIST_SET_CM;
    int cmd = (int)roundf(pidDist.update(err, dt));

    if (cmd < MIN_PWM_MOVE) cmd = MIN_PWM_MOVE;
    if (cmd > MAX_FWD_PWM) cmd = MAX_FWD_PWM;

    baseCmdPWM = cmd;
    return;
  }

  baseCmdPWM = 0;
  pidDist.reset();
}

/* ===================== Setup ===================== */
void setup() {
  delay(2000);

#if DEBUG
  Serial.begin(115200);
  delay(200);

  Serial.println(F("\n=== 3 Ultrasonic Latch Turn Code ==="));

#if ENABLE_FORWARD_BACKWARD
  Serial.println(F("MODE: front distance following ENABLED"));
#else
  Serial.println(F("MODE: forward/backward disabled. Testing side turns only."));
#endif

#if DEBUG_SENSORS
  Serial.println(F("DEBUG_SENSORS: ON"));
#else
  Serial.println(F("DEBUG_SENSORS: OFF"));
#endif

#if DEBUG_ACTIONS
  Serial.println(F("DEBUG_ACTIONS: ON"));
#else
  Serial.println(F("DEBUG_ACTIONS: OFF"));
#endif

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

  pinMode(US_LEFT_TRIG, OUTPUT);
  pinMode(US_LEFT_ECHO, INPUT);
  digitalWrite(US_LEFT_TRIG, LOW);

  pinMode(US_RIGHT_TRIG, OUTPUT);
  pinMode(US_RIGHT_ECHO, INPUT);
  digitalWrite(US_RIGHT_TRIG, LOW);

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
}

/* ===================== Main Loop ===================== */
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

    if (acquireFrontFromSideSensors()) {
      return;
    }

#if ENABLE_FORWARD_BACKWARD
    updateDistanceControl(dt);
    setMotorSignedPWM(baseCmdPWM, baseCmdPWM);
#else
    stopMotors();
#endif
  }
}
