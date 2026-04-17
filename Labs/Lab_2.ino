/*
 * Lab1.ino
 *
 * Enter the Robotics Lab Mechatronics Arena with a 
 * 90 degree angle and traversing the course from one
 * entrance to another in a straight line while
 * hitting and subsequently dragging an obstacle in the
 * middle of the arena to the exit
 */

#include <Arduino.h>

/* ===================== Pins (Raspberry Pi Pico + L298N) ===================== */
// ===== Motor pins =====
#define ENA 14
#define ENB 15
#define IN1 4
#define IN2 5
#define IN3 2
#define IN4 3

// ===== Encoder pins =====
#define ENC_L_A 6
#define ENC_L_B 7
#define ENC_R_A 17
#define ENC_R_B 16

/* ===================== Geometry / Encoder ===================== */
const float WHEEL_DIAMETER_M = 0.07070f; // use your 0.06916f if re-measured
const float WHEEL_BASE_M     = 0.21746f;
const long  TICKS_PER_REV    = 1125;
inline float ticksPerMeter(){ return (float)TICKS_PER_REV / (PI * WHEEL_DIAMETER_M); }

/* =============================== Tuning =============================== */
/* Feed-forward split: start stronger side lower and weaker higher */
int   BASE_L      = 200;   // left base PWM
int   BASE_R      = 200;   // right base PWM
int   MIN_PWM     = 180;   // floor to avoid stall

/* Controller timing & logging */
int   CTRL_DT_MS  = 15;    // control loop tick
int   PRINT_MS    = 100;   // debug cadence
#define DEBUG 1

/* Ratio controller: balance = K_RATIO * (|dR5|/|dL5| - 1), clamped */
float K_RATIO     = 150.0f; // shove per 1.0 relative error
float REL_CLAMP   = 0.01f;  // cap ratio error to ±20%
int   BAL_CLAMP   = 20;     // cap balance PWM to ±30

/* Moving average window (smoothing) */
const int MA_N    = 5;

/* Turns */
int TURN_KICK_PWM = 225;
int TURN_KICK_MS  = 225;
int TURN_PWM_L    = 225;
int TURN_PWM_R    = 225;

/* Ram push */
int RAM_KICK_PWM  = 175;
int RAM_KICK_MS   = 45;

/* Distance/turn correction knobs */
float DIST_CORR = 0.725f;
float TURN_CORR = 0.850f;

/* ===== Simultaneous start gate ===== */
int   START_KICK_PWM     = 200;   // initial shove to overcome static friction
int   START_HOLD_PWM     = 160;   // keep already-moving side slow while waiting
int   START_MAX_WAIT_MS  = 600;   // overall wait guard
int   START_WINDOW_MS    = 25;    // check cadence during start gate
long  START_TICKS_THRESH = 3;     // ticks that count as "started"

/* =============================== State =============================== */
volatile long encL=0, encR=0;
bool MOTOR_INV_LEFT=true, MOTOR_INV_RIGHT=true;
bool ENCODER_INV_LEFT=false, ENCODER_INV_RIGHT=false;

#ifndef digitalReadFast
#define digitalReadFast digitalRead
#endif

/* ============================ Encoder ISRs ============================ */
void isrLeftA(){  bool A=digitalReadFast(ENC_L_A), B=digitalReadFast(ENC_L_B); encL += (A==B)? +1:-1; }
void isrLeftB(){  bool A=digitalReadFast(ENC_L_A), B=digitalReadFast(ENC_L_B); encL += (A!=B)? +1:-1; }
void isrRightA(){ bool A=digitalReadFast(ENC_R_A), B=digitalReadFast(ENC_R_B); encR += (A==B)? +1:-1; }
void isrRightB(){ bool A=digitalReadFast(ENC_R_A), B=digitalReadFast(ENC_R_B); encR += (A!=B)? +1:-1; }

/* ============================== Helpers ============================== */
inline int clamp255(int x){ return x<0?0:(x>255?255:x); }
long getRawL(){ noInterrupts(); long v=encL; interrupts(); return v; }
long getRawR(){ noInterrupts(); long v=encR; interrupts(); return v; }
long getCountL(){ long v=getRawL(); return ENCODER_INV_LEFT? -v : v; }
long getCountR(){ long v=getRawR(); return ENCODER_INV_RIGHT? -v : v; }

void setDirLeft(bool f){ bool d=f ^ MOTOR_INV_LEFT;  digitalWrite(IN1,d?HIGH:LOW); digitalWrite(IN2,d?LOW:HIGH); }
void setDirRight(bool f){ bool d=f ^ MOTOR_INV_RIGHT; digitalWrite(IN3,d?HIGH:LOW); digitalWrite(IN4,d?LOW:HIGH); }
void setPWM(int l, int r){ analogWrite(ENA,clamp255(l)); analogWrite(ENB,clamp255(r)); }
void stopMotors(){ analogWrite(ENA,0); analogWrite(ENB,0); digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); }

/* ==================== One-time motor/encoder polarity probe ================= */
void autoConfigure(){
#if DEBUG
  Serial.println(F("\n[AUTOCONFIG] Probing polarity..."));
#endif
  long l0=getRawL(), r0=getRawR();

  setDirLeft(true); setDirRight(true);
  setPWM(170,170);
  delay(500);
  stopMotors();
  delay(120);

  long dL=getRawL()-l0, dR=getRawR()-r0;

  // If ticks decreased while moving forward, flip encoder polarity
  ENCODER_INV_LEFT  = (dL < 0);
  ENCODER_INV_RIGHT = (dR < 0);

#if DEBUG
  Serial.print(F("[AUTOCONFIG] dL=")); Serial.print(dL);
  Serial.print(F(" dR=")); Serial.print(dR);
  Serial.print(F("  ENC_INV(L/R)=")); Serial.print(ENCODER_INV_LEFT); Serial.print('/'); Serial.println(ENCODER_INV_RIGHT);
#endif
}

/* =============== Dual-start gate: ensure both encoders begin ticking =============== */
void ensureBothEncodersStarted(bool forward){
  setDirLeft(forward);
  setDirRight(forward);

  long L0 = getCountL();
  long R0 = getCountR();

  // Initial kick to wake both
  setPWM(START_KICK_PWM, START_KICK_PWM);

  unsigned long t0 = millis();
  bool lStarted=false, rStarted=false;

  while(true){
    unsigned long now = millis();
    if(now - t0 > (unsigned long)START_MAX_WAIT_MS) break; // guard against stalling forever

    long dL = labs(getCountL() - L0);
    long dR = labs(getCountR() - R0);
    lStarted = (dL >= START_TICKS_THRESH);
    rStarted = (dR >= START_TICKS_THRESH);

    if(lStarted && rStarted) break;

    // If one started and the other hasn't, favour the stuck side
    if(!lStarted && rStarted){
      setPWM(START_KICK_PWM, START_HOLD_PWM);     // push left, hold right slow
    } else if(lStarted && !rStarted){
      setPWM(START_HOLD_PWM, START_KICK_PWM);     // hold left slow, push right
    } else {
      // neither started yet — keep kicking both
      setPWM(START_KICK_PWM, START_KICK_PWM);
    }

    delay(START_WINDOW_MS);
  }
}

/* =================== Core straight controller (ratio MA) =================== */
/* Drives forward/backward until 'avg ticks' reaches target. */
void driveStraightToTicks(long targetTicksAbs, bool forward){
  // NEW: make sure both wheels actually begin moving together
  ensureBothEncodersStarted(forward);

  setDirLeft(forward);
  setDirRight(forward);

  long L0=getCountL(), R0=getCountR();
  long lastL=L0, lastR=R0;

  long bufL[MA_N]={0}, bufR[MA_N]={0};
  long sumL=0, sumR=0;
  int  idx=0, filled=0;

  unsigned long tlast=0, plast=0;

  while(true){
    unsigned long now=millis();
    if(now - tlast < (unsigned long)CTRL_DT_MS) continue;
    tlast=now;

    long L=getCountL(), R=getCountR();
    long dL=L-lastL, dR=R-lastR;
    lastL=L; lastR=R;

    // moving average (on deltas)
    sumL -= bufL[idx]; sumR -= bufR[idx];
    bufL[idx]=dL;      bufR[idx]=dR;
    sumL += bufL[idx]; sumR += bufR[idx];
    idx=(idx+1)%MA_N; if(filled<MA_N) filled++;

    long sL=sumL, sR=sumR;
    float aL = fabsf((float)sL);
    float aR = fabsf((float)sR);

    float rel;
    if (aL==0.0f && aR==0.0f) rel=0.0f;
    else if (aL==0.0f)        rel= REL_CLAMP;         // right moved, left didn’t
    else {
      rel = (aR / aL) - 1.0f;                         // >0: right faster
      if(rel >  REL_CLAMP) rel =  REL_CLAMP;
      if(rel < -REL_CLAMP) rel = -REL_CLAMP;
    }

    int balance = (int)roundf(K_RATIO * rel);
    if(balance >  BAL_CLAMP) balance =  BAL_CLAMP;
    if(balance < -BAL_CLAMP) balance = -BAL_CLAMP;

    // +balance => right faster => push left up, right down
    int pwmL = max(MIN_PWM, BASE_L + balance);
    int pwmR = max(MIN_PWM, BASE_R - balance);
    setPWM(pwmL, pwmR);

#if DEBUG
    if(now - plast >= (unsigned long)PRINT_MS){
      plast = now;
      float dLma = (filled>0) ? (float)sumL / (float)filled : 0.0f;
      float dRma = (filled>0) ? (float)sumR / (float)filled : 0.0f;
      Serial.print(F("[S] t=")); Serial.print(now);
      Serial.print(F(" L=")); Serial.print(L-L0);
      Serial.print(F(" dL=")); Serial.print(dL);
      Serial.print(F(" R=")); Serial.print(R-R0);
      Serial.print(F(" dR=")); Serial.print(dR);
      Serial.print(F(" dL5=")); Serial.print(dLma,2);
      Serial.print(F(" dR5=")); Serial.print(dRma,2);
      Serial.print(F(" rel=")); Serial.print(rel,4);
      Serial.print(F(" bal=")); Serial.print(balance);
      Serial.print(F(" pwmL/R=")); Serial.print(pwmL); Serial.print('/'); Serial.println(pwmR);
    }
#endif

    long doneL = labs(L - L0);
    long doneR = labs(R - R0);
    long done  = (doneL + doneR)/2;
    if (done >= targetTicksAbs) break;
  }
  stopMotors();
}

/* Convenience: meters forward(+)/back(-) using the ratio controller */
void driveMeters(float meters){
  bool forward = meters >= 0.0f;
  if (meters < 0) meters = -meters;
  long target = (long)(meters * ticksPerMeter());
  driveStraightToTicks(target, forward);
  delay(150);
}

/* ===================== Turn in place (deg>0=CCW, <0=CW) ===================== */
void turnInPlace(float deg){
  float arc_m = (PI * WHEEL_BASE_M) * (fabs(deg) / 360.0f);   // per-wheel arc
  long  target = (long)(arc_m * ticksPerMeter());

  bool leftFwd  = (deg < 0.0f);  // CW (right turn): left fwd, right back
  bool rightFwd = (deg > 0.0f);  // CCW (left turn):  right fwd, left back
  setDirLeft(leftFwd);
  setDirRight(rightFwd);

  long L0=getCountL(), R0=getCountR();

  // Kick to overcome static friction
  setPWM(TURN_KICK_PWM, TURN_KICK_PWM);
  delay(TURN_KICK_MS);

  // Steady power until target ticks reached on either wheel
  while (true){
    long L = labs(getCountL() - L0);
    long R = labs(getCountR() - R0);
    long done = max(L, R);
    setPWM(TURN_PWM_L, TURN_PWM_R);
    if (done >= target) break;
  }
  stopMotors();
  delay(200);
}

/* ===== Ram forward exactly N meters, then instantly reverse N meters ===== */
void ramAndReverse(float meters){
  if (meters < 0) meters = -meters;
  long target = (long)(meters * ticksPerMeter());

  // Forward to target (ratio MA)
  driveStraightToTicks(target, /*forward=*/true);

  // RAM push
  setPWM(RAM_KICK_PWM, RAM_KICK_PWM);
  delay(RAM_KICK_MS);

  // INSTANT reverse same distance (ratio MA)
  driveStraightToTicks(target, /*forward=*/false);
}

/* =============================== Setup =============================== */
void setup(){
  delay(2000);
#if DEBUG
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== Track Runner (ratio MA + auto polarity + dual-start) ==="));
#endif
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  stopMotors();

  pinMode(ENC_L_A,INPUT_PULLUP); pinMode(ENC_L_B,INPUT_PULLUP);
  pinMode(ENC_R_A,INPUT_PULLUP); pinMode(ENC_R_B,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrLeftB,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrRightB, CHANGE);

  autoConfigure();

#if DEBUG
  Serial.print(F("[PARAMS] BASE_L=")); Serial.print(BASE_L);
  Serial.print(F(" BASE_R=")); Serial.print(BASE_R);
  Serial.print(F(" MIN_PWM=")); Serial.print(MIN_PWM);
  Serial.print(F(" K_RATIO=")); Serial.print(K_RATIO,1);
  Serial.print(F(" REL_CLAMP=")); Serial.print(REL_CLAMP,2);
  Serial.print(F(" BAL_CLAMP=")); Serial.print(BAL_CLAMP);
  Serial.print(F(" MA_N=")); Serial.print(MA_N);
  Serial.print(F(" CTRL_DT_MS=")); Serial.print(CTRL_DT_MS);
  Serial.print(F(" PRINT_MS=")); Serial.println(PRINT_MS);
  Serial.print(F(" ticks/m = ")); Serial.println(ticksPerMeter());
#endif
}

/* ================================ Main ================================
   Scripted path:

   0.70 → Right 90 → 1.55 → Right 90 → 0.55 → reverse 0.55 →
   Right 60 → 0.55 → reverse 0.225 → stop
*/
void loop(){
  // 1) Forward 100 cm
  driveMeters(1.00f * DIST_CORR);


  // 3) Forward 155 cm
  //driveMeters(3.20f * DIST_CORR);

  // 4) Right 90°
  //turnInPlace(-90.0f * TURN_CORR);

  // 5) Forward 55 cm
  //driveMeters(0.25f * DIST_CORR);

  // 6) Reverse 55 cm
  //driveMeters(-0.30f * DIST_CORR);

  // 7) Right 60°
  //turnInPlace(67.5f * TURN_CORR);

  // 8) Forward 55 cm
  //driveMeters(1.550f * DIST_CORR);

  // 9) Reverse 22.5 cm
  //driveMeters(-0.20f * DIST_CORR);

  // STOP!!!
  while (1) { stopMotors(); delay(1000); }
}
