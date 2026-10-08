/*
  Cart-Pendulum LQR DRY RUN
  Arduino Mega 2560
  Motor output permanently disabled.
  Simulation K + FD/LPF 5Hz + deadband 5.
*/

#include <Arduino.h>
#include <math.h>

const uint8_t PWM_PIN = 9;
const uint8_t DIR_PIN = 8;
const uint8_t ENC_A = 3;
const uint8_t ENC_B = 2;
const uint8_t POT_PIN = A0;

const unsigned long PERIOD_US = 5000UL;
const unsigned long PRINT_US = 50000UL;
const float DT = 0.005f;
const float PI_F = 3.14159265358979323846f;
const float RAD = PI_F / 180.0f;

const float METERS_PER_COUNT = 0.00010576f;
const float POT_UP = 50.0f;
const float POT_DOWN = 570.0f;
const float ANGLE_SIGN = -1.0f;

const float K[4] = {
  -507.0928f,
  -831.9818f,
  2346.2137f,
  370.6610f
};

const float ALPHA = 0.145364f; // 1-exp(-2*pi*5*0.005)
const float DEADZONE = 47.0f;
const float DEADBAND = 5.0f;
const float PWM_LIMIT = 150.0f;

volatile long encoderCount = 0;

void isrA() {
  encoderCount +=
    (digitalRead(ENC_A) == digitalRead(ENC_B)) ? 1 : -1;
}

void isrB() {
  encoderCount +=
    (digitalRead(ENC_A) != digitalRead(ENC_B)) ? 1 : -1;
}

long readEncoder() {
  noInterrupts();
  long value = encoderCount;
  interrupts();
  return value;
}

void zeroEncoder() {
  noInterrupts();
  encoderCount = 0;
  interrupts();
}

void motorOff() {
  // Disconnect timer PWM outputs and force D8/D9 LOW.
  TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
  TCCR4A &= ~(_BV(COM4C1) | _BV(COM4C0));
  PORTH &= ~(_BV(PH5) | _BV(PH6));
  DDRH |= (_BV(PH5) | _BV(PH6));
}

void earlyMotorSafe()
  __attribute__((naked, used, section(".init3")));

void earlyMotorSafe() {
  PORTH &= ~(_BV(PH5) | _BV(PH6));
  DDRH |= (_BV(PH5) | _BV(PH6));
}

float wrapPi(float angle) {
  while (angle > PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

float readTheta() {
  int raw = analogRead(POT_PIN);
  float deg = ANGLE_SIGN *
    (raw - POT_UP) * 180.0f / fabsf(POT_DOWN - POT_UP);
  return wrapPi(deg * RAD);
}

float xPrev = 0;
float thetaPrev = 0;
float vHat = 0;
float omegaHat = 0;
float xHat = 0;
float thetaHat = 0;
float pwmVirtual = 0;
float pwmDelayed = 0;

unsigned long nextControl = 0;
unsigned long lastPrint = 0;
unsigned long overruns = 0;

void resetEstimator() {
  xHat = readEncoder() * METERS_PER_COUNT;
  thetaHat = readTheta();
  xPrev = xHat;
  thetaPrev = thetaHat;
  vHat = 0;
  omegaHat = 0;
  pwmVirtual = 0;
  pwmDelayed = 0;
}

void controlStep() {
  xHat = readEncoder() * METERS_PER_COUNT;
  thetaHat = readTheta();

  float dx = (xHat - xPrev) / DT;
  float dtheta = wrapPi(thetaHat - thetaPrev) / DT;

  vHat += ALPHA * (dx - vHat);
  omegaHat += ALPHA * (dtheta - omegaHat);

  xPrev = xHat;
  thetaPrev = thetaHat;

  float request = -(
    K[0] * xHat +
    K[1] * vHat +
    K[2] * thetaHat +
    K[3] * omegaHat
  );

  float desired = 0;

  if (fabsf(request) > DEADBAND) {
    desired = copysignf(fabsf(request) + DEADZONE, request);
  }

  desired = constrain(desired, -PWM_LIMIT, PWM_LIMIT);

  // Virtual one-sample delay, matching simulation.
  pwmVirtual = pwmDelayed;
  pwmDelayed = desired;

  // DRY RUN: no motor actuation.
  motorOff();
}

void setup() {
  motorOff();
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  motorOff();

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  Serial.begin(250000);
  resetEstimator();

  nextControl = micros() + PERIOD_US;
  lastPrint = micros();

  Serial.println(F("# LQR DRY RUN - MOTOR PERMANENTLY OFF"));
  Serial.println(F("# Commands: z=zero encoder, r=reset estimator"));
  Serial.println(F("# t_ms,enc,raw,x_m,v_mps,theta_deg,omega_degps,pwm_virtual,overruns"));
}

void loop() {
  motorOff();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      zeroEncoder();
      resetEstimator();
      Serial.println(F("# ENCODER ZERO"));
    } else if (c == 'r') {
      resetEstimator();
      Serial.println(F("# ESTIMATOR RESET"));
    }
  }

  unsigned long now = micros();

  if ((long)(now - nextControl) >= 0) {
    if ((unsigned long)(now - nextControl) >= PERIOD_US) {
      overruns++;
      nextControl = now + PERIOD_US;
    } else {
      nextControl += PERIOD_US;
    }
    controlStep();
  }

  if ((unsigned long)(now - lastPrint) >= PRINT_US) {
    lastPrint = now;

    Serial.print(millis());
    Serial.print(',');
    Serial.print(readEncoder());
    Serial.print(',');
    Serial.print(analogRead(POT_PIN));
    Serial.print(',');
    Serial.print(xHat, 5);
    Serial.print(',');
    Serial.print(vHat, 4);
    Serial.print(',');
    Serial.print(thetaHat / RAD, 3);
    Serial.print(',');
    Serial.print(omegaHat / RAD, 3);
    Serial.print(',');
    Serial.print(pwmVirtual, 2);
    Serial.print(',');
    Serial.println(overruns);
  }
}
