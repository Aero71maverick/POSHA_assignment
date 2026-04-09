#include <Wire.h>
#include <HX711.h>
#include <Adafruit_INA219.h>

// PCB: INA219 at 0x40 = servo, 0x41 = DC motor (A0 pin tied to VCC)
Adafruit_INA219 ina_servo(0x40);
Adafruit_INA219 ina_motor(0x41);

#define HX_DT  3
#define HX_SCK 2
#define SERVO_PIN 9
#define RPWM 10
#define LPWM 11
#define R_EN  7
#define L_EN  8

HX711 cells;
float a_scale = 1.0, b_scale = 1.0, a_off = 0, b_off = 0;
const float ARM_R = 0.05;

// PI parameters (same as module version)
float tau_cmd = 0.0;
const float KT_M = 0.025;
const float Kp   = 8.0, Ki = 0.5;
float iErr = 0.0;
const float I_MAX = 3.5;

uint16_t readAS5600() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(0x36, 2);
  return (((uint16_t)Wire.read() << 8) | Wire.read()) & 0x0FFF;
}

void setMotorPwm(int p) {
  if (p >= 0) { analogWrite(RPWM, constrain(p,0,255)); analogWrite(LPWM,0); }
  else         { analogWrite(RPWM,0); analogWrite(LPWM, constrain(-p,0,255)); }
}

float runPI(float dt) {
  float I_act = ina_motor.getCurrent_mA() / 1000.0f; // convert to A
  float I_cmd = constrain(tau_cmd / KT_M, 0.0f, I_MAX);
  float err = I_cmd - I_act;
  iErr = constrain(iErr + err * dt, -50.0f, 50.0f);
  setMotorPwm((int)constrain(Kp * err + Ki * iErr, -255, 255));
  return I_act;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(RPWM,OUTPUT); pinMode(LPWM,OUTPUT);
  pinMode(R_EN,OUTPUT); pinMode(L_EN,OUTPUT);
  digitalWrite(R_EN,HIGH); digitalWrite(L_EN,HIGH);

  cells.begin(HX_DT, HX_SCK);
  ina_servo.begin();
  ina_motor.begin();

  cells.set_gain(128); a_off = cells.read_average(20);
  cells.set_gain(32);  b_off = cells.read_average(20);

  Serial.println("t_ms,enc,dps,F1,F2,tau_meas,sA,smV,mA,mmV,tau_cmd");
}

float pA = 0; unsigned long pT = 0, lastStep = 0;

void loop() {
  unsigned long t = millis();
  float dt = pT ? (t - pT) / 1000.0f : 0.05f;

  uint16_t enc = readAS5600();
  float ang = enc * (360.0f / 4096.0f);
  float dps = 0;
  if (pT) {
    float da = ang - pA;
    if (da > 180) da -= 360; if (da < -180) da += 360;
    dps = da / dt;
  }
  pA = ang; pT = t;

  cells.set_gain(128); float ra = cells.read();
  cells.set_gain(32);  float rb = cells.read();
  float F1  = (ra - a_off) * a_scale;
  float F2  = (rb - b_off) * b_scale;
  float tau = (F1 - F2) * ARM_R;

  float sA  = ina_servo.getCurrent_mA();
  float sV  = ina_servo.getBusVoltage_V() * 1000;
  float mA  = runPI(dt);
  float mV  = ina_motor.getBusVoltage_V() * 1000;

  if (t - lastStep > 500) {
    tau_cmd = min(tau_cmd + 0.05f, 1.5f);
    lastStep = t;
  }

  Serial.print(t);    Serial.print(',');
  Serial.print(enc);  Serial.print(',');
  Serial.print(dps,2);Serial.print(',');
  Serial.print(F1,4); Serial.print(',');
  Serial.print(F2,4); Serial.print(',');
  Serial.print(tau,4);Serial.print(',');
  Serial.print(sA,2); Serial.print(',');
  Serial.print(sV,1); Serial.print(',');
  Serial.print(mA,3); Serial.print(',');
  Serial.print(mV,1); Serial.print(',');
  Serial.println(tau_cmd,3);
  delay(50);
}