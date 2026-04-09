#include <Wire.h>
#include <HX711.h>
#include <Adafruit_INA219.h>
#include <Adafruit_ADS1X15.h>

// --- Pin definitions ---
#define SERVO_PIN   9
#define RPWM       10   // BTS7960 forward PWM
#define LPWM       11   // BTS7960 reverse PWM
#define R_EN        7
#define L_EN        8
#define HX_DT       3
#define HX_SCK      2

// --- I2C devices ---
Adafruit_INA219 ina219(0x40);  // servo current
Adafruit_ADS1115 ads;          // ACS712 via ADS1115

// --- HX711 ---
HX711 cells;

// --- Calibration ---
float cell_a_scale  = 1.0;   // set during calibration
float cell_b_scale  = 1.0;
float cell_a_offset = 0.0;
float cell_b_offset = 0.0;
const float ARM_R   = 0.05;  // moment arm in metres

// --- Torque control (PI current loop) ---
float torque_setpoint_Nm = 0.0;
const float KT_MOTOR     = 0.025; // JGA25-370 approx Kt (N.m/A)
const float Kp_I         = 8.0;
const float Ki_I         = 0.5;
float integral_error     = 0.0;
const float I_MAX        = 3.5;   // max motor current A

// --- AS5600 raw register read ---
uint16_t readAS5600() {
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(0x36, 2);
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

// --- Motor PWM output ---
void setMotorPwm(int pwm) {
  if (pwm >= 0) {
    analogWrite(RPWM, constrain(pwm, 0, 255));
    analogWrite(LPWM, 0);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, constrain(-pwm, 0, 255));
  }
}

// --- PI current controller for load motor ---
// Call at ~20 Hz. Returns actual motor current in A.
float runMotorCurrentPI(float dt) {
  // Read motor current from ACS712 via ADS1115
  int16_t raw = ads.readADC_SingleEnded(0);
  float acs_V = raw * 0.0001875f;            // 6.144V FSR / 32768
  float I_actual = (acs_V - 2.5f) / 0.185f; // ACS712-05B: 185 mV/A, midpoint 2.5V

  float I_cmd = torque_setpoint_Nm / KT_MOTOR;
  I_cmd = constrain(I_cmd, 0.0f, I_MAX);

  float err = I_cmd - I_actual;
  integral_error += err * dt;
  integral_error = constrain(integral_error, -50.0f, 50.0f); // anti-windup

  float u = Kp_I * err + Ki_I * integral_error;
  int pwm_out = (int)constrain(u, -255, 255);
  setMotorPwm(pwm_out);

  return I_actual;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(SERVO_PIN, OUTPUT);
  pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  cells.begin(HX_DT, HX_SCK);
  ina219.begin();
  ads.setGain(GAIN_ONE);
  ads.begin();

  // Tare both channels
  cells.set_gain(128);
  cell_a_offset = cells.read_average(20);
  cells.set_gain(32);
  cell_b_offset = cells.read_average(20);

  Serial.println("t_ms,enc_raw,speed_dps,F1_N,F2_N,torque_Nm,"
                 "servo_mA,servo_mV,motor_A,tau_cmd_Nm");
}

float prevAngle = 0;
unsigned long prevTime = 0;
unsigned long lastSweepStep = 0;
const float SWEEP_STEP = 0.05;     // N.m per step
const float SWEEP_MAX  = 1.5;      // max torque
const unsigned long STEP_INTERVAL = 500; // ms between steps

void loop() {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;

  // --- Encoder ---
  uint16_t enc_raw = readAS5600();
  float angle = enc_raw * (360.0f / 4096.0f);
  float speed_dps = 0;
  if (prevTime != 0 && dt > 0) {
    float da = angle - prevAngle;
    if (da >  180) da -= 360;
    if (da < -180) da += 360;
    speed_dps = da / dt;
  }
  prevAngle = angle;
  prevTime  = now;

  // --- Load cells ---
  cells.set_gain(128); float ra = cells.read();
  cells.set_gain(32);  float rb = cells.read();
  float F1 = (ra - cell_a_offset) * cell_a_scale;
  float F2 = (rb - cell_b_offset) * cell_b_scale;
  float torque_meas = (F1 - F2) * ARM_R;

  // --- Servo current ---
  float servo_mA = ina219.getCurrent_mA();
  float servo_mV = ina219.getBusVoltage_V() * 1000.0f;

  // --- Motor PI current loop ---
  float motor_A = runMotorCurrentPI(dt);

  // --- Torque sweep (advances setpoint every STEP_INTERVAL ms) ---
  if (now - lastSweepStep > STEP_INTERVAL) {
    torque_setpoint_Nm += SWEEP_STEP;
    if (torque_setpoint_Nm > SWEEP_MAX) torque_setpoint_Nm = SWEEP_MAX;
    lastSweepStep = now;
  }

  // --- CSV output ---
  Serial.print(now);               Serial.print(',');
  Serial.print(enc_raw);           Serial.print(',');
  Serial.print(speed_dps, 2);      Serial.print(',');
  Serial.print(F1, 4);             Serial.print(',');
  Serial.print(F2, 4);             Serial.print(',');
  Serial.print(torque_meas, 4);    Serial.print(',');
  Serial.print(servo_mA, 2);       Serial.print(',');
  Serial.print(servo_mV, 1);       Serial.print(',');
  Serial.print(motor_A, 3);        Serial.print(',');
  Serial.println(torque_setpoint_Nm, 3);

  delay(50); // 20 Hz sample rate
}