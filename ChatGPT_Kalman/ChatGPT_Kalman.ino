#include <Wire.h>

#define MPU 0x68
#define T 0.004  // 250 Hz

// ===== RAW =====
float gz;
float ax, ay, az;

// ===== ANGLES =====
float angle = 0;          // Kalman output (Z rotation / yaw-like)
float accel_angle = 0;

// ===== KALMAN STATE =====
float kalman_angle = 0;
float kalman_uncertainty = 4.0;

// ===== BIAS =====
float gyro_bias = 0.0;

// ===== TIMER =====
uint32_t last_time;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // wake MPU
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  // gyro ±500 dps
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(1 << 3);
  Wire.endTransmission();

  // accel ±8g
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  delay(1000);
  last_time = micros();
}

// ===== READ SENSOR =====
void readIMU() {

  // ---- GYRO ----
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);

  int16_t gx_raw = Wire.read() << 8 | Wire.read();
  int16_t gy_raw = Wire.read() << 8 | Wire.read();
  int16_t gz_raw = Wire.read() << 8 | Wire.read();

  gz = gz_raw / 65.5;

  // ---- ACCEL ----
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();

  ax = ax_raw / 4096.0;
  ay = ay_raw / 4096.0;
  az = az_raw / 4096.0;

  // ===== YOUR ORIENTATION FIX =====
  // rotation around Z axis → use XY plane
  accel_angle = atan2(ay, ax) * 180.0 / PI;
}

// ===== KALMAN =====
void kalmanUpdate(float &state, float &uncertainty, float gyro_rate, float accel_meas) {

  // prediction (gyro integration)
  state = state + T * (gyro_rate - gyro_bias);
  uncertainty = uncertainty + 0.01;

  // measurement update
  float K = uncertainty / (uncertainty + 3.0);

  state = state + K * (accel_meas - state);
  uncertainty = (1 - K) * uncertainty;
}

void loop() {

  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000;

    readIMU();

    kalmanUpdate(kalman_angle, kalman_uncertainty, gz, accel_angle);

    Serial.println(kalman_angle);
  }
}
