#include <Wire.h>

#define MPU 0x68
#define T 0.004  // 250Hz

unsigned long last_time = 0;

// raw
float gx, gy, gz;
float ax, ay, az;

// angles
float gyro_angle = 0;
float accel_angle = 0;

// bias (set 0 for now)
float gyro_bias = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // wake MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // gyro ±500 dps
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // accel ±8g
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  delay(2000);
  last_time = micros();
}

void readIMU() {

  // ===== GYRO (0x47) =====
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission();

  Wire.requestFrom(MPU, 2);
  int16_t gyro_raw = Wire.read() << 8 | Wire.read();

  gy = - gyro_raw / 65.5 + 0.02 - 0.00483 + 0.000097 + 0.000011/60;   // deg/s

  // ===== ACCEL =====
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(MPU, 6);

  int16_t axr = Wire.read() << 8 | Wire.read();
  int16_t ayr = Wire.read() << 8 | Wire.read();
  int16_t azr = Wire.read() << 8 | Wire.read();

  ax = axr / 4096.0;
  ay = ayr / 4096.0;
  az = azr / 4096.0;

  // pitch from accel
  accel_angle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

void loop() {

  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000;

    readIMU();

    // gyro integration
    gyro_angle += (gy - gyro_bias) * T;

    // ===== SERIAL PLOTTER OUTPUT =====
    Serial.print(accel_angle);
    Serial.print(" ");
    Serial.println(gyro_angle);
  }
}
