#include <Wire.h>

#define MPU 0x68
#define T 0.004  // 250 Hz

unsigned long last_time = 0;

// RAW
float gx, gy, gz;
float ax, ay, az;

// ANGLES
float accel_angle = 0;
float gyro_angle = 0;
float angle = 0;

// BIAS (drift)
float gyro_bias = 0;

// STATS
float accel_var = 0;
float gyro_drift = 0;

float accel_avg = 0;
float gyro_avg = 0;

int samples = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // wake MPU6050
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

  Serial.println("IMU calibration started... keep robot STILL");
  delay(2000);

  last_time = micros();
}

void readIMU() {

  // GYRO Z (pretpostavka pitch ose - možeš promeniti)
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 2);

  int16_t gyro_raw = Wire.read() << 8 | Wire.read();
  gy = gyro_raw / 65.5 + 0.02 - 0.00483;

  // ACCEL
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

  accel_angle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

void loop() {

  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000;

    readIMU();

    // gyro integration
    gyro_angle += (gy - gyro_bias) * T;

    // stats
    samples++;

    float accel_error = accel_angle - accel_avg;
    accel_avg += accel_error / samples;

    accel_var += accel_error * accel_error;

    gyro_avg += gy;

    // drift estimation (simple)
    gyro_drift = gyro_avg / samples;

    // print every ~250 samples (~1 sec)
    if (samples % 250 == 0) {

      float accel_std = sqrt(accel_var / samples);

      Serial.println("===== IMU STATS =====");
      Serial.print("Accel angle avg: "); Serial.println(accel_avg);
      Serial.print("Accel noise (std): "); Serial.println(accel_std);
      Serial.print("Gyro bias (approx): "); Serial.println(gyro_drift);
      Serial.print("Gyro integrated angle: "); Serial.println(gyro_angle);
      Serial.println("=====================");
    }
  }
}
