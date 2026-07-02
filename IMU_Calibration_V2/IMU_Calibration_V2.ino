#include <Wire.h>

#define MPU 0x68
#define T 0.004  // 250 Hz (Vreme uzorkovanja u sekundama)

unsigned long last_time = 0;

// RAW DATA
float gy;
float ax, ay, az;

// ANGLES
float accel_angle = 0;
float gyro_angle = 0;

// BIAS (Ovo prepisuješ iz rezultata kalibracije)
float gyro_bias = 0.0; 

// STATS variables
float accel_avg = 0;
float M2_accel = 0; // Za tačan proračun varijanse

float gyro_avg = 0;

float fusion_error_sum = 0;
float fusion_error_sq = 0;

int samples = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Wake MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyro ±500 dps
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Accel ±8g
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Serial.println("IMU kalibracija i testiranje započeti...");
  delay(2000);

  last_time = micros();
}

void readIMU() {
  // ===== ČITANJE SVIH PODATAKA ODJEDNOM (Mnogo brže i sinhronizovano) =====
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Počinjemo od ACCEL_XOUT_H
  Wire.endTransmission();

  // Tražimo 14 bajtova (6 za Accel, 2 za Temp, 6 za Gyro)
  Wire.requestFrom(MPU, 14);

  int16_t axr = Wire.read() << 8 | Wire.read();
  int16_t ayr = Wire.read() << 8 | Wire.read();
  int16_t azr = Wire.read() << 8 | Wire.read();
  
  int16_t temp = Wire.read() << 8 | Wire.read(); // Preskačemo temperaturu
  
  int16_t gxr = Wire.read() << 8 | Wire.read();
  int16_t gyr = Wire.read() << 8 | Wire.read();
  int16_t gzr = Wire.read() << 8 | Wire.read();

  // Konverzija u G-jedinice (±8g skala -> delimo sa 4096.0)
  ax = axr / 4096.0;
  ay = ayr / 4096.0;
  az = azr / 4096.0;

  // !!! PAŽNJA: Proveri ovde koja ti je osa žiroskopa prava za nagib !!!
  // Ako robot balansira napred-nazad, a senzor je uspravan, rotacija je obično oko Z ose (gzr) ili Y ose (gyr)
  // Za sada ostavljamo GYRO_Y (gyr), ali ako integrisani ugao "ne prati" akcelerometar, promeni ovde u gzr
  gy = -gyr / 65.5 -0.3453865; 

  // ===== TVOJA TRIGONOMETRIJSKA FORMULA =====
  // Koristi ax kao glavnu vertikalnu osu, a ay i az za kompenzaciju bočnog nagiba
  accel_angle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  
  // Opciono: Ako ti uspravno i dalje daje npr. 90 stepeni, otkomentariši liniju ispod da ga resetuješ na 0:
  // accel_angle -= 90.0; 
}

void loop() {

  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000;

    readIMU();

    // Integracija žiroskopa (brzina * vreme)
    gyro_angle += (gy - gyro_bias) * T;

    samples++;

    // Welford-ov algoritam za tačnu running varijansu i srednju vrednost akcelerometra
    float delta = accel_angle - accel_avg;
    accel_avg += delta / samples;
    float delta2 = accel_angle - accel_avg;
    M2_accel += delta * delta2;

    // Srednja vrednost žiroskopa (odlično za pronalaženje pravog biasa/offseta)
    gyro_avg += gy;

    // Razlika između žiroskopa i akcelerometra (merenje drifta)
    float fusion_error = gyro_angle - accel_angle;
    fusion_error_sum += fusion_error;
    fusion_error_sq += fusion_error * fusion_error;

    // Na svakih 250 uzoraka (tačno 1 sekunda) izbaci statistiku na Serial Monitor
    if (samples % 250 == 0) {

      float accel_std = sqrt(M2_accel / (samples - 1)); // Standardna devijacija šuma akcelerometra

      float gyro_drift = gyro_avg / samples; // Ovo je tvoj stvarni Gyro Bias!

      float fusion_mean = fusion_error_sum / samples;
      float fusion_std = sqrt(fusion_error_sq / samples - fusion_mean * fusion_mean);

      Serial.println("===== IMU STATISTIKA I KALIBRACIJA =====");
      Serial.print("Ugao Akcelerometra (Srednji): "); Serial.print(accel_avg, 4); Serial.println(" °");
      Serial.print("Šum Akcelerometra (StdDev): "); Serial.print(accel_std, 4); Serial.println(" (manje je bolje)");
      Serial.print("Izmereni Gyro Bias (Offset): "); Serial.print(gyro_drift, 6); Serial.println(" dps");
      Serial.print("Integrisani ugao Žiroskopa: "); Serial.print(gyro_angle, 4); Serial.println(" °");
      Serial.print("Odstupanje Žiroskop-Akselerometar: "); Serial.print(fusion_std, 4); Serial.println(" °");
      Serial.println("========================================");
    }
  }
}
