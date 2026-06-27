#include <Wire.h> // Library for I2C communication

float pitch_rate; // Gyro rates

float x_acceleration, y_acceleration, z_acceleration;
float pitch_angle;

float pitch_kalman_angle = 0;
float pitch_kalman_angle_uncertainty = 2 * 2;
float kalman_output[] = {0, 0};
float error_pitch_angle;

uint32_t loop_timer;

void IMU() {

Wire.beginTransmission(0x68); // MPU6050 Adress
Wire.write(0x1A); // Low-Pass Filter
Wire.write(0x05); // Filter Bandwidth Frequency (10Hz)
Wire.endTransmission();

Wire.beginTransmission(0x68); // MPU6050 Adress
Wire.write(0x1B); // Sensitivity
Wire.write(0x8); // Sensitivity value (+-500 Degree/s <--> 65.5 LSB/s)
Wire.endTransmission();

Wire.beginTransmission(0x68); // MPU6050 Adress
Wire.write(0x43);
Wire.endTransmission();

Wire.requestFrom(0x68, 6); // Reading Gyro registers

int16_t gyro_roll = Wire.read() << 8 | Wire.read(); // Combining registers for Roll
int16_t gyro_pitch = Wire.read() << 8 | Wire.read(); // Combining registers for Pitch
int16_t gyro_yaw = Wire.read() << 8 | Wire.read(); // Combining registers for Yaw

pitch_rate = (float)gyro_yaw / 65.5; // Converting to Degree/s

//roll_rate += 0.04;
//pitch_rate += 3.93;
//yaw_rate -= 0.37;

//Accelerometer
Wire.beginTransmission(0x68); // MPU6050 Adress
Wire.write(0x1C); // Accelerometer Range
Wire.write(0x10); // Range Value (+-8G)
Wire.endTransmission();

Wire.beginTransmission(0x68); // MPU6050 Adress
Wire.write(0x3B);
Wire.endTransmission();
Wire.requestFrom(0x68, 6);

int16_t x_acceleration_lsb = Wire.read() << 8 | Wire.read();
int16_t y_acceleration_lsb = Wire.read() << 8 | Wire.read();
int16_t z_acceleration_lsb = Wire.read() << 8 | Wire.read();

x_acceleration = (float)x_acceleration_lsb / 4096 - 0.035;
y_acceleration = (float)y_acceleration_lsb / 4096 + 0.000;
z_acceleration = (float)z_acceleration_lsb / 4096 + 0.035;

pitch_angle = -atan(x_acceleration / sqrt(z_acceleration * z_acceleration + y_acceleration * y_acceleration)) * 1 / (3.142 / 180);}

void Kalman(float kalman_state, float kalman_uncertainty, float kalman_input, float kalman_measurement) {

kalman_state = kalman_state + 0.004 * kalman_input;
kalman_uncertainty = kalman_uncertainty + 0.004 * 0.004 * 0.5 * 0.5;

float kalman_gain = kalman_uncertainty * 1 / (1 * kalman_uncertainty + 0.5 * 0.5);
kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state);
kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty;
kalman_output[0] = kalman_state;
kalman_output[1] = kalman_uncertainty;
}

void setup() {

Serial.begin(115200);

Wire.setClock(400000);
Wire.begin(21, 22);
delay(500);
Wire.beginTransmission(0x68);
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission();

loop_timer = micros();
}

void loop() {
  
IMU();

Kalman(pitch_kalman_angle, pitch_kalman_angle_uncertainty, pitch_rate, pitch_angle);
pitch_kalman_angle = kalman_output[0];
pitch_kalman_angle_uncertainty = kalman_output[1];

Serial.print("Pitch: ");
Serial.print(pitch_kalman_angle);
Serial.print("   |    ");
Serial.print(0);
Serial.print("   |    ");
Serial.print(180);
Serial.print("   |    ");
Serial.println(-180);

while (micros() - loop_timer < 4000); // овде треба променити дилеј у зависности од фреквенције коју будемо одредили
loop_timer = micros();
}
