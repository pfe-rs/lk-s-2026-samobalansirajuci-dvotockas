#include <Wire.h> 
#include <ESP32Encoder.h> 
#include <Bluepad32.h>


// General -- 

#define Battery_Pin 36
#define Buzzer_Pin 12

#define F 500 // System frequency 
#define T 0.002 // System period 

unsigned long last_time = 0; 
float battery_voltage;
int battery_check_counter;

ControllerPtr controller = nullptr;


// Drive Control -- 

#define Dead_Zone 5
#define Wheel_Base 0.20

int throttle;
int steering;
int rotational_speed;


// Motor Control -- 

ESP32Encoder left_encoder; 
ESP32Encoder right_encoder; 

#define LApin 32
#define LBpin 33
#define RApin 26
#define RBpin 27

#define Left_Dir_Pin 16
#define Left_PWM_Pin 17
#define Right_Dir_Pin 0
#define Right_PWM_Pin 4

#define Steps_Per_Revolution 8192 
#define Wheel_Diameter 0.075 
#define mI_Treshold 0.75
#define mI_Limit 100

const float Meters_Per_Step = PI * Wheel_Diameter / Steps_Per_Revolution; 

int left_motor_power; 
int right_motor_power; 
int left_motor_direction; 
int right_motor_direction; 

long last_left_steps = 0; 
long last_right_steps = 0;

float PID_desired_velocity;
float left_desired_velocity, left_velocity, left_last_mI, left_last_velocity_error, left_mPID_return; 
float right_desired_velocity, right_velocity, right_last_mI, right_last_velocity_error, right_mPID_return; 

float mKp = 0; 
float mKi = 0; 
float mKd = 0; 
float mPID_return[3]; 


// Angle Control -- 

#define I2C_SDA_Pin 21
#define I2C_SCL_Pin 22

#define Gyro_Bias 0.03
#define Accelerometer_Std 30.9
#define Gyro_Drift 0.3453865
#define I_Treshold 3
#define I_Limit 10

#define Fall_Treshold 45

float angle_rate, acc_angle, angle, desired_angle, last_I, last_angle_error; 
float x_acceleration, y_acceleration, z_acceleration; 

float angle_uncertainty = 4; // 2 * 2 
float kalman_return[2]; 

float Kp = 0;
float Ki = 0;
float Kd = 0;
float PID_return[3]; 


void Encoder_Setup(){ 

  left_encoder.attachFullQuad(LApin, LBpin); 
  left_encoder.clearCount(); 
  right_encoder.attachFullQuad(RApin, RBpin); 
  right_encoder.clearCount(); 
} 

void Measure_Wheel_Velocity(){

  long left_steps = left_encoder.getCount(); 
  long right_steps = right_encoder.getCount();

  long delta_left = left_steps - last_left_steps; 
  long delta_right = right_steps - last_right_steps;

  last_left_steps = left_steps; 
  last_right_steps = right_steps; 

  left_velocity = delta_left * Meters_Per_Step * F; // ,,/T"
  right_velocity = delta_right * Meters_Per_Step * F;
} 

void Wheel_Velocity(){

  left_desired_velocity  = PID_desired_velocity + rotational_speed ;
  right_desired_velocity = PID_desired_velocity - rotational_speed;
}

void Compute_mPID(float desired_velocity, float velocity, float last_mI, float last_velocity_error){

  float velocity_error = desired_velocity - velocity; 

  float mP = velocity_error * mKp; 
  float mI = last_mI; 
  if(abs(velocity_error) < mI_Treshold){
    mI += velocity_error * T; 
  } 
  mI = constrain(mI, -mI_Limit, mI_Limit); 
  float mD = (velocity_error - last_velocity_error) * F; // ,,/T"

  mPID_return[0] = mP + mI * mKi + mD * mKd; 
  mPID_return[1] = mI; 
  mPID_return[2] = velocity_error; 
} 

void Calculate_mPID(){

  Measure_Wheel_Velocity();
  Wheel_Velocity();

  Compute_mPID(left_desired_velocity, left_velocity, left_last_mI, left_last_velocity_error);
  left_motor_power = (int)mPID_return[0]; 
  left_last_mI = mPID_return[1]; 
  left_last_velocity_error = mPID_return[2];

  Compute_mPID(right_desired_velocity, right_velocity, right_last_mI, right_last_velocity_error); 
  right_motor_power = (int)mPID_return[0]; 
  right_last_mI = mPID_return[1]; 
  right_last_velocity_error = mPID_return[2]; 
} 

void Drive_Motors(){

  if(left_motor_power < 0){ 
    left_motor_direction = LOW; 
    left_motor_power = abs(left_motor_power); 
  } 
  else{ 
    left_motor_direction = HIGH; 
  }

  if(right_motor_power < 0){ 
    right_motor_direction = LOW; 
    right_motor_power = abs(right_motor_power); 
  } 
  else{ 
    right_motor_direction = HIGH; 
  }
  
  left_motor_power = constrain(left_motor_power, 0, 255); 
  right_motor_power = constrain(right_motor_power, 0, 255);

  digitalWrite(Left_Dir_Pin, left_motor_direction); 
  digitalWrite(Right_Dir_Pin, right_motor_direction); 
  analogWrite(Left_PWM_Pin, left_motor_power); 
  analogWrite(Right_PWM_Pin, right_motor_power); 
} 

void Motor_Setup(){

  pinMode(Left_Dir_Pin, OUTPUT);
  pinMode(Left_PWM_Pin, OUTPUT);
  pinMode(Right_Dir_Pin, OUTPUT);
  pinMode(Right_PWM_Pin, OUTPUT);
}

void IMU_Setup(){ 

  Wire.begin(I2C_SDA_Pin, I2C_SCL_Pin);
  Wire.setClock(400000);

  Wire.beginTransmission(0x68); // MPU6050 Adress 
  Wire.write(0x6B); // Waking up the sensor
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // MPU6050 Adress 
  Wire.write(0x1A); // Low-Pass Filter 
  Wire.write(0x03); // Filter Bandwidth Frequency (44Hz) 
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // MPU6050 Adress 
  Wire.write(0x1B); // Sensitivity 
  Wire.write(0x08); // Sensitivity value (+-500 Degree/s <--> 65.5 LSB/s) 
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // MPU6050 Adress 
  Wire.write(0x1C); // Accelerometer Range 
  Wire.write(0x10); // Range Value (+-8G) 
  Wire.endTransmission(); 
} 

void IMU(){

  Wire.beginTransmission(0x68); // MPU6050 Adress 
  Wire.write(0x3B); 
  Wire.endTransmission();

  Wire.requestFrom(0x68, 14); // Reading sensor registers 

  int16_t x_acceleration_lsb = Wire.read() << 8 | Wire.read(); 
  int16_t y_acceleration_lsb = Wire.read() << 8 | Wire.read(); 
  int16_t z_acceleration_lsb = Wire.read() << 8 | Wire.read();

  int16_t temperature = Wire.read() << 8 | Wire.read();

  int16_t gyro_roll = Wire.read() << 8 | Wire.read();
  int16_t gyro_pitch = Wire.read() << 8 | Wire.read();
  int16_t gyro_yaw = Wire.read() << 8 | Wire.read(); // Combining registers for Pitch 

  angle_rate = (float)gyro_pitch / 65.5 - Gyro_Drift; // Converting to Degree/s 

  x_acceleration = (float)x_acceleration_lsb / 4096 - 0.035; 
  y_acceleration = (float)y_acceleration_lsb / 4096 + 0.000;
  z_acceleration = (float)z_acceleration_lsb / 4096 + 0.035; 

  acc_angle = -atan(x_acceleration / sqrt(z_acceleration * z_acceleration + y_acceleration * y_acceleration)) * 180 / PI; 
} 

void Kalman(float kalman_state, float kalman_uncertainty, float kalman_input, float kalman_measurement){ 

  kalman_state = kalman_state + T * kalman_input; 
  kalman_uncertainty = kalman_uncertainty + T * T * Gyro_Bias * Gyro_Bias;
  
  float kalman_gain = kalman_uncertainty * 1 / (1 * kalman_uncertainty + Accelerometer_Std * Accelerometer_Std);
  kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state);
  kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty;
  
  kalman_return[0] = kalman_state;
  kalman_return[1] = kalman_uncertainty;
} 

void Kalman_Calculate(){

  Kalman(angle, angle_uncertainty, angle_rate, acc_angle);
  angle = kalman_return[0];
  angle_uncertainty = kalman_return[1];
}

void Compute_PID(float desired_angle, float angle, float last_I, float last_angle_error){

  float angle_error = desired_angle - angle; 

  float P = angle_error * Kp; 
  float I = last_I; 
  if(abs(angle_error) < I_Treshold){
    I += angle_error * T; 
  } 
  I = constrain(I, -I_Limit, I_Limit); 
  float D = (angle_error - last_angle_error) * F; // ,,T"

  PID_return[0] = P + I * Ki + D * Kd; 
  PID_return[1] = I; 
  PID_return[2] = angle_error; 
}

void Calculate_PID(){

  Compute_PID(desired_angle, angle, last_I, last_angle_error);
  PID_desired_velocity = PID_return[0]; 
  last_I = PID_return[1]; 
  last_angle_error = PID_return[2];
}

bool Fall_Detection(){

  if(abs(angle) > Fall_Treshold){
    
    last_I = 0;
    last_angle_error = 0;
    
    left_last_mI = 0;
    left_last_velocity_error = 0;
    
    right_last_mI = 0;
    right_last_velocity_error = 0;

    left_motor_power = 0;
    right_motor_power = 0;
    Drive_Motors();
    
    return 0;
  }
  else{
    return 1;
  }
}

void Battery_Voltage(){

  uint16_t mv = analogReadMilliVolts(Battery_Pin);
  battery_voltage = (mv * 5.69) / 1000.0;
}

bool Battery_Protection(){

  battery_check_counter ++;
  if(battery_check_counter >= 1000){
    Battery_Voltage();
    battery_check_counter = 0;
  }
  
  if(battery_voltage > 8.75){
    if(battery_voltage < 9){
      digitalWrite(Buzzer_Pin, HIGH);
    }
    else{
      digitalWrite(Buzzer_Pin, LOW);
    }
    return 1;
  }
  else{
    left_motor_power = 0;
    right_motor_power = 0;
    Drive_Motors();
    return 0;
  }
}

void On_Connected_Controller(ControllerPtr game_pad){

  if(game_pad -> isGamepad()){
    controller = game_pad;
  }
}

void On_Disconnected_Controller(ControllerPtr game_pad){

  if(controller == game_pad){
    controller = nullptr;
  }
}

void Process_Gamepad(ControllerPtr game_pad){

  int r2_throttle = game_pad -> throttle();
  int brake = game_pad -> brake();
  int raw_steering = game_pad -> axisX();

  int raw_throttle = r2_throttle - brake;

  throttle = map(raw_throttle, -1023, 1023, -5, 5);
  steering = map(raw_steering, -512, 511, -30, 30);

  if(abs(steering) < Dead_Zone){
    steering = 0;
  }

  desired_angle = throttle;
  rotational_speed = (steering * Wheel_Base) / 2;
}

void Controller_Recieve(){

  BP32.update();

  if(controller && controller -> isConnected()){
    Process_Gamepad(controller);
  }
  else{
    throttle = 0;
    desired_angle = 0;
    rotational_speed = 0; 
  }
}

void setup() { 

  Serial.begin(115200); 

  last_time = micros();

  pinMode(Buzzer_Pin, OUTPUT);
  BP32.setup(&On_Connected_Controller, &On_Disconnected_Controller);
 
  IMU_Setup();
  Motor_Setup();
  Encoder_Setup();
} 

void loop() { 

  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000; 

    Controller_Recieve();
    
    IMU();
    Kalman_Calculate();
     
    if(Fall_Detection() && Battery_Protection()){
      
      Calculate_PID();  
      Calculate_mPID();
      Drive_Motors();
    }
  }
} 
