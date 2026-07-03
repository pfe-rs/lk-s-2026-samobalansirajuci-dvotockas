#include <Wire.h> 
#include <ESP32Encoder.h> 
#include <Bluepad32.h>

// General -- 

#define Battery_Pin 36
#define Buzzer_Pin 12

#define F 200     // Frekvencija sistema (200Hz)
#define T 0.005   // Period sistema (5ms)

unsigned long last_time = 0; 
float battery_voltage = 12;
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
#define mI_Limit 500
#define Left_FFK 219.30  
#define Right_FFK 212.31 
#define Left_FFN 12
#define Right_FFN 15

const float Meters_Per_Step = PI * Wheel_Diameter / Steps_Per_Revolution; 

int left_motor_power; 
int right_motor_power; 
int left_motor_direction; 
int right_motor_direction; 

long last_left_steps = 0; 
long last_right_steps = 0;

float PID_desired_velocity;
float left_desired_velocity, left_velocity, left_last_mI, left_last_velocity_error; 
float right_desired_velocity, right_velocity, right_last_mI, right_last_velocity_error; 

// Donji PID (točkovi) - ostaje fiksiran jer Feedforward radi glavni posao
float mKp = 0.1; 
float mKi = 0; 
float mKd = 2; 
float mPID_return[3]; 
float mAlpha = 0.1;


// Angle Control (GLAVNI URADI SAM INTEGRISANI IMU) -- 

#define I2C_SDA_Pin 21
#define I2C_SCL_Pin 22
#define Fall_Treshold 45

// Promenljive iz tvog testiranog koda
float pitch_rate; 
float x_acceleration, y_acceleration, z_acceleration;
float pitch_angle;
float pitch_kalman_angle = 0;
float pitch_kalman_angle_uncertainty = 4; // 2 * 2
float kalman_output[] = {0, 0};

// Mapiranje na globalnu varijablu ugla koju koristi tvoj kaskadni PID
float angle; 
float base_angle = 2.5; // FIKS: Početna vrednost mehaničkog balansa koju menjaš preko terminala
float desired_angle , last_I, last_angle_error; 
#define I_Treshold 0.5
#define I_Limit 10000

// GLAVNI KOEFICIJENTI ZA BALANS - Tjunuješ ih uživo preko Serial Monitora
float Kp = 0.025;
float Ki = 1.3; // 0.75
float Kd = 0.0;
float PID_return[3]; 


// Position Control --

float robot_position;
float K_vel = 0.01;
float K_pos = 0.01;


// Prijem komandi preko serijale uživo u toku rada (npr. v0.02 ili s0.05)
void Proveri_Serial_Komande() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    if (input.length() > 1) {
      char komanda = input.charAt(0);
      float vrednost = input.substring(1).toFloat();
        
      if (komanda == 'p' || komanda == 'P') {
        Kp = vrednost;
        Serial.print(">>> Glavni Kp (Balans) postavljen na: "); Serial.println(Kp, 5);
      } 
      else if (komanda == 'i' || komanda == 'I') {
        Ki = vrednost;
        Serial.print(">>> Glavni Ki (Balans) postavljen na: "); Serial.println(Ki, 5);
      }
      else if (komanda == 'd' || komanda == 'D') {
        Kd = vrednost;
        Serial.print(">>> Glavni Kd (Balans) postavljen na: "); Serial.println(Kd, 5);
      }
      else if (komanda == 'c' || komanda == 'C') { // FIKS: Nova komanda za ciljani ugao (Centar mase)
        base_angle = vrednost;
        Serial.print(">>> Bazni ciljani ugao postavljen na: "); Serial.println(base_angle, 2);
      }
      else if (komanda == 'v' || komanda == 'V') { // OPCOJA: Podešavanje prigušenja brzine
        K_vel = vrednost;
        Serial.print(">>> K_vel postavljen na: "); Serial.println(K_vel, 5);
      }
      else if (komanda == 's' || komanda == 'S') { // OPCIJA: Podešavanje vraćanja na poziciju
        K_pos = vrednost;
        Serial.print(">>> K_pos postavljen na: "); Serial.println(K_pos, 5);
      }
    }
  }
}

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

  float raw_left_velocity = delta_left * Meters_Per_Step * F;
  float raw_right_velocity = delta_right * Meters_Per_Step * F; 

  left_velocity = (raw_left_velocity * mAlpha) + (left_velocity * (1 - mAlpha));
  right_velocity = (raw_right_velocity * mAlpha) + (right_velocity * (1 - mAlpha));
} 

void Wheel_Velocity(){
  left_desired_velocity  = PID_desired_velocity + rotational_speed ;
  right_desired_velocity = PID_desired_velocity - rotational_speed;
}

void Compute_mPID(float desired_velocity, float velocity, float last_mI, float last_velocity_error, float ffK, int ffN){
  
  float velocity_error = desired_velocity - velocity; 
  float feed_forward; 

  if(desired_velocity > 0.001){
    feed_forward = desired_velocity * ffK + ffN;
  }
  else if(desired_velocity < -0.001){
    feed_forward = desired_velocity * ffK - ffN;
  }

  float mP = velocity_error * mKp; 
  float mI = last_mI; 
  if(fabs(velocity_error) < mI_Treshold){
    mI += velocity_error * T; 
  } 
  mI = constrain(mI, -mI_Limit, mI_Limit); 
  float mD = (velocity_error - last_velocity_error) * F; 

  mPID_return[0] = feed_forward + mP + mI * mKi + mD * mKd; 
  mPID_return[1] = mI; 
  mPID_return[2] = velocity_error; 
} 

void Calculate_mPID(){
  Measure_Wheel_Velocity();
  Wheel_Velocity();

  Compute_mPID(left_desired_velocity, left_velocity, left_last_mI, left_last_velocity_error, Left_FFK, Left_FFN);
  left_motor_power = (int)mPID_return[0]; 
  left_last_mI = mPID_return[1]; 
  left_last_velocity_error = mPID_return[2];

  Compute_mPID(right_desired_velocity, right_velocity, right_last_mI, right_last_velocity_error, Right_FFK, Right_FFN); 
  right_motor_power = (int)mPID_return[0]; 
  right_last_mI = mPID_return[1]; 
  right_last_velocity_error = mPID_return[2]; 
} 

void Drive_Motors(){
  if(left_motor_power < 0){ 
    left_motor_direction = HIGH; 
    left_motor_power = abs(left_motor_power); 
  } 
  else{ 
    left_motor_direction = LOW; 
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
  delay(500);

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set Low-Pass Filter (10Hz)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1A); 
  Wire.write(0x05); 
  Wire.endTransmission();

  // Set Gyro Sensitivity (+-500 Deg/s)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B); 
  Wire.write(0x08); 
  Wire.endTransmission();

  // Set Accelerometer Range (+-8G)
  Wire.beginTransmission(0x68); 
  Wire.write(0x1C); 
  Wire.write(0x10); 
  Wire.endTransmission(); 
} 

void IMU(){
  // Čitanje žiroskopa
  Wire.beginTransmission(0x68); 
  Wire.write(0x43); 
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); 

  int16_t gyro_roll = Wire.read() << 8 | Wire.read(); 
  int16_t gyro_pitch = Wire.read() << 8 | Wire.read(); 
  int16_t gyro_yaw = Wire.read() << 8 | Wire.read(); 

  pitch_rate = (float)gyro_yaw / 65.5; 

  // Čitanje akcelerometra
  Wire.beginTransmission(0x68); 
  Wire.write(0x3B); 
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  int16_t x_acceleration_lsb = Wire.read() << 8 | Wire.read();
  int16_t y_acceleration_lsb = Wire.read() << 8 | Wire.read();
  int16_t z_acceleration_lsb = Wire.read() << 8 | Wire.read();

  x_acceleration = (float)x_acceleration_lsb / 4096 - 0.035;
  y_acceleration = (float)y_acceleration_lsb / 4096 + 0.000;
  z_acceleration = (float)z_acceleration_lsb / 4096 + 0.035;

  pitch_angle = atan2(y_acceleration, -x_acceleration) * 180.0 / PI;
} 

void Kalman(float kalman_state, float kalman_uncertainty, float kalman_input, float kalman_measurement){ 
  kalman_state = kalman_state + T * kalman_input; 
  kalman_uncertainty = kalman_uncertainty + T * T * 1 * 1;
  
  float kalman_gain = kalman_uncertainty * 1 / (1 * kalman_uncertainty + 0.8 * 0.8);
  kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state);
  kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty;
  
  kalman_output[0] = kalman_state;
  kalman_output[1] = kalman_uncertainty;
} 

void Kalman_Calculate(){
  // Pozivamo Kalman sa čistim stanjima
  Kalman(pitch_kalman_angle, pitch_kalman_angle_uncertainty, pitch_rate, pitch_angle);
  
  // Čisto filtrirano stanje vraćamo nazad bez dodavanja ofseta
  pitch_kalman_angle = kalman_output[0]; 
  pitch_kalman_angle_uncertainty = kalman_output[1];
  
  // Ofset od +4 dodajemo tek OVDE, na finalni ugao koji ide u PID i Plotter
  angle = pitch_kalman_angle + 4.0;
}

void Compute_PID(float desired_angle, float angle, float last_I, float last_angle_error){
  
  float angle_error = desired_angle - angle;
  
  float P = angle_error * Kp;
  float I = last_I; 
  if(fabs(angle_error) > I_Treshold){
    I += angle_error * T; 
  } 
  I = constrain(I, -I_Limit, I_Limit); 
  float D = (angle_error - last_angle_error) * F; 

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
  if(fabs(angle) > Fall_Treshold){
    last_I = 0;
    last_angle_error = 0;
    left_last_mI = 0;
    left_last_velocity_error = 0;
    right_last_mI = 0;
    right_last_velocity_error = 0;

    left_motor_power = 0;
    right_motor_power = 0;
    Drive_Motors();
    return false;
  }
  else{
    return true;
  }
}

void Battery_Voltage(){
  uint16_t mv = analogReadMilliVolts(Battery_Pin);
  battery_voltage = (mv * 5.69) / 1000.0;
}

bool Battery_Protection(){
  battery_check_counter ++;
  if(battery_check_counter >= 600){
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
    return true;
  }
  else{
    left_motor_power = 0;
    right_motor_power = 0;
    Drive_Motors();
    return false;
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

  // FIKS: Kombinuje se baza iz terminala i otklon palice
  desired_angle = base_angle + throttle;
  rotational_speed = (steering * Wheel_Base) / 2;
}

void Controller_Recieve(){
  BP32.update();

  if(controller && controller -> isConnected()){
    Process_Gamepad(controller);
  }
  else{
    throttle = 0;
    rotational_speed = 0; 
  }
}

void Position_Control(){

  float avarage_velocity = (left_velocity + right_velocity) * 0.5;
  robot_position += avarage_velocity * T;

  if(abs(throttle) > 0){
    robot_position = 0;
    desired_angle = base_angle + throttle;
  }
  else{
    desired_angle = base_angle - (avarage_velocity * K_vel) - (robot_position * K_pos);
  }
}

void setup() { 
  // Brzina je podignuta na 921600 i postavljen timeout da se osigura stabilnih 200Hz
  Serial.begin(115200); 
  Serial.setTimeout(1); 
  
  last_time = micros();

  pinMode(Buzzer_Pin, OUTPUT);
  BP32.setup(&On_Connected_Controller, &On_Disconnected_Controller);
   
  IMU_Setup();
  Motor_Setup();
  Encoder_Setup();
} 

void loop() { 
  Proveri_Serial_Komande();

  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000; 

    Controller_Recieve();
    
    IMU();
    Kalman_Calculate();
       
    if(Fall_Detection() && Battery_Protection()){
      Position_Control();
      Calculate_PID();  
      Calculate_mPID();
      Drive_Motors();
    }

    // Čist ispis za Serial Plotter sa uključenim K_vel i K_pos vrednostima
    Serial.print("Ciljani_Ugao:");   Serial.print(desired_angle); Serial.print(",  ");
    Serial.print("Trenutni_Ugao:");  Serial.print(angle);         Serial.print(",  ");
    Serial.print("Kp:");              Serial.print(Kp, 5);         Serial.print(",  ");
    Serial.print("Ki:");              Serial.print(Ki, 5);         Serial.print(",  ");
    Serial.print("Kd:");              Serial.print(Kd, 5);         Serial.print(",  ");
    Serial.print("K_vel:");           Serial.print(K_vel, 5);      Serial.print(",  ");
    Serial.print("K_pos:");           Serial.print(K_pos, 5);      Serial.println(",  ");
  }
}
