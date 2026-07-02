#include <Wire.h> 
#include <ESP32Encoder.h> 

// ==========================================
//          PARAMETRI ZA TJUNING (DINAMIČKI)  p25.5 i20 d14
// ==========================================
float mKp = 70;   
float mKi = 10.0;   
float mKd = 10;   

float TEST_DESIRED_VELOCITY = 0.0; 
// ==========================================

// Parametri za automatski test kockastog signala
bool test_aktivan = false;
unsigned long test_start_vreme = 0;

#define Battery_Pin 36
#define Buzzer_Pin 12 

#define F 200       // OPTIMIZACIJA: Frekvencija sistema za fiksni period od 2ms
#define T 0.005     // Period sistema (2ms -> 500Hz)
unsigned long last_time = 0; 
float battery_voltage = 12.0; // Početna sigurna vrednost
int battery_check_counter = 0;

ESP32Encoder left_encoder; 
ESP32Encoder right_encoder; 

// NOVI PIN_OUTS ZA ENKODERE
#define LApin 32
#define LBpin 33
#define RApin 27
#define RBpin 26

// NOVI PIN_OUTS ZA MOTORE
#define Left_Dir_Pin 16
#define Left_PWM_Pin 17
#define Right_Dir_Pin 0
#define Right_PWM_Pin 4

#define Steps_Per_Revolution 8192
#define Wheel_Diameter 0.075 
#define mI_Treshold 0.75
#define mI_Limit 500
#define Left_FFK 212.31
#define Right_FFK 219.30


const float Meters_Per_Step = PI * Wheel_Diameter / Steps_Per_Revolution; 

int left_motor_power; 
int right_motor_power; 
int left_motor_direction; 
int right_motor_direction; 

long last_left_steps = 0; 
long last_right_steps = 0;

float left_desired_velocity, left_velocity, left_last_mI, left_last_velocity_error; 
float right_desired_velocity, right_velocity, right_last_mI, right_last_velocity_error; 

float mP;
float mD;
float mAlpha = 0.20;

void Proveri_Serial_Komande() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); 
    
    if (input.length() > 0) {
      char komanda = input.charAt(0);
      
      if (komanda == 't' || komanda == 'T') {
        test_aktivan = true;
        test_start_vreme = millis();
        left_last_mI = 0;
        right_last_mI = 0;
        Serial.println(">>> Pokrenut automatski test kockastog signala!");
        return;
      }
      
      if (input.length() > 1) {
        float vrednost = input.substring(1).toFloat();
        
        if (komanda == 'p' || komanda == 'P') {
          mKp = vrednost;
          Serial.print(">>> mKp promenjen na: ");
          Serial.println(mKp);
        } 
        else if (komanda == 'i' || komanda == 'I') {
          mKi = vrednost;
          Serial.print(">>> mKi promenjen na: ");
          Serial.println(mKi);
        }
        else if (komanda == 'd' || komanda == 'D') {
          mKd = vrednost;
          Serial.print(">>> mKd promenjen na: ");
          Serial.println(mKd);
        }
        else if (komanda == 's' || komanda == 'S') {
          TEST_DESIRED_VELOCITY = vrednost;
          test_aktivan = false;
          left_last_mI = 0;
          right_last_mI = 0;
          Serial.print(">>> Ciljana brzina promenjena na: ");
          Serial.println(TEST_DESIRED_VELOCITY);
        }
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

  left_velocity = (mAlpha * raw_left_velocity) + ((1.0 - mAlpha) * left_velocity);
  right_velocity = (mAlpha * raw_right_velocity) + ((1.0 - mAlpha) * right_velocity);
} 

float Compute_mPID(float desired_velocity, float velocity, float &last_mI, float &last_velocity_error, float ffK){
  float velocity_error = desired_velocity - velocity; 

  float feed_forward = ffK * desired_velocity;
  mP = velocity_error * mKp; 
  
  // OPTIMIZACIJA: fabs() umesto abs() za float vrednosti
  if(fabs(velocity_error) < mI_Treshold){
    last_mI += velocity_error * T; 
  } 
  last_mI = constrain(last_mI, -mI_Limit, mI_Limit); 
  
  // OPTIMIZACIJA: Deljenje zamenjeno množenjem sa konstantom F
  mD = (velocity_error - last_velocity_error) * F;
  last_velocity_error = velocity_error; 

  return (feed_forward + mP + (last_mI * mKi) + (mD * mKd)); 
} 

void Calculate_mPID(){
  Measure_Wheel_Velocity();
  
  left_desired_velocity  = TEST_DESIRED_VELOCITY;
  right_desired_velocity = TEST_DESIRED_VELOCITY;

  left_motor_power = (int)Compute_mPID(left_desired_velocity, left_velocity, left_last_mI, left_last_velocity_error, Left_FFK);
  right_motor_power = (int)Compute_mPID(right_desired_velocity, right_velocity, right_last_mI, right_last_velocity_error, Right_FFK); 
} 

void Drive_Motors(){
  if(left_motor_power < 0){ 
    left_motor_direction = HIGH; 
    left_motor_power = abs(left_motor_power); 
  } else { 
    left_motor_direction = LOW; 
  }

  if(right_motor_power < 0){ 
    right_motor_direction = LOW; 
    right_motor_power = abs(right_motor_power); 
  } else { 
    right_motor_direction = HIGH; 
  }
  
  left_motor_power = constrain(left_motor_power, 0, 255); 
  right_motor_power = constrain(right_motor_power, 0, 255);

  digitalWrite(Left_Dir_Pin, left_motor_direction); 
  digitalWrite(Right_Dir_Pin, right_motor_direction); 
  analogWrite(Left_PWM_Pin, left_motor_power); 
  analogWrite(Right_PWM_Pin, right_motor_power); 
} 

void Battery_Voltage(){
  uint16_t mv = analogReadMilliVolts(Battery_Pin);
  battery_voltage = (mv * 5.69) / 1000.0;
}

bool Battery_Protection(){
  battery_check_counter++;
  // OPTIMIZACIJA: Na 500Hz, 500 ciklusa iznosi tačno 1 sekundu
  if(battery_check_counter >= 1000){
    Battery_Voltage();
    battery_check_counter = 0;
  }
  
  if(battery_voltage > 8.75){
    if(battery_voltage < 9.0){
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

void Motor_Setup(){
  pinMode(Left_Dir_Pin, OUTPUT);
  pinMode(Left_PWM_Pin, OUTPUT);
  pinMode(Right_Dir_Pin, OUTPUT);
  pinMode(Right_PWM_Pin, OUTPUT);
}

void setup() { 
  Serial.begin(115200); 
  last_time = micros();
  pinMode(Buzzer_Pin, OUTPUT);
  Motor_Setup();
  Encoder_Setup();
  Battery_Voltage(); // Inicijalno očitavanje napona
} 

void loop() { 
  Proveri_Serial_Komande();

  if (test_aktivan) {
    unsigned long proteklo_vreme = millis() - test_start_vreme;
    
    if (proteklo_vreme < 3000) {
      TEST_DESIRED_VELOCITY = 0.8;   
    } 
    else if (proteklo_vreme < 6000) {
      TEST_DESIRED_VELOCITY = -0.8;  
    } 
    else if (proteklo_vreme < 9000) {
      TEST_DESIRED_VELOCITY = 1.0;   
    } 
    else if (proteklo_vreme < 12000) {
      TEST_DESIRED_VELOCITY = -1.0;  
    } 
    else {
      TEST_DESIRED_VELOCITY = 0.0;   
      test_aktivan = false;
      Serial.println(">>> Automatski test završen.");
    }
  }

  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000; 
    
    
    if(Battery_Protection()){
      Calculate_mPID();
      Drive_Motors();
    }
    
    // Brzi ispis podataka za Plotter
    Serial.print("Ciljana:");    Serial.print(TEST_DESIRED_VELOCITY); Serial.print(",");
    Serial.print("Levi_Tocak:"); Serial.print(left_velocity);          Serial.print(",");
    Serial.print("Desni_Tocak:");Serial.println(right_velocity);
//    Serial.print("  PWM: "); Serial.println(left_motor_power);
//    Serial.print(" mP: "); Serial.print(mP);
//    Serial.print(" mI: "); Serial.print(left_last_mI  * mKi);
//    Serial.print(" mD: "); Serial.println(mD * mKd);
  }
}
