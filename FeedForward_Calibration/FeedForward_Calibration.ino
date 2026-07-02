#include <Wire.h> 
#include <ESP32Encoder.h> 

#define T 0.01 // Period sistema (100Hz = 10ms)
unsigned long last_time = 0; 

ESP32Encoder left_encoder; 
ESP32Encoder right_encoder; 

// PIN_OUTS ZA ENKODERE (Tvoji originalni pinovi)
#define LApin 32
#define LBpin 33
#define RApin 27
#define RBpin 26

// PIN_OUTS ZA MOTORE (Tvoji originalni pinovi)
#define Left_Dir_Pin 16
#define Left_PWM_Pin 17
#define Right_Dir_Pin 0
#define Right_PWM_Pin 4

#define Steps_Per_Revolution 8110
#define Wheel_Diameter 0.075 
const float Meters_Per_Step = PI * Wheel_Diameter / Steps_Per_Revolution; 

long last_left_steps = 0; 
long last_right_steps = 0;
float left_velocity = 0;
float right_velocity = 0;

// Varijable za kalibraciju
bool kalibracija_aktivna = false;
int trenutni_pwm = 0;
int brojac_ciklusa = 0;
const int CIKLUSA_ZA_STABILIZACIJU = 50; // 50 * 10ms = 500ms čekanja na svakom koraku PWM-a

void Encoder_Setup(){ 
  left_encoder.attachFullQuad(LApin, LBpin); 
  left_encoder.clearCount(); 
  right_encoder.attachFullQuad(RApin, RBpin); 
  right_encoder.clearCount(); 
} 

void Motor_Setup(){
  pinMode(Left_Dir_Pin, OUTPUT);
  pinMode(Left_PWM_Pin, OUTPUT);
  pinMode(Right_Dir_Pin, OUTPUT);
  pinMode(Right_PWM_Pin, OUTPUT);
}

void Measure_Wheel_Velocity(){
  long left_steps = left_encoder.getCount(); 
  long right_steps = right_encoder.getCount();

  long delta_left = left_steps - last_left_steps; 
  long delta_right = right_steps - last_right_steps;

  last_left_steps = left_steps; 
  last_right_steps = right_steps; 

  left_velocity = delta_left * Meters_Per_Step / T;
  right_velocity = delta_right * Meters_Per_Step / T;
} 

void setup() { 
  Serial.begin(115200); 
  Motor_Setup();
  Encoder_Setup();
  last_time = micros();
  
  Serial.println("=================================================");
  Serial.println("Pošalji slovo 'g' u Serial Monitor za START testa");
  Serial.println("=================================================");
} 

void loop() { 
  // Provera komande za start
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') {
      kalibracija_aktivna = true;
      trenutni_pwm = 0;
      brojac_ciklusa = 0;
      Serial.println("PWM,Levi_M_S,Desni_M_S"); // CSV zaglavlje za Excel
    }
  }

  // Glavna vremenska petlja na 100Hz (10ms)
  if (micros() - last_time >= T * 1000000) {
    last_time += T * 1000000; 
    
    Measure_Wheel_Velocity();
    
    if (kalibracija_aktivna) {
      // Pusti napon na motore (pravac NAPRED)
      digitalWrite(Left_Dir_Pin, HIGH);
      digitalWrite(Right_Dir_Pin, LOW);
      analogWrite(Left_PWM_Pin, trenutni_pwm);
      analogWrite(Right_PWM_Pin, trenutni_pwm);
      
      brojac_ciklusa++;
      
      // Kada prođe 500ms, motor je postigao maksimalnu brzinu za taj PWM
      if (brojac_ciklusa >= CIKLUSA_ZA_STABILIZACIJU) {
        // Ispiši podatke u CSV formatu
        Serial.print(trenutni_pwm); Serial.print(",");
        Serial.print(left_velocity); Serial.print(",");
        Serial.println(right_velocity);
        
        // Prelazak na sledeću vrednost PWM-a
        trenutni_pwm++;
        brojac_ciklusa = 0;
        
        // Ako smo stigli do kraja (255)
        if (trenutni_pwm > 255) {
          kalibracija_aktivna = false;
          analogWrite(Left_PWM_Pin, 0);
          analogWrite(Right_PWM_Pin, 0);
          Serial.println(">>> Kalibracija završena! Ugasi motore.");
        }
      }
    }
  }
}
