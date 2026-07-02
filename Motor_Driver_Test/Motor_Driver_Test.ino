#define Dir_pin 26
#define PWM_pin 27

int PWM_value = 0;

void setup() {

  pinMode(Dir_pin, OUTPUT);
  pinMode(PWM_pin, OUTPUT);
  digitalWrite(PWM_pin, HIGH);
}

void loop() {

  while(PWM_value < 254){
    PWM_value++;
    analogWrite(Dir_pin, PWM_value);
    delay(10);
  }
  
  while(PWM_value > 0){
   PWM_value--;
   analogWrite(Dir_pin, PWM_value);
    delay(10);
  } 
}
