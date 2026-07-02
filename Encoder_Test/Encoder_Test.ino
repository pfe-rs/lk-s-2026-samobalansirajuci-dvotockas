#include <ESP32Encoder.h>

ESP32Encoder encoder;

#define Apin 32
#define Bpin 33

#define Steps_Per_Revolution 8192
#define Wheel_Diameter 0.075
const float Meters_Per_Step = PI * Wheel_Diameter / Steps_Per_Revolution;

long counts;
float distance;

void setup() {

  Serial.begin(115200);
  encoder.attachFullQuad(Apin, Bpin);
  encoder.clearCount();
  Serial.println("Encoder ready");
}

void loop() {

  counts = encoder.getCount();
  distance = counts * Meters_Per_Step;

  Serial.print("Counts: ");
  Serial.print(counts);

  Serial.print("  Distance (m): ");
  Serial.println(distance, 4);

  delay(100);
}
