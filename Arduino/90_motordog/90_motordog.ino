#include <Servo.h>
Servo servo[6];

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 6; i++) {
    servo[i].attach(i + 2);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i < 6; i++) 
    servo[i].write(120);
  delay(300);
  for (int i = 0; i < 6; i++)
    servo[i].write(90);
  delay(3000);
}
