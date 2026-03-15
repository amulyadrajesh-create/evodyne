#include <Servo.h>
Servo servos[2]; //servos[0] and servos[1]

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 2; i++){
    servos[i].attach(i+2);
  }
}

int deg[2] = {90,100}; 
int incr[2] = {1,1};
int startpos[2] = {10,30};
int endpos[2] = {170,120};

void loop() {
  for (int i = 0; i < 2; i++){
    deg[i] += incr[i];
    if ( deg[i] > endpos[i] or deg[i] < startpos[i]){
      incr[i] = -1;
    }
    if ( deg[i] < startpos[i]){
      incr[i] = -incr[i];
    }
    servos[i].write(deg[i]);
  }
  delay(30);
}
