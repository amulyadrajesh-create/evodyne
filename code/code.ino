#include <Servo.h>
Servo servo1;
Servo servo2;

void setup() {
  Serial.begin( 115200 );
  servo1.attach( 2 );
  servo2.attach( 3 );
  Serial.setTimeout( 10000 );
}

void loop() {
  Serial.println("Enter which servo: 1 or 2?");
  String s = Serial.readStringUntil('/n');
  if (s == ""){
    return;
  }
int which = s.toInt();
if (which < 1 or which > 2)
  return;
Serial.print("Enter degrees for servo");
Serial.print( which );
s = Serial.readStringUntil( '/n' );
if (s != "")
{
  int deg = s.toInt();
  if ( deg >= 10 && deg <= 170 )
  {
    if ( which == 1 )
      servo1.write( deg );
    else 
      servo2.write( deg );
  }
}
}
