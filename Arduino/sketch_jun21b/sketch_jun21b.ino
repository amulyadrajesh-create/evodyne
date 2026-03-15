int echo_pin = A0;
int trig_pin = A1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(10000);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT); 

  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
}

void drive(int speed, int seconds){
  int time = seconds * 1000;
  if(speed > 40 && speed <= 255){
    digitalWrite(9,HIGH);
    digitalWrite(10,LOW);
    analogWrite(11,speed);

    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    analogWrite(3 ,speed);

    delay(time);

  } else if (speed <= -40 && speed >= -255) {
    digitalWrite(9,LOW);
    digitalWrite(10,HIGH);
    analogWrite(11, -speed);

    digitalWrite(6,LOW);
    digitalWrite(5,HIGH);
    analogWrite(3,--speed);

    delay(time);
  } else {
    analogWrite(11,0);
  }
}

float checkDist(){
  float distance;
  digitalWrite(trig_pin,LOW); //makes sure trig is turned off
  delayMicroseconds(2);
  digitalWrite(trig_pin,HIGH); //trig pin emits sound
  delayMicroseconds(10); 
  digitalWrite(trig_pin, LOW); //stop trig pin from emitting sound
  int duration = pulseIn(echo_pin, HIGH); //measures duration for sound to come back
  if (duration >= 0){
    float speed_of_sound = 0.343; //speed in mm/us
    distance = duration * speed_of_sound / 2.0;
  }
  return distance;
}


void loop() {
  // put your main code here, to run repeatedly:
  float distance = checkDist();
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(1000);

  //drive(200,3);
  //delay(1000);
  //drive(0,3);
  //delay(1000);
  //drive(50,3);
  //delay(1000);
}
