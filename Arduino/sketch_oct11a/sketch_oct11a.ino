#include "ServoDriver.h"
ServoDriver servo_driver;
/*void servogoto (Servo& servo, float ang)
{
  servo.write(ang + 90);
}*/


double getangleabc(double ab, double bc, double ca) {
  double v = (ab * ab + bc * bc - ca * ca) / (2.0 * ab * bc);
  if (v > 1.0) {
    v = 1.0;
  }
  if (v < -1.0) {
    v = -1.0;
  }
  return acos(v) * RAD_TO_DEG;
}

void servogoto (int servonum, float ang)
{
  servo_driver.gotoPos(servonum, ang + 90);
}

#define ROTRADIUS 31.0
#define LTHIGH 100.0
#define LCALF 125.327

#define ROT 0 
#define HIP 1
#define KNEE 2

#define FL 0
#define BL 1
#define FR 2
#define BR 3

struct LegInfo{
  int servo_num[3]; // servo_num[0]...[1]...[2], RHK pin numbers
  int ninety_eror[3]; //RHK errors
  bool reversed; //true or false
};

LegInfo leg_info[4] =
{
  { {0,1,2} , {0,0,45}, false}, //FL leg_info[0]
  { {3,4,5} , {0,0,45}, false}, //BL leg_info[1]
  { {6,7,8} , {0,0,45}, true }, //FR leg_info[2]
  { {9,10,11} , {0,0,45}, true}, //BR leg_info[3]
};


class Leg 
{
  public:
  int leg_id; //0,1,2,3
  void gotoAng(float rot, float hip, float knee)
  {

    LegInfo& li = leg_info[leg_id];
    servogoto( li.servo_num[0], rot);
    servogoto( li.servo_num[1], hip);
    servogoto( li.servo_num[2], knee);
    //leg_info[leg_id]
  }
  void calcAngles(float x, float y, float& h, float& k)
  {
    double max_length = LTHIGH + LCALF;
    double l = sqrt(x * x + y * y);
    if (l > max_length)
      l = max_length;
    double l_hip = getangleabc(LTHIGH, l, LCALF);
    double l_angle = getangleabc(l, y, x);
    if (x < 0)
      l_angle = -l_angle;
    h = l_hip - l_angle;;
    k = 180.0 - getangleabc(LTHIGH, LCALF, l);
  }
void gotoXY(float x, float y)
{
  float h = 0;
  float k = 0; 
  calcAngles(x, y , h, k);
  gotoAng(0, h, k);
}     

}; //end of class leg

Leg legs[4]; //create one leg

void setup() {
  // put your setup code here, to run once:
  servo_driver.setup();
}

void loop() {
  for (int i = 0; i < 12; i++)
  {
    servogoto(i,0);
  }
  return;

  legs[0].gotoAng(0,0,0);
  delay(3000);
  legs[0].gotoAng(30,30,30);
  delay( 300);
  return;

}