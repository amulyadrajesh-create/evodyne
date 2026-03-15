#ifndef _SERVO_DRIVER_H_
#define _SERVO_DRIVER_H_

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm;

#define MIN_PULSE_WIDTH 544
#define MAX_PULSE_WIDTH 2400

#define SERVO_FREQ 50
#define NUM_INCREMENTS 4096
#define ONE_SEC_US 1000000
#define PWM_PERIOD ONE_SEC_US / SERVO_FREQ
#define US_PER_INCR ((float)PWM_PERIOD / (float)NUM_INCREMENTS)
#define SERVO_MIN (MIN_PULSE_WIDTH / US_PER_INCR)
#define SERVO_MAX (MAX_PULSE_WIDTH / US_PER_INCR)

float mapf(float x,
           float in_min,
           float in_max,
           float out_min,
           float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class ServoDriver
{
  public:
  void gotoPos(int servonum, float deg)
  {
    int pulselen = mapf(deg, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(servonum, 0, pulselen);
  }
  void stop(int servonum)
  {
    pwm.setPWM(servonum, 0, 0);
  }

  void setup()
  {
    pwm.begin();
    Wire.setClock(400000L);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
  }
};
#endif 