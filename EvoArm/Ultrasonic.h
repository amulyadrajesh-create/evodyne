#include <stdio.h>
#include <stdlib.h>

#define echoPin A0
#define trigPin A1

class Ultrasonic
{
  public:
    int duration;
    int avg = 10000;
    int numReadings = 20;
    int readings[20];
    int readings_copy[20];
    int index = -1;
    bool hasdata = false;
    static int cmpfunc (const void * a, const void * b)
    {
      return ( *(int*)a - * (int*)b );
    }

    void init()
    {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      avg = 10000; //far
      index = -1;
      for (int i = 0; i < numReadings; i++ )
      {
        readings[i] = 0;
        readings_copy[i] = 0;
      }
    }

    void update()
    {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Read the signal from the sensor: a HIGH pulse whose
      // duration is the time (in microseconds) from the sending
      // of the ping to the reception of its echo off of an object.

      duration = pulseIn(echoPin, HIGH, 5000);

      // Convert the time into a distance
      index++;
      if ( index >= numReadings )
      {
        index = 0;
        hasdata = true;
      }
      if ( duration < 0 )
        readings[index] = 30000;
      else
        readings[index] = ((float)duration / 2.0) * 0.343;   // Divide by 29.1 or multiply by 0.0343

      if ( hasdata )
      {
        for (int i = 0; i < numReadings; i++ )
        {
          readings_copy[i] = readings[i];
        }
        qsort(readings_copy, numReadings, sizeof(int), cmpfunc);

        //avg
        long sum = 0; //was overflowing with int
        for (int i = 5; i < (numReadings - 5); i++)
        {
          sum += readings_copy [i];
        }
        avg = sum / (numReadings - 10);
      }
    }

    bool hasData()
    {
      return hasdata;
    }

    int getDistance()
    {
      //return avg;
      return readings[index];
    }

};
