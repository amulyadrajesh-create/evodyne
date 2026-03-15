/**
*                                                                       
* Author    :  Raghav Gupta                                             
* Copyright :  Evodyne Robotics, 2020 Onwards
*
* License:
* This code is the property of Evodyne Robotics and distribution is
* expressly prohibited
*
*/

#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

float getUltrasonicDistance( int echopin, int trigpin )
{
  digitalWrite( trigpin, LOW );
  delayMicroseconds( 2 );

  digitalWrite( trigpin, HIGH );
  delayMicroseconds( 10 );

  digitalWrite( trigpin, LOW );

  int duration = pulseIn( echopin, HIGH, 5000 );  //waits for a pulse to come in, 5 milliseconds
  //speed of sound = 343 meters per second
  //or 343*1000 milli-meters per second
  //or 343 milli-meters per milli-second
  //or 343/1000 milli-meters per micro-second
  float speed_of_sound = 343.0 / 1000.0; //mm per micro-second
  if ( duration > 0 )
  {
     float distance = (speed_of_sound * (float)duration / 2.0);
     //Serial.print( "distance: " );
     //Serial.println( distance );
     return distance;
  }
  return -1;
}

#endif // _ULTRASONIC_H_
