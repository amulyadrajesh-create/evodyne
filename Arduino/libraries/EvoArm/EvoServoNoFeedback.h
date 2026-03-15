/*******************************************************
* Copyright (C) 2020 onwards
* Evodyne Robotics Corporation / Evodyne Robotics Academy
* support@evodyneacademy.com
* Can not be copied and/or distributed under any
* circumstances, directly or
* indirectly, verbatim or modified or derived or inspired
*******************************************************/

#ifndef _EVOSERVO_H_
#define _EVOSERVO_H_


// #define USE_SERVODRIVER


#ifdef USE_SERVODRIVER
#include "ServoDriver.h"
ServoDriver servo_driver;
void setupDriver()
{
  servo_driver.setup();
}
#else
#include <Servo.h>
#endif

#include "Power.h"

#define default_move_increment 0.16 //increased by rg from 0.08 on July 5 2022
#define min_increment 0.05   //increased by rg from 0.01 on July 5 2022


#define INVALID_DEG -100

struct MotorInfo {
  int orange_pin;
  int init_deg;
  int min_deg;
  int max_deg;
  float actual_at_90;
};


float mapf( float x,
            float in_min,
            float in_max,
            float out_min,
            float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool isValid( float deg )
{
   return (deg > 0.0 && deg <= 180.0 ); //do not include zero
}

#ifdef USE_SERVODRIVER
void servoWriteDeg( int servo_id, float deg )
{
  servo_driver.gotoPos( servo_id, deg );
}
#else
void servoWriteDeg( Servo& servo, float deg )
{
  //because default servo.write(deg) takes only ints
  //int us = mapf( deg, 0, 180.0, 1000.0, 2000.0 );
  int us = mapf( deg, 0.0, 180.0, (float)MIN_PULSE_WIDTH, (float)MAX_PULSE_WIDTH );
  servo.writeMicroseconds( us );
}
#endif

/***** MOTOR CLASS *****/

class EvoServo
{
  public:

    MotorInfo* info;

    //putting ints together
    int id;
    #ifndef USE_SERVODRIVER
    Servo servo;
    #endif

    float current_target; //intermediate targets as we slowly move toward final_target
    float final_target; //where the user wants it to go,

    //putting bools together
    bool attached;
    bool user_detached;

    unsigned long last_target_received;
    unsigned long prev_target_received;
    float move_increment;
    float start_position; //when move started
    bool reversed;

    EvoServo()
    {
      info = NULL;
      id = -1; //unknown
      current_target = INVALID_DEG;
      final_target = INVALID_DEG;
      start_position = INVALID_DEG;
      reversed = false;
      attached = false;
      user_detached = true;
      last_target_received = 0;
      prev_target_received = 0;
      move_increment = default_move_increment;
    }

    float curTarget( bool remove_offset = false) const
    {
      if ( !remove_offset )
        return current_target;
      return current_target + (info->actual_at_90 - 90.0);
    }
    //int positionUs() { return servo.readMicroseconds(); }
    float finalTarget() const { return final_target; }
    float startPosition() const { return start_position; }
    unsigned long targetReceived() const { return last_target_received; }
    void setMoveIncrement( float incr, bool dbg )
    {
      move_increment = incr;
      if ( dbg )
      {
        Serial.print( id );
        Serial.print( ": move increment " );
        Serial.println( move_increment );
      }
    }
    //initialization
    void init(MotorInfo* info_, int index, bool dbg )
    {
      info = info_;
      id = index;

      final_target = info->init_deg;
      attached = false;
      user_detached = true; //so doesnt perk upon power

      current_target = info->init_deg;
      last_target_received = 0;
      prev_target_received = 0;
      move_increment = default_move_increment;
      //we will attach in the cycle function
    }

    void detach( bool dbg ) //user detach for manual motion
    {
      if ( user_detached )
        return;
      #ifndef USE_SERVODRIVER
      servo.detach();
      #endif
      //current_target = actual_deg;
      final_target = current_target; //cancel ongoing motion
      last_target_received = 0;
      prev_target_received = 0;
      user_detached = true;
      move_increment = default_move_increment;
      attached = false;
    }
    void reAttach( bool power_just_turned_on, bool dbg )
    {
      //if ( !user_detached )
      //    return;

      move_increment = default_move_increment;
      user_detached = false;
      //let the cycle function pick up attached on its own
    }

    void print( const char* l, float v, bool ln = false) //to save on memory a bit
    {
      Serial.print( l );
      Serial.print( ": " );
      if ( ln )
        Serial.println( v );
      else
        Serial.print( v );
    }
    void print( const char* l, int v, bool ln = false) //to save on memory a bit
    {
      Serial.print( l );
      Serial.print( ": ");
      if ( ln )
        Serial.println( v );
      else
        Serial.print( v );
    }
    /*
    void print( const char* l1, int v1, const char* l2, int v2, bool ln = false ) //to save on memory a bit
    {
      char buf[128];
      snprintf( buf, 128, "%s: %d, %s: %d", l1, v1, l2, v2 );
      buf[127] = 0;
      if ( ln )
        Serial.println( buf );
      else
        Serial.print( buf );
    }
    */

    //called in loop
    void cycle( bool dbg )
    {
      if ( !isValid( final_target) && isValid( current_target ) ) //never got initialized, maybe power was off
        moveTo( info->init_deg );

      if ( !isValid( final_target) )
        return;

      //we don't want to go too far off actual position
      if ( dbg )
        print( "Deg", current_target, true );

      if ( !isValid( current_target ) )
        return;

      //lets find the correct move increment for accel/decel
      float cur_move_increment = move_increment;
      #ifdef GRADUAL
      if ( fabs(current_target - final_target) > 0.00001 ) //not at target
      {
          float moved = current_target - start_position;
          float total = final_target - start_position;
          if ( fabs(total) > 6.0 )
          {
            float percent_done = fabs( moved / total );
            if ( percent_done < 0.1 ) //dont start too slow
              percent_done = 0.1;
            if ( percent_done > 0.9 )
              percent_done = 0.9;
            if ( percent_done < 0.25 )
              cur_move_increment = percent_done * 6.0 * move_increment; //increase
            else if ( percent_done < 0.75 )
                cur_move_increment = 8.0/4.0 * move_increment; //fast
            else
              cur_move_increment = (1.0 - percent_done) * 6.0 * move_increment; //slow
            if ( cur_move_increment < min_increment )
              cur_move_increment = min_increment;
          }
          //cur_move_increment = move_increment;
      }
      #endif

      if ( final_target >= (current_target + cur_move_increment ) )
      { // if target value is greater than current position
        current_target += cur_move_increment; //increase servo by on degree
      }
      else if ( final_target <= (current_target - cur_move_increment ) )
      { // if target value is less than current position
        current_target -= cur_move_increment; //decrease servo by one degree
      }
      else
      {
        current_target = final_target; //difference is less than move_increment
      }
      if ( fabs(current_target - final_target) < 0.00001 )
        move_increment = default_move_increment;

      int actual_target = current_target;
      if ( reversed )
        actual_target = 180.0 - actual_target;
      if ( !user_detached &&
           !attached )
      {
        attached = true;
        #ifdef USE_SERVODRIVER
        servoWriteDeg( id, actual_target + (info->actual_at_90 - 90.0) );
        #else
        servoWriteDeg( servo, actual_target + (info->actual_at_90 - 90.0) );
        servo.attach( info->orange_pin );
        #endif
        if ( dbg )
        {
          print( "Attaching", id, true);
        }
      }
      #ifdef USE_SERVODRIVER
      servoWriteDeg( id, actual_target + (info->actual_at_90 - 90.0) );
      #else
      servoWriteDeg( servo, actual_target + (info->actual_at_90 - 90.0) );
      #endif
    }

    //called once
    void moveTo( float target, bool dbg = false )
    {
      //check if out of bounds
      float ninety_offset = info->actual_at_90 - 90.0;

      if( target < (info->min_deg - ninety_offset) )
        target = info->min_deg - ninety_offset;

      if( target > (info->max_deg - ninety_offset) )
        target = info->max_deg - ninety_offset;

      unsigned long curtime = millis();
      if ( (prev_target_received + 100) < curtime )
        start_position = final_target; //current_target; //or should it be final_target?
      final_target = target;
      prev_target_received = last_target_received;
      last_target_received = curtime;

      if ( dbg )
        print( "Target", final_target, true );
    }

    void increment(float inc, bool dbg = false)
    {

      if ( !isValid( current_target ) || !isValid( final_target ) )
        return;
        moveTo( final_target + inc, dbg );
    }

    void goHome()
    {
      moveTo( info->init_deg );
    }


};

#endif //_EVOSERVO_H_
