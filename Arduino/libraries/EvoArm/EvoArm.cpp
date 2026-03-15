/*******************************************************
* Copyright (C) 2020 onwards
* Evodyne Robotics Corporation / Evodyne Robotics Academy
* support@evodyneacademy.com
* Can not be copied and/or distributed under any
* circumstances, directly or
* indirectly, verbatim or modified or derived or inspired
*******************************************************/

#include <Arduino.h>
#include <EEPROM.h>
#include "EvoServoNoFeedback.h"
#include "EvoArm.h"
#include <math.h>


char g_command_str[64] = {0}; //dont use String
int g_command_len = 0;

bool have_evocar = false;


//save 90-offset config to eeprom
//starting at offset 32
//at offset 30 we save two specific byte values
//to signify that values have been saved
//hoping that these values don't occur naturally
#define EEPROM_90_SAVED_OFFSET  30
#define EEPROM_90S_SAVED_MAGIC0 239
#define EEPROM_90S_SAVED_MAGIC1 74

#define EEPROM_90_OFFSET 32

void saveDegToEEProm( int addr, float val ) //must convert float to positive int
{
    int ival = round( val );
    ival += 127; //add to make positive range
    EEPROM.update( addr, ival );
}

float readDegFromEEProm( int addr ) //must byte to float
{
    int ival  = EEPROM.read( addr );
    float val = ival - 127; //undo applied offset while saving
    return val;
}

void haveEvocar()
{
  have_evocar = true;
}

MotorInfo motor_info[] = { //Create array of MotorInfo struct
//{orange_pin, init_deg, min_deg, max_deg, actual_at_90 },
{2, 90,  10, 170, 90 }, // a
{3, 100, 10, 170, 90 }, // b
{4, 120, 10, 148, 90 }, // c
{5, 90,  10, 170, 90 }, // d
{6, 90,  10, 170, 90 }, // e
{7, 90,  40, 120, 90 }  // f
};
int evocar_arm_pins[] = { 2, 4, 7, 8, 12, 13 };

bool debug = false;

int floatprintf( char* buf, int sz, float val, float dec_places )
{
  int l = (int)val;
  float r = val - (float)l;
  if ( r == 0 )
    return snprintf( buf, sz, "%d", l );
  else
    return snprintf( buf, sz, "%d.%d", l, round(r * pow(10,dec_places)) );
}


//Declare Motor Array
const int num_motors = 6;
EvoServo servos[num_motors];

float speed_factor = 1.0; //default
//unsigned long when_speed_factor_received = 0; //time in millis

bool joystick_mode = false;

void reverseMotor( int i )
{
  servos[i].reversed = true;
}

void EvoArm::syncSpeeds()
{
  //for all the motors, we need to find
  //which ones got the command at the same time
  //and if any two or more motors got the command at
  //the same time, we need to make their move_increments
  //proportional to the length of the motion, to
  //make them end at the same time
  //if any two motors got their commands within 2 milliseconds of the other
  //assume they were commanded together

  char num_groups = 0;
  char motor_group[num_motors]; //which group this motor belongs to
  unsigned long group_timestamp[num_motors]; //max
  float ranges[num_motors]; //longest range for each group

  unsigned long curtime = millis();

  for( int i = 0; i < num_motors; i++ )
  {
    motor_group[i] = -1; //init to invalid.. which group this motor belongs to
    if ( !isValid( servos[i].curTarget() ) ||
         !isValid( servos[i].finalTarget() ) )
    {
       continue;
    }
    float range = fabs( servos[i].curTarget() - servos[i].finalTarget() );
    if ( range < 0.0001 )
        continue; //already at target
    motor_group[i] = num_groups; //new group
    if ( debug )
    {
      Serial.print( i );
      Serial.print( ": timestamp: " );
      Serial.println( servos[i].targetReceived() );
    }
    for( int g = 0; g < num_groups; g++ )
    {
      unsigned long tr = servos[i].targetReceived();
      if ( tr > 0 &&
          ( group_timestamp[g] == tr ||
            (group_timestamp[g] < tr && (group_timestamp[g]+5) > tr ) ||
            (group_timestamp[g] > tr && (group_timestamp[g] < (tr+5) ) ) //all this because unsigned
          )
        )
      {
        motor_group[i]= g;
        break;
      }
    }
    if ( motor_group[i] == num_groups ) //new group
    {
      group_timestamp[num_groups] = servos[i].targetReceived();
      ranges[ motor_group[i] ] = range;
      num_groups++;
    }
    else if ( range > ranges[ motor_group[i] ] )
    {
      ranges[ motor_group[i] ] = range;
    }
  }
  //now each motor has a group, if valid
  for( int i = 0; i < num_motors; i++ )
  {
    if ( motor_group[i] < 0 ||
      (group_timestamp[ motor_group[i] ] + 6) < curtime ) //no need to revisit previously set motors if unchanged
      continue;
    //we say that the longest range should have the default move increment
    //the shortest one should move slower to compensate, to reach its target
    //at the same time
    float range = fabs( servos[i].curTarget() - servos[i].finalTarget() );
    if ( range < 0.0001 )
      continue; //no need to move
    float max_range = ranges[ motor_group[i] ];
    float incr = default_move_increment * range / max_range; ///?? RG, check, wt if max_range is zero?
    if ( range < 2 )
        incr = default_move_increment;
    else if ( incr < min_increment )
        incr = min_increment;
    if ( debug )
    {
      Serial.print( i );
      Serial.print( ": range: " );
      Serial.print( range );
      Serial.print( ", group: " );
      Serial.println( (int)motor_group[i] );
    }
    servos[i].setMoveIncrement( incr * speed_factor, debug ); //scale everything by given factor
  }
}

void EvoArm::turnOnPower()
{
  if  ( isPowerOn() )
    return;
    /*
  for( int i = 0; i < num_motors; i++ )
  {
     servos[i].detach( debug );
  }
  power( false ); //only mosfet
  for( int i = 0; i < num_motors; i++ )
  {
     servos[i].reAttach( true, debug ); //true for power just turned on
  }
  */
  #ifdef USE_SERVODRIVER
  digitalWrite( A3, LOW ); //low on means on
  #endif
  for( int i = 0; i < num_motors; i++ )
  {
     servos[i].reAttach( true, debug ); //true for power just turned on
  }
  delay( 50 );
  power( true ); //only mosfet
}

void EvoArm::turnOffPower()
{
  if  ( !isPowerOn() )
    return;
  #ifdef USE_SERVODRIVER
  digitalWrite( A3, HIGH ); //high means off
  #endif
  for( int i = 0; i < num_motors; i++ )
  {
     servos[i].detach( debug );
  }
  power( false );
}


//read Input From Serial Monitor
int readLine( char* cmd ) //0 means nothing, 1 means comma, 2 means newline
{
  while(Serial.available() > 0)
  {
    char c = Serial.read();

    if ( !joystick_mode )
    {
        if ( c == ',' || c == '\n' || c == '\r' )
        {
            strcpy( cmd, g_command_str );
            g_command_str[0] = 0;
            g_command_len = 0;
            if ( cmd[0] != 0 )
            {
                return ( c == ',' ? 1 : 2 );
            }
        }
        else
        {
          g_command_str[g_command_len++] = c; // if c is not ',' or '\n' then add the char to a string
          g_command_str[g_command_len] = 0;
        }
    }
    else
    {
        if ( c != ',' && c != '\n' && c != '\r' )
        {
            cmd[0] = c;
            cmd[1] = 0;
            g_command_str[0] = 0;
            g_command_len = 0;
            return 2; //understand as full line
        }
    }
  }
  return 0;
}

void EvoArm::getInfo( bool remove_offset, bool include_power, const char* car_info ) // if the first char is 'i'
{
    //current target value for each servo
    char buf[128];
    int n = snprintf( buf, 128, "a:" );
    n += floatprintf( buf + n, 128 - n, servos[0].curTarget( remove_offset ), 4 ); //1 decimal place
    for( int i = 1; i < num_motors; i++ )
    {
      n += snprintf( buf + n, 128 - n, ",%c:", 'a' + i );
      n += floatprintf( buf + n, 128 - n, servos[i].curTarget( remove_offset ), 4 );
    }
    if ( include_power )
      n += snprintf( buf + n, 128 - n, ",pwr:1" ); //always on for basic arm
    if ( car_info[0] == 0 )
      Serial.println( buf );
    else
    {
      Serial.print( buf);
      Serial.print( "," );
      Serial.println( car_info );
    }
}

/*
void EvoArm::getPosUs() // if the first char is 'u', microseconds
{
    //current actual position for each servo
    char buf[ 128 ];
    int n = snprintf( buf, 128, "a:%d", servos[0].positionUs() );
    for( int i = 1; i < num_motors; i++ )
    {
      n += snprintf( buf + n, 128 - n, ",%c:%d", 'a' + i, servos[i].positionUs() );
    }
    Serial.println(buf);
}
*/
void EvoArm::goHome()
{
    for (int i = 0; i < num_motors; i++ )
    {
        servos[i].goHome(); //set every motor to its init_deg
        speed_factor = 1.0; //reset
    }
}

bool EvoArm::processCommand( char* cmd, const char* car_info )
{
  int len = strlen( cmd );
  int cIndex = -1; //index of the ':'
  float deg;
  if ( !strcmp( cmd, "home") )
  {
      goHome();
      return true;
  }
  else if (!strcmp( cmd,"ignore")) {
    return false;
  }
  else if ( !strcmp( cmd,"set90")) {
    setCurrentAsNinety();
    return false;
  }
  else if ( !strcmp( cmd, "i")) {
    getInfo( false, false, car_info );
    return false;
  }
  else if ( !strcmp( cmd, "k")) {
    getInfo( false, true, car_info );
    return false;
  }
  //else if(cmd == "u") {
  //  getPosUs();
  //  return false;
  //}
  else if ( !strcmp( cmd, "o")) {
    getInfo( true, false, car_info );
    return false;
  }
  else if ( !strcmp( cmd, "on") )
  {
    turnOnPower();
    //Serial.println("Power: On");
    return true;
  }
  else if ( !strcmp( cmd,"off"))
  {
    turnOffPower();
    //Serial.println("Power: Off");
    return false;
  }
  else if ( !strcmp( cmd,"debug on"))
  {
    debug = true;
    return false;
  }
  else if ( !strcmp( cmd,"debug off") )
  {
    debug = false;
    return false;
  }
  else if ( !strcmp( cmd,"version"))
  {
    Serial.println( EVO_VERSION );
    return false;
  }
  else if ( !strcmp( cmd, "90" ) )
  {
    for( int i = 0; i < num_motors; i++ )
      servos[i].moveTo(90);
    return true;
  }
  else if ( !strcmp( cmd,"j" ))
      joystick_mode = !joystick_mode;
  else if ( joystick_mode && len == 1 )
  {
      char s = cmd[0];
      if ( s >= 'a' && s <= 'f' )
          strcat( cmd, ":+3" );
      else if ( s >= 'A' && s <= 'F' )
      {
          s += 'a' - 'A';
          cmd[0] = s;
          strcat( cmd, ":-3" );
      }
      else if ( s == 'h' || s == 'H' )
      {
          cmd[0] = 0;
          len = 0;
          goHome();
          return true;
      }
  }


  for (int i = 0; i < len; i ++)
  {
     if ( cmd[i] == ':' ) //while iterating through the string get the index of the ':'
     {
       cIndex = i;
       break;
     }
  }

  if ( cIndex != -1 )
  { //if ':' has been found
    char c = cmd[0];
    char s = cmd[cIndex - 1]; //set s to the letter before the ':'
    int sidx = s - 'a'; //subtract 'a' from s to get an integer value.

    if ( cIndex == 1 && sidx >= 0 && sidx < num_motors ) //only one letter
    {
      if( cmd[cIndex + 1] == '+') // if the char after is a '+'
      {
        //float degInc = cmd.substring( cIndex + 2 ).toFloat(); // get the rest of the string after the '+'
        float degInc = atof(cmd + cIndex + 2);//substring( cIndex + 2 ).toFloat(); // get the rest of the string after the '+'
        if ( debug )
        {
            Serial.print( "incr: ");
            Serial.println( degInc );
        }

        servos[sidx].increment(degInc, debug ); //increment the respective servo byt degInc
        return true;
      }
      else if( cmd[cIndex + 1] == '-') // if the char after is a '-'
      {
        float degInc = atof( cmd + cIndex + 2); //cmd.substring( cIndex + 2 ).toFloat(); // get the rest of the string after the '-'
        servos[sidx].increment( degInc * -1.0, debug ); //increment the respective servo byt degInc * -1 because
        //the '-' does not get counted in the integer value.
        return true;
      }
    }

    //deg = cmd.substring( cIndex + 1 ).toFloat(); //if the char after ':' is not a '+' or a '-' then
    deg = atof(cmd + cIndex + 1);
    //set the target degree to the numbers after the ':'

    if ( cIndex == 1 && sidx >= 0 && sidx < num_motors ) //if servo exists
    {
      servos[sidx].moveTo(deg); //set respective servo's target value to deg.
    }
    else if ( cIndex == 1 && s == 's' ) //speed
    {
      speed_factor = deg;
    }
    else if ( cIndex == 2 && c == 'n' )
    {
        //na:80 => set a motor's ninety index to 80
        if ( sidx >= 0 && sidx < num_motors )
        {
          float prev = motor_info[sidx].actual_at_90;
          motor_info[sidx].actual_at_90 = deg;
        }
    }
  }
  return true;
}

const Sequence* sequence_active = NULL;
char sequence_pos = -1;
char sequence_len = 0;
long sequence_last_command_when = 0;

void EvoArm::runSequence( const Sequence* sequence, int len )
{
  //Serial.println( "running seq");
  sequence_active = sequence;
  sequence_len = len;
  sequence_pos = -1;
  sequence_last_command_when = 0;
}

void EvoArm::sequenceCycle()
{
  if ( sequence_active )
  {
    long curtime = millis();
    if ( sequence_pos < 0 ||  //execute first command immediately
        (sequence_last_command_when + sequence_active[sequence_pos].delay_ms ) < curtime )
    {
      sequence_pos++;
      if ( sequence_pos < sequence_len ) //check if done
      {
        sequence_last_command_when = curtime;
        //Serial.print( sequence_last_command_when );
        //Serial.print( " executing: " );
        //Serial.println( sequence_active[sequence_pos].cmd );
        sendCommand( sequence_active[sequence_pos].cmd );
      }
      else //done
      {
        //sequence is done
        sequence_active = NULL;
        sequence_len = 0;
        sequence_pos = -1;
        sequence_last_command_when = 0;
      }
    }
  }
}

bool something_happened = false;

void EvoArm::processExternalCommand( char* cmd, const char* car_info )
{
  if ( processCommand( cmd, car_info ) )
    something_happened = true;
}

void EvoArm::loop()
{
  sequenceCycle();

  if ( something_happened )
        syncSpeeds();

  for( int j = 0; j < num_motors; j++ )
  {
    servos[j].cycle( debug && something_happened ); // call cycle function for each servo.
  }

  if ( debug )
  {
    getInfo( false, false, "" );
  }

  something_happened = false; //reset
}


void EvoArm::sendCommand( const char* commandstr ) // "a:5,b:20" etc
{
  char buf[128];
  strlcpy( buf, commandstr, 127 );
  buf[127] = 0;
  // Read each command pair
  char* cmd = strtok( buf, ",");
  while (cmd!= 0)
  {
      if ( cmd[0] != 0 )
      {
        char scmd[32];
        strcpy( scmd, cmd );
        if ( processCommand( scmd, "" ) )
          something_happened = true;
      }
      cmd = strtok(0, ",");
  }
}

void EvoArm::setCurrentAsNinety()
{
    bool remove_offset = true; //must remove offset from current position
    for( int i = 0; i < num_motors; i++ )
    {
        float cur_position_without_offset = servos[i].curTarget( remove_offset );
        float cur_position_with_offset = servos[i].curTarget( false );
        float change = motor_info[i].actual_at_90 - cur_position_without_offset;
        motor_info[i].actual_at_90 = cur_position_without_offset;
        //also save to eeprom if different
        saveDegToEEProm( EEPROM_90_OFFSET + i, cur_position_without_offset ); //must convert float to positive int
        //now must also offset this motor' target according to change in remove_offset
        servos[i].moveTo( cur_position_with_offset + change );
        //to prevent it from thinking it is at some other place
        //and then coming back, tell it it is where it needs to be
        servos[i].current_target = servos[i].final_target;
    }
    //mark eeprom as having a saved values
    EEPROM.update( EEPROM_90_SAVED_OFFSET + 0, EEPROM_90S_SAVED_MAGIC0 );
    EEPROM.update( EEPROM_90_SAVED_OFFSET + 1, EEPROM_90S_SAVED_MAGIC1 );
}

bool EvoArm::setNinetyFromEEProm()
{
    //if available
    if ( EEPROM.read( EEPROM_90_SAVED_OFFSET+0 ) != EEPROM_90S_SAVED_MAGIC0 ||
         EEPROM.read( EEPROM_90_SAVED_OFFSET+1 ) != EEPROM_90S_SAVED_MAGIC1 )
      return false;

    //get each motors 90 offset from sequential bytes
    for( int i = 0; i < num_motors; i++ )
    {
      motor_info[i].actual_at_90 = readDegFromEEProm( EEPROM_90_OFFSET + i );
    }
    return true;
}

void EvoArm::setNinetyPositions( const char* ninety_positions )
{
  char buf[64];
  strncpy( buf, ninety_positions, 63 );
  buf[63] = 0;
  // Read each command pair
  char* cmd = strtok( buf, ",");
  while (cmd!= 0)
  {
      // Split the command in two values
      char* separator = strchr(cmd, ':');
      if (separator != 0)
      {
          // Actually split the string in 2: replace ':' with 0
          *separator = 0;
          int servoId = *cmd - 'a';
          ++separator;
          float position = atof(separator);
          motor_info[servoId].actual_at_90 = position;
      }
      // Find the next command in input string
      cmd = strtok(0, ",");
  }
}

void EvoArm::init(  const char* ninety_positions )
{
  #ifdef USE_SERVODRIVER
  setupDriver();
  pinMode( A3, OUTPUT ); //connected on OE/output-enable on servo driver
  digitalWrite( A3, HIGH ); //high means off
  #endif

  if ( have_evocar )
    for( int i = 0; i < num_motors; i++ )
      motor_info[i].orange_pin = evocar_arm_pins[i];
  //update ninety_positions, first check EEPROM
  //else with passed values
  if ( !setNinetyFromEEProm() )
  {
    //Serial.println( "90 info from user");
    setNinetyPositions( ninety_positions );
  }

  //pinMode( MOSFET_GATE_PIN, OUTPUT );
  power( false ); //keep servos off for the moment, mosfet only here
  for ( int i = 0; i < num_motors; i++ )
  {
    servos[i].init( &motor_info[i] , i, debug ); //init but detached
    pinMode( motor_info[i].orange_pin, OUTPUT );
  }
}
