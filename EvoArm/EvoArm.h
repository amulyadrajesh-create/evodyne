/*******************************************************
* Copyright (C) 2020 onwards
* Evodyne Robotics Corporation / Evodyne Robotics Academy
* support@evodyneacademy.com
* Can not be copied and/or distributed under any
* circumstances, directly or
* indirectly, verbatim or modified or derived or inspired
*******************************************************/

#ifndef _EVOARM_H_
#define _EVOARM_H_


#define EVO_VERSION "2.1"

#define asize(A) (sizeof(A) / sizeof(A[0]))

struct Sequence
{
  const char* cmd;
  int delay_ms;
};

int floatprintf( char* buf, int sz, float val, float dec_places );
int readLine( char* cmd ); //0 means nothing, 1 means comma, 2 means newline
void haveEvocar();
void reverseMotor(int i);

class EvoArm
{
public:
  void syncSpeeds();

  void turnOffPower();
  void turnOnPower();
  void getInfo( bool remove_offset, bool include_power, const char* car_info ); // if the first char is 'i'
  void getPosUs(); // if the first char is 'u', microseconds
  void init( const char* ninety_positions );
  void loop();
  void processExternalCommand( char* cmd, const char* car_info );
  void sendCommand( const char* commandstr ); // "a:5,b:20" etc
  void runSequence( const Sequence* sequence, int len );
  void sequenceCycle();
  bool processCommand( char* command, const char* car_info );
  void goHome();
  bool setNinetyFromEEProm();
  void setCurrentAsNinety();
  void setNinetyPositions( const char* ninety_positions );
};

#endif //_EVOARM_H_
