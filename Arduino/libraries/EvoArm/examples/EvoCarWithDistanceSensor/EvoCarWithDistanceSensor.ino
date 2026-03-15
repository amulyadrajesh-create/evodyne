#include "EvoCar.h"
#include "EvoArm.h"
#include "Ultrasonic.h"

Ultrasonic ultra;
EvoCar car;
EvoArm arm;

//uncomment when evocar present
#define EVOCAR



//ARM SIGNAL WIRE PIN NUMBERS FOR SERVOS A THRU F: 2,4,7,8,12,13
//CAR PINNUMBERS:
//RIGHT EN1, IN1, IN2 => 3,  5,  6
//LEFT  EN2, IN3, IN4 => 11, 10, 9

//ULTRASONIC DISRANCE SENSOR PIN CONNECTIONS:
//GND  => GND ON ARDUINO
//ECHO => A0  ON ARDUINO
//TRIG => A1  ON ARDUINO
//VCC  => 5V  ON ARDUINO


//Motors are labelled a thru f from base to gripper
//in serial monitor, you can send any motor to any position in degrees
//with 90 being its 'center' position
//using the syntax "a:70", then press enter (don't type the quotes)
//you can position multiple motors simultaneously using commas, without spaces
//like so
// a:70,b:80,c:120
//the range of most joints is between 30 to 150

//you can also move motors incrementally using a + or - prefix, like so:
// a:+3 <enter> to move the 'a' motor by 3 degrees

//use command 'home' to send the arm to its home position
//use 'i' to see current position of all motors

//Before using, we need to calibrate the 90 positions of every joint
//in serial monitor, move every motor incrementally
//to visually make the arm into a perfect inverted L shape,
//with shoulder perfectly vertical, and elbow perfectly horizontal
//do for all joints a thru f
//then press lowercase 'o' in serial monitor
//replace the string value below with the response from 'o'

const char* ninety_positions = "a:90,b:90,c:90,d:90,e:90,f:90";

//after calibrating and uploading, type
// '90' in serial monitor
// this should put the arm in a perfect inverted L shape
//if not, adjust until perfect and press 'o' to get the correct value for
//the ninety_positions variable above

void setup()
{
  Serial.begin(115200);

  #ifdef EVOCAR
  haveEvocar(); //let arm know, sinces pins will be different
  #endif

  arm.init( ninety_positions );
  #ifdef EVOCAR
  car.init();
  ultra.init(); //added for distance sensor
  #endif

  
}

//simple sequence
Sequence sequence1[] = {
  { "a:70",  1500 }, //command, wait delay
  { "a:110", 1500 },
  { "a:90",  1000 },
};

//pick and place
Sequence sequence2[] = {
  //change these lines to match something you want to try
  { "a:90,b:40,c:120,d:90,e:156,f:90", 1600 }, //command, wait delay of 2 seconds
  { "a:90,b:40,c:120,d:90,e:156,f:60", 1600 },
  { "a:90,b:90,c:90,d:90,e:100,f:60", 1600 },
  { "a:40,b:90,c:90,d:90,e:100,f:60", 1600 },
  { "a:40,b:40,c:120,d:90,e:156,f:60", 1600 },
  { "a:40,b:40,c:120,d:90,e:156,f:90", 1600 },

  { "home", 1000 }, //return to home at end of sequence
};

//try your own sequence here!
//move the arm via the serial monitor to desired positions
//at each postion type "i" in serial monitor to get its position info
//and paste it in the first entry for each line below
Sequence sequence3[] = {
  //change these lines to match something you want to try
  { "a:90,b:40,c:120,d:90,e:156,f:90", 2000 }, //command, wait delay in milliseconds

  { "home", 1000 }, //return to home at end of sequence
};

//added for distance sensor
void stopIfObstacle()
{
  #ifdef EVOCAR  
  //aha: must stop only if moving forward into obstacle
  //we don't want to prevent backing out away from the obstacle!
  int max_curspeed = 0;
  if (car.dc_motors[0].targetSpeed > max_curspeed )
	max_curspeed = car.dc_motors[0].targetSpeed;
  if (car.dc_motors[1].targetSpeed > max_curspeed )
	max_curspeed = car.dc_motors[1].targetSpeed;
  if ( max_curspeed < 0 )
    max_curspeed = 0;
  if ( ultra.hasData() and 
       ultra.getDistance() > 10 and 
	max_curspeed > 0 and
       ultra.getDistance() < max_curspeed ) //distance proportional to forward speed
  {
     //only stop if going forwards
      if ( car.dc_motors[0].targetSpeed > 0 )
        car.processCommand( "r:0" );
      if ( car.dc_motors[1].targetSpeed > 0 )
        car.processCommand( "l:0" );
   }
   #endif   
}


int loop_counter = 0;
bool just_started = true;
bool very_first_time = true;

void loop()
{


  loop_counter++;
  if ( loop_counter > 10000 ) //no need to let it overflow
    loop_counter = 100;
  if ( just_started )
  {
    if ( loop_counter < 10 )
    {
      delay( 100 );
      return;
    }
    else
    {
      just_started = false;
    }
  }

  //added for distance sensor
  #ifdef EVOCAR
  if ( loop_counter % 10 == 0 )
  {
    ultra.update();
  }
  #endif
  
  char car_info[32];
  car_info[0] = 0;

  while (very_first_time) {
    arm.processExternalCommand( "on", car_info);
    arm.processExternalCommand( "home", car_info);
    very_first_time = false;
  }


  char cmd[64];
  cmd[0] = 0;
  int ret = -1;
  while( ret = readLine(cmd) ) //if there is input from the Serial Monitor
  {
    if (  !strcmp(cmd,"1" ))
      arm.runSequence( sequence1, asize(sequence1) );
    else if ( !strcmp(cmd, "2" ))
      arm.runSequence( sequence2, asize(sequence2) );
    else if ( !strcmp(cmd, "3" ))
      arm.runSequence( sequence3, asize(sequence3) );
    //add new sequences here
    else if ( !strcmp(cmd, "0" ))
      arm.runSequence( NULL, 0 ); //press 0 to stop running sequence!

    
    car_info[0] = 0;
#ifdef EVOCAR
    car.getInfo(car_info);
    char buf[12];
    snprintf( buf, 12, ",u:%d", ultra.getDistance() );
    strcat( car_info, buf );
#endif

    arm.processExternalCommand( cmd, car_info ); //arm
    #ifdef EVOCAR
    stopIfObstacle(); //added for distance sensor
    if ( cmd != "" )
      car.processCommand( cmd );
    #endif
    if ( ret == 2 ) //if newline was received at end of this command
      break;
  }
  
  arm.loop();
  #ifdef EVOCAR
  stopIfObstacle(); //added for distance sensor
  car.loop(1000);
  #endif
  delay( 1 );
}
