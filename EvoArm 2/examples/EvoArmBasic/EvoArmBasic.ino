#include "EvoArm.h"
EvoArm arm;

//ARM SIGNAL WIRE PIN NUMBERS FOR SERVOS A THRU F: 2,3,4,5,6,7

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
  arm.init( ninety_positions );
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

int loop_counter = 0;
bool just_started = true;
bool very_first_time = true; // to make default mode when plugged in to "Power: ON"

void loop()
{
  loop_counter++;
  if ( loop_counter > 10000 ) //no need to let it overflow
    loop_counter = 100;
  if ( just_started )
  {
    if ( loop_counter < 10 )
    {
      delay( 50 );
      return;
    }
    else
    {
      just_started = false;
    }
  }

  char cmd[64];
  cmd[0] = 0;
  int ret = -1;
  
  char car_info[32];
  car_info[0] = 0;

  if (very_first_time) {
    arm.processExternalCommand("on", car_info); // to run command "on" automatically without userinput
    arm.processExternalCommand("home", car_info);
    very_first_time = false;
  }

  while( ret = readLine(cmd) ) //if there is input from the Serial Monitor
  {
    if ( !strcmp(cmd,"1" ))
      arm.runSequence( sequence1, asize(sequence1) );
    else if ( !strcmp(cmd, "2" ))
      arm.runSequence( sequence2, asize(sequence2) );
    else if ( !strcmp(cmd, "3" ))
      arm.runSequence( sequence3, asize(sequence3) );
    //add new sequences here
    else if ( !strcmp(cmd,"0" ))
      arm.runSequence( NULL, 0 ); //press 0 to stop running sequence!

    arm.processExternalCommand( cmd, "" ); //arm
    if ( ret == 2 ) //if newline was received at end of this command
      break;
    
    car_info[0]=0;
  }

  arm.loop();
  delay( 1 );
}
