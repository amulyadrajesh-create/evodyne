/*
  Author    :  Raghav Gupta
  Copyright :  Evodyne Robotics, 2020 Onwards

  License:
  This code is the property of Evodyne Robotics and distribution is
  expressly prohibited

*/

#include "ServoDriver.h"
#include "Utils.h"
#include "Ultrasonic.h"
#include "Gait.h"
#include "Matrix.h"

//number to steps to take, saved multiplied by four, one for each leg
//therefore max steps is 32 (since *4 is 128, max for int8_t. or is it 127?
//in which case it would be 31 )
int8_t num_pending_steps = -1; //for walking limited steps in gait

#define FIRMWARE_VERSION 22

#define FLAG_DEBUG  1
#define FLAG_PAUSED 2
#define FLAG_POWER  4


struct Flags
{
  uint8_t flags;
  bool isset( uint8_t flag )
  {
    return (flags & flag);
  }
  void set( uint8_t flag )
  {
    flags |= flag;
  }
  void unset( uint8_t flag )
  {
    flags &= ~flag;
  }
};
Flags flags = { FLAG_PAUSED };


ServoDriver servo_driver;
//char (*__kaboom)[sizeof( ServoDriver )] = 1; //to see sizeof at compile time

void servogoto( int8_t servo_id, float pos )
{
  servo_driver.gotoPos( servo_id, ( pos + 90.0 ) );
}

void println( const char* name, float val )
{
  Serial.print( name );
  Serial.print( ':' );
  Serial.print( ' ' );
  Serial.println( val );
}

//void printpos( float* pos, const char* prefix = "" )
//{
  /*
    Serial.print( prefix );
    Serial.print( pos[0] );
    Serial.print( ',' );
    Serial.print( pos[1] );
    Serial.print( ',' );
    Serial.println( pos[2] );
  */
//}
void printpos2( int8_t* pos, const char* prefix = "" )
{
  Serial.print( prefix );
  Serial.print( (float)pos[0] );
  Serial.print( ',' );
  Serial.print( (float)pos[1] );
  Serial.print( ',' );
  Serial.print( (float)pos[2] );
}

//mm
#define ROTRADIUS 31
#define LTHIGH 100.0
#define LCALF  125.327

#define BODY_LENGTH 221.8
#define BODY_WIDTH  86
#define HALF_BODY_LENGTH 110.9
#define HALF_BODY_WIDTH 43.0


#define ROTATOR 0
#define HIP     1
#define KNEE    2


#define  FL    0
#define  BL    1
#define  FR    2
#define  BR    3
#define  FRONT 4
#define  BACK  5
#define  LEFT  6
#define  RIGHT 7
#define  ALL   8

#define REVERSED 1
#define IS_XYZ   2

struct LegInfo {
  int8_t ids[3]; //servo_ids of each. todo: cant we calculate these?

  //how much offset to apply to make centered at zero. this is for installation offset, not runtime
  int8_t ninety_offset_pos[3];
  Flags flags;
};

#define NUM_LEGS 4
//fl, bl, fr, br
LegInfo leg_info[] = {   //rot, hip, knee
  // { {0, 1, 2}, { 0, 0,-45}, 0        } },
  // { {3, 4, 5}, { 0, 0,-45}, 0        } },
  // { {6, 7, 8}, { 0, 0, 45}, REVERSED } },
  // { {9,10,11}, { 0,0,  45}, REVERSED } },

  //your values
  { {0,  1,  2}, {  0,  0, -45 }, 0        }, //change to 0,0,-45 for defaults
  { {3,  4,  5}, {  0,  0, -45 }, 0        }, //change to 0,0,-45 for defaults
  { {6,  7,  8}, {  0,  0,  45 }, REVERSED }, //change to 0,0,45 for defaults
  { {9, 10, 11}, {  0,  0,  45 }, REVERSED }  //change to 0,0,45 for defaults
};

int8_t offsets[4][3] = {0};
int8_t gait_offsets[4][3] = {0};

//for robot positioning, we need to save/maintain the robot top rotation and displacement from default center
//position also, and then calculate angles using that
int8_t robot_top_pos[3] = {0, 0, 0}; //pos,x,y,z displacement from default
int robot_top_rot[3] = {0}; //multiplied by 10 //angles, x,y,z rotation from straight ahead, roll, pitch, yaw
float body_rot[3][3] = {0}; //rotation matrix



void updateBodyRot()
{
  //update body rotation matrix given desired rotation angle of body top
  get3drot( (float)robot_top_rot[0] / 10.0, (float)robot_top_rot[1] / 10.0, (float)robot_top_rot[2] / 10.0, body_rot );
}

GaitLegParam gait_trot = { //original, reference trot gait values
  //amplitude, center, phase, clipmin%, clipmax%
  { {30,  0,  0, -100, 100 }, //x
    {30, 190, 90, -100,  20 }, //y, RG Jan 12 2024, changed amplitude from 20
    {0,   0,  0, -100, 100 }  //z
  },

  //if foot down and up phases were of equal length
  //0 deg is at bottom of middle of down phase
  //at x=0 and y=max. at y=max, foot is most below hip
  //so touchdown start happens at -90, or 270
  //and end happens at +90
  290, //touchdown_start_deg //270 + 20
  70, //touchdown_end_deg    //90 - 20

  //300 and 130 are known to be good
  300, //down_duration_ms
  120  //up_duration_ms

  //trying faster speed, RG Jan 12 2024
  //200, //down_duration_ms
  //90 //up_duration_ms
};


GaitRobot trot_gait[4] = {
  { gait_trot,   0, NULL, 0, {25, 0, 0} },
  { gait_trot, 180, NULL, 0, {45, 0, 0} },
  { gait_trot, 180, NULL, 0, {25, 0, 0} },
  { gait_trot,   0, NULL, 0, {45, 0, 0} },
};

GaitLegParam gait_walk = {
  //amplitude, center, phase offset, clipmin
  { {30,   0,  0,  -100, 100 }, //x
    {20, 190, 90,   -90,  20 }, //y
    {0,    0,  0,  -100, 100 }  //z
  },
  290, //touchdown_start_deg //270 + 20
  70, //touchdown_end_deg    //90 - 20
  600, //down_duration_ms
  300  //up_duration_ms
};

GaitLegParam def_gait_walk = gait_walk;

//legs raise in order 3,2,1,0
//FL, BR, FR, BL

//BR raises at degrees 12, need cg to be on left side z offset shud be +50
//FR raises at degrees 72 or 84, need cg on to stay/be left side
//BL raises at degrees 168, need cg to SWITCH to right side quickly, z offset shud become -50
//FL raises at degrees 252, need cg to stay/be on right side, -50
//BR raises at degrees 0, need cg to be on left side, switch z offset to +50

//evaluate making 3 sep arrays for x y and z, then need
//not define those that aren't needed
/*
  CGOffset walk_cg_offsets[30] = {
  { 5, 0, 20 }, //0 degrees, back right is rising
  { 5, 0, 30 }, //12
  { 0, 0, 40 }, //24
  { 0, 0, 40 }, //36
  { 0, 0, 40 }, //48
  { 0, 0, 40 }, //60
  { 0, 0, 40 }, //72
  {-5, 0, 40 }, //84 //FR front right rises
  {-5, 0, 40 }, //96
  {-5, 0, 40 }, //108
  { 0, 0, 40 }, //120
  { 0, 0, 30 }, //132
  { 0, 0, 20 }, //144
  { 0, 0,  0 }, //156
  { 5, 0,-20 }, //168 BL back left rises
  { 5, 0,-30 }, //180
  { 5, 0,-40 },  //192
  { 0, 0,-40 }, //204
  { 0, 0,-40 }, //216
  { 0, 0,-40 }, //228
  { 0, 0,-40 }, //240
  {-5, 0,-40 }, //252 FL front left rises
  {-5, 0,-40 }, //264
  {-5, 0,-40 }, //276
  { 0, 0,-40 }, //288
  { 0, 0,-40 }, //300
  { 0, 0,-40 }, //312
  { 0, 0,-30 }, //324
  { 0, 0,-20 }, //336
  { 0, 0,  0 }, //348
  };
*/
//CGOffset walk_cg_offsets[30] = {
int8_t walk_cg_offsets_x[30] = { 5, 5, 0, 0, 0, 0, 0, -5, -5, -5, 0, 0, 0, 0, 5, 5, 5, 0, 0, 0, 0, -5, -5, -5, 0, 0, 0, 0, 0, 0 };
//char walk_cg_offsets_y[30] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
int8_t walk_cg_offsets_z[30] = { 20,  30,  40,  40,  40,  40,  40,  40,  40,  40,  40,  30,  20,  0,
                                      -20, -30, -40, -40, -40, -40, -40, -40, -40, -40, -40, -40, -40, -30, -20, 0
                                    };
int8_t* walk_cg_offsets[3] = {walk_cg_offsets_x, NULL, walk_cg_offsets_z };

GaitRobot walk_gait[4] = {
  { gait_walk, 0, walk_cg_offsets, 30, {25, 0, 0} }, //FL
  { gait_walk,  90, NULL, 0, {45, 0, 0} }, //BL
  { gait_walk, 180, NULL, 0, {25, 0, 0}}, //FR
  { gait_walk, 270, NULL, 0, {45, 0, 0} }, //BR
};

GaitRobot* robot_gait = trot_gait;
#define gait_step 12 //12 degrees per cycle
int8_t turn_right = 0;//more positive for more right, negative for turning left
int8_t shuffle_right = 0;//more positive for more right, negative for turning left
bool gait_direction = true;


void resetGaits()
{
  for (int8_t i = 0; i < 4; i++ )
  {
    trot_gait[i].gait = gait_trot;
    walk_gait[i].gait = gait_walk;
    turn_right = 0;
    shuffle_right = 0;
    gait_direction = true;
  }
}
GaitLegParam* getRefGaitLegParam( GaitRobot* gait )
{
  if ( gait == trot_gait )
    return &gait_trot;
  return &gait_walk;
}

class Leg
{
  public:
    int8_t leg_id;
    float target_pos[3];
    float start_pos[3];
    float cur_pos[3];

    int index = 0;
    unsigned long start_time = 0;
    unsigned long end_time = 0;

    void setup( int8_t id )
    {
      pinMode( 2, OUTPUT ); //connected on OE/output-enable on servo driver
      digitalWrite( 2, HIGH ); //high means off
      //ultrasonic sensor
      pinMode( A0, INPUT ); //ECHO
      pinMode( A1, OUTPUT ); //TRIG

      leg_id = id;
      for ( int8_t i = 0; i < 3; i++ )
      {
        cur_pos[i] = 0; //leg_info[id].init_pos[i]; // + 10; //so incr isnt zero
        start_pos[i] = cur_pos[i];
        target_pos[i] = cur_pos[i];
      }
    }
    void print( bool include_offset )
    {
      Serial.print( (int)leg_id );
      for (int8_t i = 0; i < 3; i++ )
      {
        Serial.print( '\t' );
        Serial.print( cur_pos[i] + (include_offset ? offsets[leg_id][i] : 0));
      }
      Serial.println();
    }
    void goTo( float* pos, int duration, bool reversed, unsigned long curtime ) //in milliseconds
    {
      for ( int8_t i = 0; i < 3; i++ )
        if ( isnan( pos[i] ) || isinf(pos[i]) || !isValidAngle(pos[i] ) )
          return; //don't do it

      start_time = curtime;
      end_time = curtime + duration;
      for ( int8_t i = 0; i < 3; i++ )
      {
        start_pos[i] = cur_pos[i];
        if ( i == 0 )
          target_pos[i] = ( leg_id == 1 || leg_id == 3 ? -pos[i] : pos[i] );
        else
          target_pos[i] = ( reversed ? -pos[i] : pos[i] );

        //check out of bounds
        int tpos = target_pos[i] + leg_info[leg_id].ninety_offset_pos[i];
        if ( tpos < -85 )
          target_pos[i] = -85  - leg_info[leg_id].ninety_offset_pos[i];
        else if ( tpos > 85 )
          target_pos[i] = 85  - leg_info[leg_id].ninety_offset_pos[i];
      }
    }
    void goTo( int* pos, int duration, bool reversed, unsigned long curtime ) //in milliseconds
    {
      float fpos[3] = { (float)pos[0], (float)pos[1], (float)pos[2] };
      goTo( fpos, duration, reversed, curtime );
    }

    void goTo( float* pos, int duration, bool reversed, unsigned long curtime, bool is_xyz ) //in milliseconds
    {
      if ( is_xyz )
      {
        //adding feb 26 2023
        //body top may be rotated
        //here we are saying that pos input to goTo is actually with respect to the fixed default leg origin
        //without acknowledging that body could be rotated on top
        //we use the known rotation now to, to make sure the body stays rotated as desired

        //so if we want to keep things reversible, then when we do getPositionXYZ using current leg joint angles
        //we need to add back the effect of body rotation, to get back positions without acknowledging body rotation
        //so that if those same positions were sent back to this goTo function, there would be no movement

        //so the global offsets are applied to the communicated positions, which is good
        //but it means the offsets should be applied to the input positions *before* the calculations below

        float leg_origin[3] = { (leg_id == 0 || leg_id == 2 ? HALF_BODY_LENGTH : -HALF_BODY_LENGTH),
                                (leg_id < 2 ? HALF_BODY_WIDTH : -HALF_BODY_WIDTH), 0
                              }; //from robot_top, center of robot body top
        //get desired foot pos
        //and assume the body is rotated and leg oriented to reach desired foot pos
        //now if we rotated the body back to default
        //it would rotate back by opposite of how it was moved
        //and the foot pos would also move by opposite of how it moved
        //so if we just move desired foot pos by opposite rot, we could
        //use that to do joint calculations

        //first add offsets in input
        add( pos, offsets[leg_id] );

        //find desired foot pos
        //also convert from leg coordinate system to world, switch coordinates around, sign reverse etc
        //since for the FL leg, X increases backward, Y increases downward, Z increases inward(to the right)
        float foot_pos[3] = {
          -pos[0] + leg_origin[0] - robot_top_pos[0],
          -pos[2] + leg_origin[1] - robot_top_pos[1],
          -pos[1]                 - robot_top_pos[2]
        };

        //rotate this by the opposite of what we need the body to rotate
        //printpos( foot_pos, "4: " );
        float virtual_foot_pos[3];
        multtranspose( body_rot, foot_pos, virtual_foot_pos );
        //printpos( virtual_foot_pos, "5: " );

        //now subtract leg origin from it
        pos[0] = -(virtual_foot_pos[0] - leg_origin[0]);
        pos[1] = -virtual_foot_pos[2];
        pos[2] = -(virtual_foot_pos[1] - leg_origin[1]);


        //end approach 2


        float angpos[3];
        if ( calcAnglesXYZ( pos[0], pos[1], pos[2], angpos[0], angpos[1], angpos[2] ) )
          goTo( angpos, duration, reversed, curtime );
        else if ( flags.isset( FLAG_DEBUG) )
          println( "E", 0 );
      }
      else
      {
        goTo( pos, duration, reversed, curtime );
      }
    }
    void goTo( int* pos, int duration, bool reversed, unsigned long curtime, bool is_xyz ) //in milliseconds
    {
      float fpos[3] = { (float)pos[0], (float)pos[1], (float)pos[2] };
      goTo( fpos, duration, reversed, curtime, is_xyz );
    }
    void goTo( int8_t* pos, int duration, bool reversed, unsigned long curtime, bool is_xyz ) //in milliseconds
    {
      float fpos[3] = { (float)pos[0], (float)pos[1], (float)pos[2] };
      goTo( fpos, duration, reversed, curtime, is_xyz );
    }

    void updateTarget( float* new_target, unsigned long curtime, int duration = 500 )
    {
      //if doing a motion, only update target
      //else send to new target
      if ( end_time == 0 || curtime > end_time )
      {
        goTo( new_target, duration, leg_info[leg_id].flags.isset(REVERSED), curtime, true );
      }
      else
      {
        //currently moving, update target without starting new motion
        goTo( new_target, end_time - start_time, leg_info[leg_id].flags.isset(REVERSED), start_time, true );
      }
    }
    void directGoto( int j, float pos )
    {
      if ( pos < -500 || pos < -150 || pos > 150 )
        return; //ignore this motor
      //find leg id and motor id within it

      servogoto( leg_info[leg_id].ids[j], leg_info[leg_id].ninety_offset_pos[j] + pos );
      cur_pos[j] = pos;
      target_pos[j] = pos;
    }
    void directGoto( float* pos )
    {
      for ( int8_t j = 0; j < 3; j++ )
      {
        if ( pos[j] < -500 || pos[j] < -150 || pos[j] > 150 )
          return; //ignore this motor
        servogoto( leg_info[leg_id].ids[j], leg_info[leg_id].ninety_offset_pos[j] + pos[j] );
        cur_pos[j] = pos[j];
        target_pos[j] = pos[j];
      }
    }
    void directGoto( int* pos )
    {
      float fpos[3] = { (float)pos[0], (float)pos[1], (float)pos[2] };
      directGoto( fpos );
    }
    void loop( unsigned long curtime )
    {
      if (end_time == 0 )
        return;
      for ( int8_t i = 0; i < 3; i++ )
      {
        //desired position at this time in the cycle
        cur_pos[i] = ( float(curtime - start_time ) /
                       (float)(end_time - start_time) * (float)(target_pos[i] - start_pos[i]) ) +
                     (float)start_pos[i];
      }

      for ( int8_t i = 0; i < 3; i++ )
      {
        int pos = cur_pos[i] + leg_info[leg_id].ninety_offset_pos[i]; //add the installation error adjustment
        servogoto( leg_info[leg_id].ids[i], pos );
      }
      if ( curtime > end_time )
      {
        end_time = 0; //done, mark motion as complete
      }
    }
    bool calcAngles( float x, float y, float& h, float& k ) //from xy position
    {
      double max_length = LTHIGH + LCALF;

      double l = sqrt(x * x + y * y); //length of leg in desired  position
      if ( l > max_length )
        l = max_length;
      //printf( "length: %0.1lf\r\n", l );
      //now upper leg and lower leg and l form a triangle
      //this is angle from l, not vertical
      double l_hip = getangleabc( LTHIGH, l, LCALF);
      //printf( "l_hip: %0.1lf\r\n", l_hip );
      double l_angle = getangleabc( l, y, x );
      if ( x < 0 )
        l_angle = -l_angle;
      //printf( "l_angle: %0.1lf\r\n", l_angle );
      double hip_angle = l_hip + l_angle;

      //knee angle
      double knee_angle = 180.0 - getangleabc( LCALF, LTHIGH, l );

      h = hip_angle;
      k = knee_angle;
      float p = h + leg_info[leg_id].ninety_offset_pos[1];

      if ( p < -85 or p > 85 )
      {
        return false;
      }

      p = (leg_info[leg_id].flags.isset(REVERSED) ? -k : k) + leg_info[leg_id].ninety_offset_pos[2];
      return ( p >= -85 and p <= 85 );

      //clip( hip_angle, -60.0, 80.0 );
      //clip( knee_angle, -90.0, 135.0 );
    }
    bool calcAnglesXYZ( float x, float y, float z, float& r, float& h, float& k ) //from xyz position
    {
      float max_length = LTHIGH + LCALF;
      if ( x < -max_length || x > max_length || y < 0 || y > max_length ||
           z < -150 || z > 150 )
      {
        if ( flags.isset( FLAG_DEBUG) )
          println( "E", 1 );
        return false;
      }
      
      //RG aug 19 2024. bug was here
      //if right legs, then ROTRADIUS must be subtracted, not added
      double o = (leg_id <= 1 ? -ROTRADIUS : ROTRADIUS );      
      double oplusz = o + z;
      double dsq = y * y + oplusz * oplusz;
      double d = sqrt( dsq );
      double cosa = oplusz / d;
      double a = acos( cosa );
      double cosaplusr = o / d;
      double aplusr = acos( cosaplusr );

      r = aplusr - a;//in radians
      if ( isnan(r ) )
      {
        if ( flags.isset( FLAG_DEBUG) )
          println( "E", 2 );
        return false;
      }

      r = r * RAD_TO_DEG;
      r = -r;


      double lsq = dsq - ROTRADIUS * ROTRADIUS;
      double l = sqrt( lsq );
      //if ( debug )
      //  println( "r", r );
      //now the 2d part
      return calcAngles( x, l, h, k );
    }

    /* not used for now, but keep
      void calcPosition( float hip_angle, float knee_angle, float& x, float& y ) //from angles
      {
      knee_angle = 180.0 - knee_angle;
      //find height of knee below hip
      double sa_rad = hip_angle * DEG_TO_RAD;
      double yk = LTHIGH * cos( sa_rad );
      double xk = LTHIGH * sin( sa_rad  );

      //find height of foot below knee
      double ak_rad = (180.0 - hip_angle - knee_angle) * DEG_TO_RAD;
      double yf = LCALF * cos( ak_rad );
      double xf = LCALF * sin( ak_rad );

      y = -yk - yf;

      double xz = xk - xf;

      //now find x and z from xz
      //double wa_rad = waist_angle * deg2rad;
      x = xz; // * cos( wa_rad);
      //z = 0; //-xz * sin( wa_rad);

      y = -y;
      }
    */
    void calcPositionXYZ(  float rot_angle,
                           float hip_angle,
                           float knee_angle,
                           float& x, float& y, float& z ) //from angles
    {
      knee_angle = 180.0 - knee_angle;
      //find height of knee below hip
      double sa_rad = hip_angle * DEG_TO_RAD;
      double yk = LTHIGH * cos( sa_rad );
      double xk = LTHIGH * sin( sa_rad  );

      //find height of foot below knee
      double ak_rad = (180.0 - hip_angle - knee_angle) * DEG_TO_RAD;
      double yf = LCALF * cos( ak_rad );
      double xf = LCALF * sin( ak_rad );

      y = -yk - yf;

      double xz = xk - xf;

      //now find x and z from xz
      //double wa_rad = waist_angle * deg2rad;
      x = xz; // * cos( wa_rad);
      //z = 0; //-xz * sin( wa_rad);

      y = -y;

      //this is x and y assuming leg was vertical with zero rot_angle
      //when rotator rotates, x remains the same, only y changes
      double r = rot_angle * DEG_TO_RAD;
      double cosr = cos(r);
      double sinr = sin(r);
      double o = (leg_id <=1 ? -ROTRADIUS : ROTRADIUS );
      z = ( o * cosr ) - ( y * sinr );
      z -= o; //remove effect of rot_radius
      //update y
      y = y * cosr + o * sinr;
    }
    /*
      void getPosition( float* pos, bool target, bool remove_offsets ) //from angles
      {
      bool reversed = leg_info[leg_id].flags.isset(REVERSED);
      if ( !target )
      {
        pos[0] = cur_pos[0];
        calcPosition( ( reversed ? -cur_pos[1] : cur_pos[1] ),
                      ( reversed ? -cur_pos[2] : cur_pos[2] ),
                      pos[1], pos[2] );
      }
      else
      {
        pos[0] = target_pos[0];
        calcPosition( (reversed ? -target_pos[1] : target_pos[1] ),
                      (reversed ? -target_pos[2] : target_pos[2] ),
                      pos[1], pos[2] );
      }
      //fix for left right back forward reversing of motor angles
      pos[0] = ( leg_id == 1 || leg_id == 3 ? -pos[0] : pos[0] );

      if ( remove_offsets )
        for ( char i = 0; i < 3; i++ )
          pos[i] -= offsets[leg_id][i];

      }
    */
    void getPositionXYZ( float* pos, bool target, bool remove_offsets ) //from angles
    {
      /*
        if ( j == 0 )
            {
              if ( l == 1 || l == 3 )
                apos = -apos;
            }
        else if ( leg_info[l].reversed )
              apos = -apos;
      */
      bool reversed = leg_info[leg_id].flags.isset(REVERSED);
      if ( !target )
      {
        calcPositionXYZ( ( (leg_id == 1 || leg_id == 3) ? -cur_pos[0] : cur_pos[0] ),
                         ( reversed ? -cur_pos[1] : cur_pos[1] ),
                         ( reversed ? -cur_pos[2] : cur_pos[2] ),
                         pos[0], pos[1], pos[2] );
      }
      else
      {
        calcPositionXYZ( ((leg_id == 1 || leg_id == 3)  ? -target_pos[0] : target_pos[0] ),
                         (reversed ? -target_pos[1] : target_pos[1] ),
                         (reversed ? -target_pos[2] : target_pos[2] ),
                         pos[0], pos[1], pos[2] );
      }
      //fix for left right back forward reversing of motor angles
      //pos[0] = ( leg_id == 1 || leg_id == 3 ? -pos[0] : pos[0] );

      //transform to apply rotation of body top

      //printpos( pos, "1: " );

      //position of foot is with respect to rotated leg origin
      //so find this position with respect to center of body top
      //in an unrotated frame
      float leg_origin[3] = { (leg_id == 0 || leg_id == 2 ? HALF_BODY_LENGTH : -HALF_BODY_LENGTH),
                              (leg_id < 2 ? HALF_BODY_WIDTH : -HALF_BODY_WIDTH), 0
                            }; //from center of robot body top

      float virtual_foot_pos[3] = { -pos[0] + leg_origin[0], -pos[2] + leg_origin[1], -pos[1] };
      //printpos( virtual_foot_pos, "2: " );

      //but actually leg_origin is rotated about robot_top by robot_top_rot
      float foot_pos[3];
      mult( body_rot, virtual_foot_pos, foot_pos );
      //printpos( foot_pos, "3: " );

      //now get relative position of foot, assuming leg origin is at its default position
      //remove leg_origin from robot_foot_pos
      pos[0] = -(foot_pos[0] - leg_origin[0]) - robot_top_pos[0];
      pos[1] = -foot_pos[2]                   - robot_top_pos[2];
      pos[2] = -(foot_pos[1] - leg_origin[1]) - robot_top_pos[1];

      //printpos( pos, "4: " );

      if ( remove_offsets )
        subtract( pos, offsets[leg_id] );
    }

};

struct LegStep
{
  int* pos; //[3];
  int duration : 15; //max 16384,
  bool is_xyz : 1; //must be converted before use
};

struct LegMotion
{
  LegStep* legsteps;
  int8_t len;
  //int8_t offset;
};

struct BodyMotion
{
  //bool continuous;
  LegMotion leg_motion[4];
};


int8_t speed_factor10 = 10; //save three bytes, have to divide by 10 everywhere

float speedFactor()
{
  return (float)speed_factor10 / 10.0;
}

//front-left, back-left, front-right, back-right

/*
  #define DownPos  { 0, 160, 0 } //is_xy now
  #define RestPos  { 0, 200, 0 } //is_xy in mm
  #define TallPos  { 0, 0,  0}

  #define SitPos { 0, 10, 100 }
*/
int DownPos[3] = { 0, 160, 0 }; //is_xy now
int RestPos[3] = { 0, 200, 0 }; //is_xy in mm
int TallPos[3] = { 0, 0,  0};

int SitPos[3]  = { 0, 10, 100 };

LegStep Crouch[] = {
  { DownPos, 800, true },
  { DownPos, 500, true }, //stay there
  { RestPos, 800, true },
};

LegStep Sit[] = {
  { DownPos, 1000, true },
  { SitPos,  1800, false },
};

LegStep Stand[] = {
  { RestPos, 1000, true },
};

//#define GetupPos { 0, 50, 120 }
int GetupPos[3] =  { 0, 50, 120 };

LegStep Getup[] = {
  { GetupPos, 1200, false },
  { DownPos, 500, true },
  { RestPos, 1000, true }
};

LegStep Tall[] = {
  { TallPos, 1000, false }
};

LegStep GetupToTall[] = {
  { GetupPos, 1200, false },
  { DownPos, 500, true },
  { RestPos, 800, true },
  { TallPos, 1000, false }
};


//need a way to specify diff sequences for diff legs
//and also a "wait at current" position position
BodyMotion BodyCrouch =
{
  {
    { Crouch, asize(Crouch) },
    { Crouch, asize(Crouch) },
    { Crouch, asize(Crouch) },
    { Crouch, asize(Crouch) },
  }
};

BodyMotion BodySit =
{
  {
    { Sit, asize(Sit) },
    { Sit, asize(Sit) },
    { Sit, asize(Sit) },
    { Sit, asize(Sit) },
  }
};

BodyMotion BodyStand =
{
  {
    { Stand, asize(Stand) },
    { Stand, asize(Stand) },
    { Stand, asize(Stand) },
    { Stand, asize(Stand) },
  }
};
BodyMotion BodyTall =
{
  {
    { Tall, asize(Tall) },
    { Tall, asize(Tall) },
    { Tall, asize(Tall) },
    { Tall, asize(Tall) },
  }
};
BodyMotion BodyGetup =
{
  {
    { Getup, asize(Getup) },
    { Getup, asize(Getup) },
    { Getup, asize(Getup) },
    { Getup, asize(Getup) },
  }
};

BodyMotion BodyGetupToTall =
{
  {
    { GetupToTall, asize(GetupToTall) },
    { GetupToTall, asize(GetupToTall) },
    { GetupToTall, asize(GetupToTall) },
    { GetupToTall, asize(GetupToTall) },
  }
};


BodyMotion BodyBow =
{
  {
    { Crouch, asize(Crouch) },
    { NULL, 0 },
    { Crouch, asize(Crouch) },
    { NULL, 0 },
  }
};

class Body
{
  public:
    Leg legs[NUM_LEGS];
    BodyMotion* cur_body_motion = NULL;
    GaitRobot* cur_gait = NULL;
    bool manual_stepping_mode = false;
    bool manual_step_allowed = false;

    void init()
    {
      unsigned long curtime = millis();
      for ( int8_t i = 0; i < NUM_LEGS; i++ )
      {
        legs[i].setup( i );
        int8_t init_pos[3] = {0};
        legs[i].goTo( init_pos, //leg_info[i].init_pos,
                      1000, leg_info[i].flags.isset(REVERSED),
                      curtime, leg_info[i].flags.isset( IS_XYZ ) );
      }

      legs[0].index = 0;
      legs[1].index = 4;
      legs[2].index = 2;
      legs[3].index = 6;

      //test
      //robot_top_rot[2] = 20;
      updateBodyRot();
    }

    void directGoto( int j, float pos ) //which motor from 0 to 11
    {
      int leg_id = j / 3;
      int motor_id = j % 3;
      legs[leg_id].directGoto( motor_id, pos );
    }

    void stop()
    {
      cur_gait = NULL;
      robot_gait = NULL;
      cur_body_motion = NULL;
      gait_direction = true;
      for (int8_t i = 0; i < 4; i++ )
        legs[i].end_time = 0; //make it stop
    }
    void beginGait( GaitRobot* gait, bool direction )
    {
      cur_body_motion = NULL;

      cur_gait = gait;
      gait_direction = direction;

      /*
      if ( num_pending_steps > 0 )
      {
        //single step mode?
        cur_gait[0].gait.xyz[0].clipmin = 0; //dont let this foot go behind center
        cur_gait[3].gait.xyz[0].clipmin = 0; //dont let this foot go behind center

        cur_gait[1].gait.xyz[0].clipmax = 0; //dont let this foot go in front of center
        cur_gait[2].gait.xyz[0].clipmax = 0; //dont let this foot go in front of center
      }
      else //restore
      {
        cur_gait[0].gait.xyz[0].clipmin = -100;
        cur_gait[3].gait.xyz[0].clipmin = -100; //bug was here (wrong sign), delete after showing to Raghav

        cur_gait[1].gait.xyz[0].clipmax = 100;
        cur_gait[2].gait.xyz[0].clipmax = 100;
      }
      */
      for (int8_t l = 0; l < 4; l++ )
      {
        legs[l].index = cur_gait[l].phase_offset;
        for (int8_t j = 0; j < 3; j++ ) //reset gait offset
          gait_offsets[l][j] = 0;
      }
    }
    void beginMotion( BodyMotion* body_motion )
    {
      cur_gait = NULL;
      robot_gait = NULL;
      for (int8_t i = 0; i < 4; i++ )
        legs[i].index = -1; //next ++ will make it zero
        //legs[i].index = (body_motion->continuous ? body_motion->leg_motion[i].offset - 1 : -1) ;
      cur_body_motion = body_motion;
    }

    bool loop() //returns true if something moved
    {
      unsigned long curtime = millis();
      bool none_moving = true;
      float speed_factor = speedFactor();

      for ( int8_t i = 0; i < 4; i++ )
      {
        //Serial.print( "Leg index: " );
        //Serial.println( i );
        if ( legs[i].end_time == 0 ||
             legs[i].end_time <= curtime
           )
        { //move to next step
          if ( cur_gait && (!manual_stepping_mode || manual_step_allowed) )
          {

            //float gait_step_ms = GAIT_STEP_MS * (float)(cur_gait[i].gait.xyz[0].amplitude +
            //                                    cur_gait[i].gait.xyz[1].amplitude +
            //                                    cur_gait[i].gait.xyz[2].amplitude )  / 30.0;
            float gait_step_ms = (float)cur_gait[i].gait.getTotalDuration() / (float)gait_step;
            if ( i == 0 && flags.isset( FLAG_DEBUG) )
            {
              Serial.print( F("deg\t") );
              Serial.print( legs[i].index );
              //println( "gait_step_ms", gait_step_ms );
            }

            int duration = gait_step_ms; //constant duration, but increased X and Z stride
            float pos[3];
            int vdeg = cur_gait[i].gait.getPosition( i, legs[i].index, pos, speed_factor, turn_right, shuffle_right );
            //if a cg offset is defined by this step, then execute it
            //but how, apply to global offsets, or add another cg offset global
            //todo: ideally need separate global, but for now modify regular offsets
            if ( cur_gait[i].num_cg_offsets > 0 )
            {
              int8_t cycle_index = (float)legs[i].index / 360.0 * cur_gait[i].num_cg_offsets;
              for ( int8_t l = 0; l < 4; l++ )
              {
                for (int8_t j = 0; j < 3; j++ )
                {
                  if ( cur_gait[i].cg_offsets[j] )
                    gait_offsets[l][j] = cur_gait[i].cg_offsets[j][cycle_index]; //.o[j];
                }
              }
            }

            for (int8_t j = 0; j < 3; j++ )
            {
              //technically, we have calculated and saved the default main_offset
              //at a speed/stride-length of 1, we should make it proportional
              //dont change y offset though, only x and z since they denote horizontal movement
              pos[j] += ( (j == 1 ? 1.0 : speed_factor) * (float)cur_gait[i].main_offset[j] ) + gait_offsets[i][j];
            }
            if ( i == 0 && flags.isset( FLAG_DEBUG) )
            {
              for ( int8_t k = 0; k < 3; k++ )
              {
                Serial.print( '\t' );
                Serial.print( pos[k] );
              }
              Serial.print( '\t' );
              Serial.println( vdeg );
            }
            /*
              if ( debug && i == 0 )
              {
              println( "Deg", legs[i].index );
              for(int k = 0; k < 3; k++ )
                Serial.println( pos[k] );
              }
            */
            legs[i].goTo( pos, duration, leg_info[i].flags.isset(REVERSED),
                          curtime, true );
            none_moving = false;

            //legs[i].index += (gait_direction ? gait_step : -gait_step );
            //if this is the last step in counted steps
            //then we dont want to move a leg if it has reached its center position
            //but we need to let the remaining legs continue moving until they get to their target
            if ( num_pending_steps <= 0 || num_pending_steps > 4 || cur_gait[i].phase_offset == 180 ||
                legs[i].index != 360 )
            {
              legs[i].index += (gait_direction ? gait_step : -gait_step );
              if ( legs[i].index >= 360 )
              {
                legs[i].index -= 360;
                if ( num_pending_steps > 0 )
                  num_pending_steps--;
              }
              if ( legs[i].index < 0 ) //perhaps walking backward
              {
                legs[i].index += 360;
                if ( num_pending_steps > 0 )
                  num_pending_steps--;
              }
            }
            continue;
          }
          if ( !cur_body_motion )
            continue;
          legs[i].index++;
          LegMotion& leg_motion = cur_body_motion->leg_motion[i];
          //rg removing mar 27 2025, we dont really have continuous body motions
          //if ( cur_body_motion->continuous && legs[i].index >= leg_motion.len)
          //{
          //  legs[i].index = 0;
          //  legs[i].start_time = curtime;
          //}
          if ( leg_motion.legsteps != NULL &&
               legs[i].index < leg_motion.len ) //not done
          {
            if ( legs[i].index < 0 )
              legs[i].index = 0;
            int index = legs[i].index;
            {
              //Serial.print( "Leg index: " );
              //Serial.println( index );
            }
            int duration = ((float)leg_motion.legsteps[index].duration / speed_factor);
            legs[i].goTo( leg_motion.legsteps[index].pos, duration, leg_info[i].flags.isset(REVERSED),
                          curtime, leg_motion.legsteps[index].is_xyz );
            none_moving = false;
          }
        }
        else
        {
          none_moving = false;
          legs[i].loop( curtime);
        }
        manual_step_allowed = false;
      }
      if ( cur_gait and num_pending_steps == 0 )
      {
        cur_gait = NULL;
        robot_gait = NULL;
        num_pending_steps = -1;
        //this was the last step, now go to stand
        beginMotion( &BodyStand );
      }
      if ( none_moving )
      {
        //Serial.println( "Motion done" );
        cur_body_motion = NULL;
      }
      return (!none_moving);
    }

};

Body body;

void setup() {
  Serial.begin( 500000 );
  servo_driver.setup();

  body.init();
}

struct CmdMotion
{
  const char* cmd;
  BodyMotion* motion;
};
CmdMotion commands[] = {
  { "crouch", &BodyCrouch },
  { "sit", &BodySit },
  //{ "stand", &BodyStand }, //now executes getup first if too low
  { "getup", &BodyGetup },
  //{ "tall", &BodyTall },
  { "bow", &BodyBow },
  //{ "liedown",  &BodyLieDown },
  //{ "down",     &BodyDown }
};

bool legIdMatches( int8_t l, int8_t leg_id )
{
  return ( (leg_id < 4 && l == leg_id ) || //individual leg
           leg_id == 8 || //all
           (leg_id == 4 && (l == 0 || l == 2 ) ) || //front
           (leg_id == 5 && (l == 1 || l == 3 ) ) || //back
           (leg_id == 6 && (l == 0 || l == 1 ) ) || //left
           (leg_id == 7 && (l == 2 || l == 3 ) )    //right
         );
}

void adjustPos( float* pos, const char* jcmd, float incr )
{
  if ( jcmd[1] == 0 )
  {
    if ( jcmd[0] == 'x' )
      pos[0] -= incr;
    else if ( jcmd[0] == 'X' )
      pos[0] += incr;
    else if ( jcmd[0] == 'y' )
      pos[1] -= incr;
    else if ( jcmd[0] == 'Y' )
      pos[1] += incr;
    else if ( jcmd[0] == 'z' )
      pos[2] -= incr;
    else if ( jcmd[0] == 'Z' )
      pos[2] += incr;
  }
}

void adjustPos( int8_t* pos, const char* jcmd, float incr )
{
  if ( jcmd[1] == 0 )
  {
    if ( jcmd[0] == 'x' )
      pos[0] -= incr;
    else if ( jcmd[0] == 'X' )
      pos[0] += incr;
    else if ( jcmd[0] == 'y' )
      pos[1] -= incr;
    else if ( jcmd[0] == 'Y' )
      pos[1] += incr;
    else if ( jcmd[0] == 'z' )
      pos[2] -= incr;
    else if ( jcmd[0] == 'Z' )
      pos[2] += incr;
  }
}
void adjustXYZPos( int8_t leg_id, const char* jcmd, float incr )
{
  float pos[3];
  body.legs[leg_id].getPositionXYZ( pos, true, true );
  adjustPos( pos, jcmd, incr );
  int duration = fabs(incr) * 10; //5mm = 50ms
  body.legs[leg_id].goTo( pos, duration, leg_info[leg_id].flags.isset(REVERSED), millis(), true );
}

void adjustAngles( int8_t leg_id, const char* jcmd, float increment )
{
  float pos[3];
  for (int8_t i = 0; i < 3; i++ )
    pos[i] = (i == 0 && (leg_id == 1 || leg_id == 3) ? -body.legs[leg_id].cur_pos[i] : body.legs[leg_id].cur_pos[i] );
  int8_t incr = ( (leg_id == 2 || leg_id == 3 ) ? -increment : increment );
  if ( jcmd[1] == 0 )
  {
    if ( jcmd[0] == 'r' )
      pos[0] -= increment;
    else if ( jcmd[0] == 'R' ) 
      pos[0] += increment;
    else if ( jcmd[0] == 'h' )
      pos[1] -= incr;
    else if ( jcmd[0] == 'H' )
      pos[1] += incr;
    else if ( jcmd[0] == 'k' )
      pos[2] -= incr;
    else if ( jcmd[0] == 'K' )
      pos[2] += incr;
  }
  body.legs[leg_id].goTo( pos, 50, false, millis() ); //don't reverse again
}

void startGait( GaitRobot* gait, bool direction )
{
  //todo, need to add reset gait somewhere
  //and verify speed and backward operation at zero boundary
  //also add gait height control

  if ( robot_gait == gait ) //already walking
  {
    if ( direction == gait_direction )
    {
      //do nothing
    }
    else
    {
      //switch direction
      body.beginGait( robot_gait, direction );
    }
  }
  else
  {
    robot_gait = gait;
    //get direction if specified
    body.beginGait( robot_gait, direction );
  }
}

unsigned long last_ultrasonic_when = 0;
int ultrasonic_distance = -1;

void loop() {
  // put your main code here, to run repeatedly:

  if ( Serial.available() > 0 )
  {
    unsigned long curtime = millis();

    char cmd[96];
    char c = Serial.read();
    int len = 0;
    while ( c != '\n' )
    {
      if ( c != -1 )
        cmd[len++] = c;
      //Serial.print( c );
      while ( !Serial.available() )
        delayMicroseconds( 10 );
      c = Serial.read();
    }
    cmd[len] = 0;
    //Serial.println( len );
    //Serial.println( cmd );
    if ( !strcmp( cmd, "noop" ) )
    {
      //do nothing
    }
    else if ( !strcmp(cmd, "stop" ) )
    {
      flags.set( FLAG_PAUSED );
      body.cur_body_motion = NULL;
      body.cur_gait = NULL;
      robot_gait = NULL;
    }
    else if  ( len == 2 and cmd[0] == 'g' and cmd[1] == 'o' ) // ( !strcmp(cmd, "go" ) )
      flags.unset( FLAG_PAUSED );
    else if (  len == 2 and cmd[0] == '9' and cmd[1] == '0' ) //( !strcmp( cmd, "90" ) )
    {
      flags.set( FLAG_PAUSED );
      int zeropos[] = {0, 0, 0};
      for ( int8_t i = 0; i < NUM_LEGS; i++ )
        body.legs[i].directGoto( zeropos );
    }
    else if ( !strcmp(cmd, "don" ) )
      flags.set( FLAG_DEBUG );
    else if ( !strcmp(cmd, "doff" ) )
      flags.unset( FLAG_DEBUG );
    else if ( len > 2 && cmd[0] == 'b' && cmd[1] == ':' ) //motor goto
    {
      int motor_id = atoi( cmd + 2 );
      int pos = 3;
      while ( pos < len && cmd[pos] != ',' )
        pos++;
      if ( cmd[pos] == ',' && len > (pos + 1) )
      {
        int deg = atof( cmd + pos + 1 );
        //Serial.print( motor_id);
        //Serial.print( " go to " );
        //Serial.println( deg );
        //servogoto( motor_id, deg );
        body.directGoto( motor_id, deg );
      }
    }
    else if ( !strcmp( cmd, "on" ) )
    {
      flags.set( FLAG_POWER );
      /*
        for( char i = 0; i < 10; i++ )
        {
        digitalWrite( 2, LOW );
        delay( 10 );
        digitalWrite( 2, HIGH );
        delay( 10 );
        }
      */
      digitalWrite( 2, LOW );
    }
    else if ( !strcmp( cmd, "off" ) )
    {
      flags.unset( FLAG_POWER );
      for ( int8_t i = 0; i < 13; i++ ) //include mouth servo
        servo_driver.stop( i );
      delay( 100 );
      for ( int8_t i = 0; i < 10; i++ )
      {
        digitalWrite( 2, LOW );
        delay( 10 );
        digitalWrite( 2, HIGH );
        delay( 10 );
      }
      digitalWrite( 2, HIGH );
    }
    else if ( len == 1 and cmd[0] == 'p' ) //!strcmp( cmd, "p" ) )
    {
      for (int i = 0; i < 4; i++ )
        body.legs[i].print( true );
    }
    else if (len == 2 and cmd[0] == 'p' and cmd[1] == 'o' ) // ( !strcmp( cmd, "po" ) )
    {
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
          Serial.print(int(offsets[i][j]));
          Serial.print(' ');
        }
        Serial.println();
      }
    }
    else if ( cmd[0] == 'j' && cmd[1] == ',' )
    {
      //j,1,Y  will increase Y for back left
      //j,8,x,3 will decrease x for all 4 legs
      int8_t leg_id = atoi(cmd + 2);
      const char* jcmd = cmd + 4;
      int incr = 5; //default
      if ( len > 6 && cmd[5] == ',' )
      {
        cmd[5] = 0;
        incr = (int)atof( cmd + 6 );
      }
      for ( int8_t l = 0; l < 4; l++ )
      {
        if ( legIdMatches( l, leg_id ) )
          adjustXYZPos( l, jcmd, incr );
      }
    }
    else if ( cmd[0] == 'k' && cmd[1] == ',' )
    {
      int8_t leg_id = atoi(cmd + 2);
      const char* jcmd = cmd + 4;
      int incr = 3; //default
      if ( len > 6 && cmd[5] == ',' )
      {
        cmd[5] = 0;
        incr = atof( cmd + 6 );
      }
      for (int8_t l = 0; l < 4; l++ )
      {
        if ( legIdMatches(l, leg_id ) )
          adjustAngles( l, jcmd, incr );
      }
    }
    else if ( cmd[0] == 'o' && cmd[1] == ',' )
    {
      //save target position before offsets
      float current_target[4][3];
      for ( int8_t i = 0; i < 4; i++ )
        body.legs[i].getPositionXYZ( current_target[i], true, true );

      const char* ocmd = cmd + 2;
      if ( !strcmp( ocmd, "r" ) ) //reset all offsets
      {
        for (int8_t i = 0; i < 4; i++ )
          for (int8_t j = 0; j < 3; j++ )
            offsets[i][j] = 0;
      }
      else
      {
        int8_t leg_id = atoi(ocmd);
        const char* jcmd = cmd + 4;
        int incr = 5; //default
        if ( len > 6 && cmd[5] == ',' )
        {
          cmd[5] = 0;
          incr = atof( cmd + 6 );
        }
        for (int8_t l = 0; l < 4; l++ )
        {
          if ( legIdMatches(l, leg_id ) )
            adjustPos( offsets[l], jcmd, incr );
        }
      }
      //now offsets are changed
      for ( int8_t i = 0; i < 4; i++ )
        body.legs[i].updateTarget( current_target[i], curtime );

      //print offsets
      /*
        if ( debug )
        {
        for (int i = 0; i < 4; i++) {
          for (int j = 0; j < 3; j++) {
            Serial.print(int(offsets[i][j]));
            Serial.print(" ");
          }
          Serial.println();
        }
        Serial.println("---");
        }
      */
    }
    else if ( len > 2 && cmd[0] == 's' && cmd[1] == ':' ) //set speed factor
    {
      speed_factor10 = atof( cmd + 2 ) * 10.0;
      /*
        if ( debug )
        {
        Serial.print( "Setting speed factor to " );
        Serial.println( speed_factor );
        }
      */
    }
    else if ( len > 2 and cmd[0] == 'S' and (cmd[1] == ',' or cmd[1] == ':' ) )
    {
      //single step mode while gait
      num_pending_steps = atoi(cmd + 2) * 4; //one for each leg
    }
    else if ( len == 1 and cmd[0] == 'S' )
    {
      num_pending_steps = -1;
    }
    else if ( len > 2 && (cmd[0] == 'r' || cmd[0] == 'R') && (cmd[1] == ':' || cmd[1] == ',') ) //body top rotation
    {
      //lowercase r, absolute rotation. uppercase R, relative from current rotation
      //save target position before offsets
      bool relative = (cmd[0] == 'R');
      float current_target[4][3];
      for ( int8_t i = 0; i < 4; i++ )
        body.legs[i].getPositionXYZ( current_target[i], true, true );

      //body_top rotation angle: r:x,y,z
      char* vals[5];
      int n = split( cmd + 2, ',', 5, vals );
      if ( n >= 3 )
      {
        for ( int8_t i = 0; i < 3; i++ )
        {
          robot_top_rot[i] = (relative ? robot_top_rot[i] : 0 ) + atof(vals[i]) * 10.0;
          //clip to between -45 and +45 degs to prevent over-twisting
          if ( robot_top_rot[i] > 450 ) //45 deg times 10
            robot_top_rot[i] = 450;
          else if ( robot_top_rot[i] < -450 ) //-45 deg times 10
            robot_top_rot[i] = -450;
        }
        int duration = 500;
        if ( n > 3 )
          duration = atoi( vals[3] );
        updateBodyRot();
        //cause motors to move to update to go back to previous position
        for ( int8_t i = 0; i < 4; i++ )
          body.legs[i].updateTarget( current_target[i], curtime, duration );
      }
    }
    else if ( len > 2 && cmd[0] == 'l' && (cmd[1] == ':' or cmd[1] == ',') ) //move leg to
    {
      //first val after : is the leg_id,
      //each of the three comma sep numbers after leg_id specifies the position
      //of rotation, hip, and knee
      char* vals[8];
      int n = split( cmd + 2, ',', 8, vals );
      if ( n > 3 )
      {
        int leg_id = atoi( vals[0] );
        float pos[3];
        for ( int8_t i = 0; i < 3; i++ )
        {
          if ( vals[i + 1][0] == 0 ) //blank, ignore this motor, or leave it where it is
            pos[i] = -1000; //ignored if below -500
          else
            pos[i] = atof(vals[i + 1]);
        }
        body.legs[leg_id].directGoto( pos );
      }
    }
    else if ( len > 2 && cmd[0] == 'e' && ( cmd[1] == ':' or cmd[1] == ',') ) //set all "tall" offsets to
    {
      //12 numbers, rotator, hip, knee for each leg
      //each number if new error_offset
      char* vals[14];
      int n = split( cmd + 2, ',', 14, vals );
      if ( n > 11 )
      {
        int8_t i = 0;
        for (int8_t l = 0; l < 4; l++ )
          for ( int8_t j = 0; j < 3; j++ )
          {
            float offset_pos = atof( vals[i++] );
            if ( offset_pos > -100 and offset_pos < 100 )
            {
              //first adjust target and current pos of joint
              float change = offset_pos - leg_info[l].ninety_offset_pos[j];
              body.legs[l].cur_pos[j] -= change;
              body.legs[l].target_pos[j] -= change;
              leg_info[l].ninety_offset_pos[j] = offset_pos;
            }
          }
      }
    }
    /*
      else if ( len > 2 && cmd[0] == 'g' && cmd[1] == ':' ) //move leg to x, y
      {
       //each of the three comma sep numbers after : specifies the position
       //of rotation, hip, and knee
       char* vals[10];
       int n = split( cmd + 2, ',', 10, vals );
       if ( n > 2 )
       {
         int leg_id = atoi( vals[0] );
         float x = atof( vals[1] );
         float y = atof( vals[2] );
         float pos[] = { 0, 0, 0 };
         body.legs[leg_id].calcAngles( x, y, pos[1], pos[2] );
         body.legs[leg_id].directGoto( pos );
       }
      }
    */
    else if ( len == 1 && cmd[0] == 'i' ) //all info
    {
      Serial.print( F("info,") );
      Serial.print( ( flags.isset( FLAG_POWER ) ? 1 : 0 ) );
      Serial.print( ',' );
      Serial.print( ( flags.isset( FLAG_PAUSED ) ? 1 : 0 ) );
      Serial.print( ',' );
      Serial.print( ultrasonic_distance );
      //Serial << "info,"
      //       << ( flags.isset( FLAG_POWER ) ? 1 : 0 ) <<  ","
      //       << ( flags.isset( FLAG_PAUSED ) ? 0 : 1 ) //enabled, opposite of paused
      //       << ","
      //       << ultrasonic_distance;
      for ( int8_t l = 0; l < 4; l++ )
      {
        for ( int8_t j = 0; j < 3; j++ )
        {
          float apos = body.legs[l].cur_pos[j];
          if ( j == 0 )
          {
            if ( l == 1 || l == 3 )
              apos = -apos;
          }
          else if ( leg_info[l].flags.isset(REVERSED) )
            apos = -apos;

          Serial.print( ',' );
          Serial.print( round(apos) );
        }
      }
      Serial.println();
    }
    else if ( len == 1 && cmd[0] == 'u' ) //cur actual positions without offsets or sign reversal etc
    {
      Serial.print( F("apos,") );
      Serial.print( speedFactor() );
      for ( int8_t l = 0; l < 4; l++ )
      {
        for ( int8_t j = 0; j < 3; j++ )
        {
          float apos = body.legs[l].cur_pos[j] + leg_info[l].ninety_offset_pos[j];
          Serial.print( ',' );
          Serial.print( round(apos) );
        }
      }
      Serial.println();
    }
    else if ( len == 1 && cmd[0] == 'I' ) //upper case I
    {
      //robot_top_rot, robot_top_pos
      Serial.print( 'I' );
      for ( int i = 0; i < 3; i++ )
      {
        Serial.print( ',' );
        Serial.print( (float)robot_top_rot[i] / 10.0 ); //stored as int after mult by 10
      }
      //printpos2( robot_top_pos, "," );
      Serial.println();
    }
    else if ( len == 1 && cmd[0] == 'v' )
    {
      Serial.println( FIRMWARE_VERSION );
    }
    else if ( len == 1 && cmd[0] == 't' ) //all info as xyz
    {
      Serial.print( F("xyz,") );
      Serial.print( (flags.isset( FLAG_POWER ) ? 1 : 0 ) );
      Serial.print( ',' );
      Serial.print( (flags.isset( FLAG_PAUSED )  ? 0 : 1 ) ); //enabled, opposite of paused
      Serial.print( ',' );
      Serial.print( ultrasonic_distance );

      for ( int8_t l = 0; l < 4; l++ )
      {
        float cur_xyz[3];
        body.legs[l].getPositionXYZ( cur_xyz, false, true );
        for ( int8_t j = 0; j < 3; j++ )
        {
          Serial.print( ',' );
          Serial.print( round(cur_xyz[j]) );
        }
      }
      Serial.println();
    }
    else if ( len > 2 && cmd[0] == 'd' && cmd[1] == ',' ) //move leg to x, y, z over time duration
    {
      //d,<leg_id>,<duration_ms>,x,y,z
      char* vals[8];
      int n = split( cmd + 2, ',', 10, vals );
      if ( n > 3 )
      {
        int8_t leg_id = atoi( vals[0] );
        //int duration = atoi( vals[1] );
        float pos[3] = { atof( vals[2] ), //x
                         atof( vals[3] ), //y
                         ( n > 4 ? atof( vals[4] ) : 0 ) //z
                       };
        body.legs[leg_id].goTo( pos,
                                atoi( vals[1] ), //duration
                                leg_info[leg_id].flags.isset(REVERSED),
                                curtime,
                                true );
      }
    }
    else if ( cmd[0] == 'w' && cmd[1] == ',' )
    { //w,duration,12 motors r,h,k angles for 4 legs
      unsigned long curtime = millis();
      char* vals[15];
      int n = split( cmd + 2, ',', 14, vals );
      if ( n >= 12 )
      {
        int duration = atoi( vals[0] );
        //vals[1] is 'r', or rotation of rotator
        float pos[3];
        for ( int8_t i = 0; i < 4; i++ )
        {
          for ( int8_t j = 0; j < 3; j++ )
            pos[j] = atof( vals[1 + (i * 3) + j] );
          body.legs[i].goTo( pos, duration, leg_info[i].flags.isset(REVERSED), //leg_info[leg_id].reversed // raw values
                             curtime );
        }
      }
    }
    else if ( !strncmp( cmd, "trot", 4 ) )
    {
      if ( cmd[4] == ',' and cmd[5] != 0 )
        num_pending_steps = atoi( cmd + 5 ) * 4; //one for each leg
      else if ( cmd[4] == 'b' and cmd[5] == ',' and cmd[6] != 0 )
        num_pending_steps = atoi( cmd + 6 ) * 4; //one for each leg
      startGait( trot_gait, cmd[4] != 'b' );
    }
    else if ( !strncmp( cmd, "walk", 4 ) )
    {
      startGait( walk_gait, cmd[4] != 'b' );
    }
    else if ( !strcmp( cmd, "rg" ) )
    {
      resetGaits();
    }
    else if ( cmd[0] == 'g' and (cmd[2] == 0 || cmd[2] == ',') ) //all the g? commands
    {
      //GaitLegParam* ref_gait_leg_param = getRefGaitLegParam( robot_gait );

      int param = -1;
      if ( cmd[2] == ',' and cmd[3] != 0 )
      {
        param = atoi( cmd + 3 );
      }

      if ( cmd[1] == 'l' ) //"gl" gait turn left
      {
        if ( cmd[2] == ',' and cmd[3] != 0 )
          turn_right -= param;
        else
          turn_right--;
      }
      else if ( cmd[1] == 'r' ) // "gr" gait turn right
      {
        if ( cmd[2] == ',' and cmd[3] != 0 )
          turn_right += param;
        else
          turn_right++;
      }
      else if ( cmd[1] == 's' ) // "gs" gait go straight
      {
        turn_right = 0;
        shuffle_right = 0;
      }
      else if ( cmd[1] == 'x' ) //( !strcmp( cmd, "gx" ) ) //gait increase X stride
      {
        incrGaitAmplitude( robot_gait, 0, 3 ); //x
      } 
      else if ( cmd[1] == 'X' ) //( !strcmp( cmd, "gX" ) ) //gait decrease X stride
      {
        incrGaitAmplitude( robot_gait, 0, -3 ); //x
      }
      else if ( cmd[1] == 'y' ) //( !strcmp( cmd, "gy" ) ) //gait increase Y stride
      {
        incrGaitAmplitude( robot_gait, 1, 3 ); //y
      }
      else if ( cmd[1] == 'Y' ) //( !strcmp( cmd, "gY" ) ) //gait decrease Y stride
      {
        incrGaitAmplitude( robot_gait, 1, -3 );
      }
      else if ( cmd[1] == 'd' ) //( !strcmp( cmd, "gd" ) ) //reverse
      {
        gait_direction = false;
      }
    }
    else if ( !strcmp( cmd, "sl" ) ) //gait shuffle left
    {
      shuffle_right--;
    }
    else if ( !strcmp( cmd, "sr" ) ) //gait shuffle right
    {
      shuffle_right++;
    }
    else if ( !strcmp( cmd, "ms" ) ) //manual stepping mode
    {
      body.manual_stepping_mode = !body.manual_stepping_mode;
      body.manual_step_allowed = false;
    }
    else if ( !strcmp( cmd, "mn" ) ) //manual stepping mode next step
    {
      body.manual_step_allowed = true;
    }
    else if ( !strcasecmp( cmd, "stand" ) || !strcasecmp( cmd, "tall" ) )
    {
      //if body is too low, then execute getup instead of stand
      float current_target[3];
      int max_y = 0;
      for ( int8_t i = 1; i < 3; i += 2 ) //only back legs
      {
        body.legs[i].getPositionXYZ( current_target, true, true );
        if ( current_target[1] > max_y )
          max_y = current_target[1];
        if ( !strcasecmp( cmd, "stand" )  )
          body.beginMotion(  max_y < 150 ? &BodyGetup : &BodyStand );
        else if ( !strcasecmp( cmd, "tall" ) )
          body.beginMotion( max_y < 150 ? &BodyGetupToTall : &BodyTall );
      }
    }
    else
    {
      for ( uint8_t i = 0; i < asize( commands ); i++ )
      {
        if ( !strcmp( commands[i].cmd, cmd ) )
        {
          robot_gait = NULL;
          body.beginMotion( commands[i].motion );
        }
      }
    }
  }

  //stuff that happens every loop
  unsigned long curtime = millis();
  //RG: June 20, 2024:
  //does it mean, 10 times a second we are waiting for 5ms for the ultrasonic pulseIn response?
  //thats too much!! we need to see how to make the sensor asynchronous if possible
  if ( (last_ultrasonic_when + 100) < curtime ) //not more than 10 times a second
  {
    ultrasonic_distance = getUltrasonicDistance( A0, A1 );
    last_ultrasonic_when = curtime;
  }
  if ( !flags.isset( FLAG_PAUSED ) )
  {
    if ( !body.loop() )
    {
      if ( !Serial.available()  )
        delay( 5 ); //nothing happened, so wait a bit
    }
    //else
    //{
    //delay( 1 ); not needed since servodriver is pretty slow anyway
    //}
  }
  else
  {
    if ( !Serial.available() )
      delay( 10 );
  }
}
