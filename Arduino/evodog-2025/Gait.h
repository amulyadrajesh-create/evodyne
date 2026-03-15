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

#ifndef _GAIT_H_
#define _GAIT_H_

struct GaitParam
{
    //moves in ellipse
    int amplitude; //in mm
    int center; //mm
    int phase; //degrees in 360 loop
    int8_t clipmin; //percentage of amplitude, -100 to 100
    int8_t clipmax; //percentage of amplitude, -100 to 100

    float getValue( float amp, float deg )
    {
        //float amp = (float)amplitude;
        amp = fabs(amp);
        float val = amp* sin( (float)(deg + phase)* DEG_TO_RAD );
        float minmaxval = amp * (float)clipmin / 100.0;
        if ( val < minmaxval ) val = minmaxval;
        minmaxval = amp * (float)clipmax / 100.0;
        if ( val > minmaxval ) val = minmaxval;
          return (center + val);
    }
};

struct GaitLegParam
{
    GaitParam xyz[3];
    int touchdown_start_deg;
    int touchdown_end_deg;
    int down_duration_ms;
    int up_duration_ms;
    int getTotalDuration() 
    {
      return (down_duration_ms + up_duration_ms);
    }
    //we need to calculate a virtual degree position
    //accounting for different velocities in down and up phases
    int getVirtualDeg( int deg )
    {
      //position in 360 cycle
      //starts with beginning of touchdown
      if ( touchdown_end_deg < touchdown_start_deg )
        touchdown_end_deg += 360;
      int touchdown_deg_range = touchdown_end_deg - touchdown_start_deg;
      int total_ms = down_duration_ms + up_duration_ms;
      float pos = (float)deg / 360.0;
      float ms_into_cycle = pos * (float)total_ms;
            
      if ( ms_into_cycle <= down_duration_ms )
      {
          //scale into deg range of touchdown phase
          deg = (ms_into_cycle / (float)down_duration_ms * (float)touchdown_deg_range) + touchdown_start_deg;
      }
      else //up phase
      {
          //how much into up_phase
          deg = (float)(ms_into_cycle - down_duration_ms)  / 
                  (float)up_duration_ms * (360.0 - touchdown_deg_range) + touchdown_end_deg;
      }
      while( deg > 360 )
        deg -= 360;
      while( deg < 0 )
        deg += 360;

      return deg;
    }
    int getPosition( int leg_id, int deg, float* pos, 
                      float speed_factor, 
                      int8_t turn_right, 
                      int8_t shuffle_right )
    {
        deg = getVirtualDeg( deg );
        //find rotation factors for x and y amplitudes
        float ang = ( ( leg_id == 0 || leg_id == 2 ? 
                        -turn_right -shuffle_right : 
                        turn_right-shuffle_right ) 
                        * 7.5) * DEG_TO_RAD; //7.5 degs per incr
        float amp = sqrt( xyz[0].amplitude*xyz[0].amplitude + xyz[2].amplitude*xyz[2].amplitude );
        pos[0] = xyz[0].getValue( amp* cos(ang), deg ) * speed_factor;
        pos[1] = xyz[1].getValue( xyz[1].amplitude, deg ); //no affect on Y of either turning, or stride speed

        int phase = 0;
        if ( ang < 0 )
          phase = 180;
        pos[2] = xyz[2].getValue( amp * sin(ang), deg + phase) * speed_factor;
        
        return deg;
    }
};

//struct CGOffset
//{
//  uint8_t o[3]; //x, y, z, forward/backward, up-down,left-right
//};

struct GaitRobot
{
    GaitLegParam& gait; //by reference
    int phase_offset;
    //CGOffset* cg_offsets;
    int8_t** cg_offsets;
    uint8_t num_cg_offsets;
    int8_t main_offset[3]; //to establish neutral position of this gait/leg
};

/*
int getCGOffset( int deg, CGOffset* cgoffsets, int n )
{
  //n is number of entries in array around 360 loop
  int i = (float)deg / 360.0 * (float)n;
  return i;
}
*/
void incrGaitAmplitude( GaitRobot* robot_gait, uint8_t which, int8_t howmuch )
{
  //dont let it get negative
  for(uint8_t i = 0; i < 4; i++ )
  {
    int amplitude = robot_gait[i].gait.xyz[which].amplitude + howmuch;
    if ( amplitude < 0 )
      amplitude = 0;
    robot_gait[i].gait.xyz[which].amplitude = amplitude;
  }
}

#endif //_GAIT_H_
