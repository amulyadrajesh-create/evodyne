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

void subtract( float* pthis, signed char* pother ) //pthis -= pother
{
    for( int k = 0; k < 3; k++ )
        pthis[k] -= pother[k];
}

void add( float* pthis, signed char* pother ) // pthis += pother
{
    for( int k = 0; k < 3; k++ )
        pthis[k] += pother[k];
}

void transpose( float (*rot)[3], float (*trot)[3] )
{
    for (int i = 0; i < 3; i++ )
        for (int j = 0; j < 3; j++ )
            trot[i][j] = rot[j][i];
}

void get3drot( float r, float p, float w, float (*rot)[3] )
{
    r *= DEG_TO_RAD;
    p *= DEG_TO_RAD;
    w *= DEG_TO_RAD;
    float cosw = cos(w);
    float cosp = cos(p);
    float sinw = sin(w);
    float sinp = sin(p);
    float cosr = cos(r);
    float sinr = sin(r);
    /*    
    double rot1[3][3] = 
    {
        { cosw*cosp , cosw*sinp*sinr - sinw*cosr, cosw*sinp*cosr + sinw*sinr },
        { sinw*cosp , sinw*sinp*sinr + cosw*cosr, sinw*sinp*cosr - cosw*sinr },
        { -sinp       , cosp*sinr                       , cosp*cosr                        },
    };
    
    for( int i = 0; i < 3; i++ )
        for( int j = 0; j < 3; j++ )
            rot[i][j] = rot1[i][j];
    */
    rot[0][0] = cosw*cosp; rot[0][1] = cosw*sinp*sinr - sinw*cosr; rot[0][2] = cosw*sinp*cosr + sinw*sinr;
    rot[1][0] = sinw*cosp; rot[1][1] = sinw*sinp*sinr + cosw*cosr; rot[1][2] = sinw*sinp*cosr - cosw*sinr;
    rot[2][0] = -sinp    ; rot[2][1] = cosp*sinr                 ; rot[2][2] = cosp*cosr                 ;
}

float* mult( float (*rot)[3], float* pos, float* result )
{
    for( int i = 0; i < 3; i++ )
    {
        result[i] = 0;
        for( int j = 0; j < 3; j++ )
            result[i] += rot[i][j] * pos[j];
    }
    return result;
}
float* multtranspose( float (*rot)[3], float* pos, float* result )
{
    //multiply with transpose of matrix, without having to create a transpose matrix
    for( int i = 0; i < 3; i++ )
    {
        result[i] = 0;
        for( int j = 0; j < 3; j++ )
            result[i] += rot[j][i] * pos[j]; //switched i and j
    }
    return result;
}
