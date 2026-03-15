/*******************************************************
* Copyright (C) 2020 onwards
* Evodyne Robotics Corporation / Evodyne Robotics Academy
* support@evodyneacademy.com
* Can not be copied and/or distributed under any
* circumstances, directly or
* indirectly, verbatim or modified or derived or inspired
*******************************************************/

#include <Arduino.h>
#include "Power.h"

bool is_power_on = false;

void power( bool on )
{
    //digitalWrite( MOSFET_GATE_PIN, (on ? HIGH : LOW ) );
    is_power_on = on;
    delay( 1 );
}

bool isPowerOn()
{
  return is_power_on;
}
