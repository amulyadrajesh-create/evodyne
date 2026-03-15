#define EVOCAR

#define num_dc_motors 2

class EvoCar {
  public:

    struct DCMotorInfo
    {
      int enPin;
      int inp1;
      int inp2;
    };

    int numMotors()
    {
      return num_dc_motors;
    }

    class DCMotor
    {
      public:
        int speed = 0;
        int targetSpeed = 0;
        DCMotorInfo* info;
        int accelerate_incr = 10;
        int decelerate_incr = 20;

        void init( DCMotorInfo* info_)
        {
          speed = 0;
          targetSpeed = 0;

          info = info_;
          pinMode(info->inp1, OUTPUT);
          pinMode(info->inp2, OUTPUT);
          pinMode(info->enPin, OUTPUT);

        }
        void setIncr( int incr_ )
        {
          accelerate_incr = incr_;
          decelerate_incr = accelerate_incr * 2;
        }

        void stop()
        {
          speed = 0;
          targetSpeed = 0;
          analogWrite( info->enPin, 0 );
        }

        void drive ( int targetSpeed_ )
        {
          targetSpeed = targetSpeed_;
        }

        void sendMotorSpeed( int newSpeed, int enPin, int inp1, int inp2, int ultra )
        {
          speed = newSpeed;
          if ( speed > 0 )
          {
            digitalWrite( inp1, HIGH );
            digitalWrite( inp2, LOW );
            int dist = speed * 5;
            if (ultra >= 0 && ultra < dist)
            {
                int maxspeed = (float)dist / 5.0;
                if ( speed > maxspeed )
                    speed = maxspeed;
            }
          }
          else if ( speed < 0 )
          {
            digitalWrite( inp1, LOW );
            digitalWrite( inp2, HIGH );
          }
          else
          {
          }
          int realSpeed = abs( speed );
          analogWrite( enPin, realSpeed );
        }

        void cycle( int ultra )
        {
            int inc = accelerate_incr;
          if ( speed > 0 && targetSpeed > 0 )
          {
              if ( targetSpeed < speed ) //decelerating
                 inc = decelerate_incr;
          }
          else if ( speed < 0 && targetSpeed < 0 )
          {
              if ( targetSpeed > speed ) //decelerating
                 inc = decelerate_incr;
          }
          int newSpeed = speed;

          if ( targetSpeed > (newSpeed + inc) )
          {
            newSpeed += inc;
          }
          else if ( targetSpeed < (newSpeed - inc) )
          {
            newSpeed -= inc;
          }
          else
            newSpeed = targetSpeed;

          sendMotorSpeed( newSpeed, info->enPin, info->inp1, info->inp2, ultra );
        }
    };

    DCMotorInfo dc_motor_info[ num_dc_motors ] =
    {
      //pwm on pins 9 and 10 stops working when servo library
      //is used for even a single servo on any pin!
      //https://forum.arduino.cc/index.php?topic=262661.0
      {3, 5, 6}, //right
      {11,10, 9}  //left
    };

    DCMotor dc_motors[num_dc_motors];

    void init() {
      for (int i = 0; i < num_dc_motors; i++)
      {
        setPwmFrequency( dc_motor_info[i].enPin, 1024);
      }
      for (int i = 0; i < num_dc_motors; i++)
      {
        dc_motors[i].init( &dc_motor_info[i] );
      }
    }

    char* getInfo( char* buf )
    {
      sprintf( buf, "R:%d,L:%d", dc_motors[0].targetSpeed, dc_motors[1].targetSpeed );
      return buf;
    }

    void setIncr( int incr_ )
    {
      for (int i = 0; i < num_dc_motors; i++)
      {
        dc_motors[i].setIncr( incr_ );
      }
    }

    void setPwmFrequency(int pin, int divisor)
    {
      byte mode;
      if (pin == 5 || pin == 6 || pin == 9 || pin == 10)
      {
        switch (divisor) {
          case 1: mode = 0x01; break;
          case 8: mode = 0x02; break;
          case 64: mode = 0x03; break;
          case 256: mode = 0x04; break;
          case 1024: mode = 0x05; break;
          default: return;
        }
        if (pin == 5 || pin == 6)
        {
          TCCR0B = TCCR0B & 0b11111000 | mode;
        } else {
          TCCR1B = TCCR1B & 0b11111000 | mode;
        }
      }
      else if (pin == 3 || pin == 11)
      {
        switch (divisor) {
          case 1: mode = 0x01; break;
          case 8: mode = 0x02; break;
          case 32: mode = 0x03; break;
          case 64: mode = 0x04; break;
          case 128: mode = 0x05; break;
          case 256: mode = 0x06; break;
          case 1024: mode = 0x07; break;
          default: return;
        }
        TCCR2B = TCCR2B & 0b11111000 | mode;
      }
    }

    void processCommand( const char* command )
    {
      int cIndex = -1; //index of the ':'
      int speed = 0;
      if (command[0] == ' '  and command[1] == 0 ) //|| command == "s" || command == "S")
      {
        for (int i = 0; i < num_dc_motors; i ++)
        {
          dc_motors[i].stop();
        }
        return;
      }
      if ( !strcmp(command, "ignore") )
      {
        return;
      }

      int len = strlen( command );

      for (int i = 0; i < len; i++)
      {
        if (command[i] == ':') //while iterating through the string get the index of the ':'
        {
          cIndex = i;
          break;
        }
      }
      //Serial.println( cIndex );
      if (cIndex != -1)
      { //if ':' has been found
        char s = command[cIndex - 1]; //set s to the letter before the ':'
        //Serial.println( command + 2 );
        if ( command[cIndex + 1] == '-')
        {
          speed = -atoi(command + cIndex + 2);
        }
        else
        {
          speed = atoi(command + cIndex + 1);
        }

        //Serial.print( "speed:" );
        //Serial.println( speed );
        if (s == 'r' || s == 'R' )
        {
          dc_motors[0].drive( speed );
          return;
        }
        else if (s == 'l' || s == 'L' )
        {
          dc_motors[1].drive( speed );
          return;
        }
        else if (s == 'm')
        {
          for (int i = 0; i < num_dc_motors; i++)
          {
            dc_motors[i].drive( speed );
          }
        }

      }
    }

    void loop( int ultra )
    {
      for (int i = 0; i < num_dc_motors; i++ )
      {
        dc_motors[i].cycle( ultra );
      }
    }
    void setMotorSpeedLeft( int s )
    {
        dc_motors[1].drive( s );
    }
    void setMotorSpeedRight( int s )
    {
        dc_motors[0].drive( s );
    }
    void stop()
    {
        for(int i = 0; i < num_dc_motors; i++ )
            dc_motors[i].stop();
    }
};
