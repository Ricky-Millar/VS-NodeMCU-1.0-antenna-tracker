#include <Arduino.h>
#include <Servo.h>

/*
this file will control the servos,
it will keep the 360 degree motor smooth and not out of control
around the end of its travel

it will take the raw angles and convert it into something that makes
sence mechanichaly for my set-up

It will take the measurements from the servos position
taken from a 9DOF IMU as a reference of the trackers place-in-space
*/

double servo_movement_calculator(double bearing, int min_pan, int max_pan, int offset, bool invert, double mag_offset)
{    
    
    Serial.println("PRE CALCS--------");
    Serial.print("bearing:  ");
    Serial.println(bearing);
    Serial.print("mag offset:  ");
    Serial.println(mag_offset);

    if (bearing >= 0)
    {
        bearing = 360 - (bearing + offset);
    }
    else
    {
        bearing = (bearing + offset) * -1;
    }

if (mag_offset >= 180) {
    mag_offset = ((mag_offset - 180)-180);
}

    Serial.println("POST CALCS--------");
    Serial.print("bearing:  ");
    Serial.println(bearing);
    Serial.print("mag offset:  ");
    Serial.println(mag_offset);

bearing = bearing + mag_offset;



if (bearing > 360){ bearing = bearing - 360;}

if (bearing < 0){ bearing = 360 - bearing;}

    Serial.println("PRE LIMIT--------");
    Serial.print("bearing:  ");
    Serial.println(bearing);


    Serial.println(bearing);
    if (invert == true)
    {
        bearing = 360 - bearing;
    }

    if (bearing >= max_pan)
    {
        bearing = max_pan;
        Serial.println("-x-x-xSERVO PAN MAX-x-x-x");
    }

    if (bearing <= min_pan)
    {
        bearing = min_pan;
        Serial.println("-x-x-x-SERVO PAN LOW-x-x-x-x");
    }
    Serial.println("FINAL--------");
    Serial.print("bearing:  ");
    Serial.println(bearing);
    return bearing;
}

double servo_tilt_calculator(double bearing, int min_pan, int max_pan, int offset, bool invert)
{

    bearing = bearing + offset;
    Serial.print("offset tilt: ");
    Serial.println(bearing);
    if (invert == true)
    {
        bearing = 360 - bearing;
    }
    if (bearing >= max_pan)
    {
        bearing = max_pan;
        Serial.println("x-x-x-SERVO TILT MAX-x-x-x-x");
    }

    if (bearing <= min_pan)
    {
        bearing = min_pan;
        Serial.println("x-x-x-x-SERVO TILT LOW-x-x-x-x");
    }

    return bearing;
}
