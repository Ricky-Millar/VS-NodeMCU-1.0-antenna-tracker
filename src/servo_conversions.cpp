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

double servo_movement_calculator(Servo servo, double bearing, int min_pan, int max_pan, int offset, bool invert)
{


   if (bearing >= 0){
       bearing = 360 - (bearing + offset);
   }
   else
    {
        bearing = (bearing + offset) * -1 ;
   }



            Serial.println(bearing);
        if (invert == true){
        bearing = 360 - bearing;




    }
    if (bearing >= max_pan)
    {
        bearing = max_pan;
        Serial.println("SERVO PAN MAX");
    }

    if (bearing <= min_pan)
    {
        bearing = min_pan;
        Serial.println("SERVO PAN LOW");
    }


    return bearing;
}


double servo_tilt_calculator(Servo servo, double bearing, int min_pan, int max_pan, int offset, bool invert)
{

bearing = bearing + offset;


            Serial.println(bearing);
        if (invert == true){
        bearing = 360 - bearing;




    }
    if (bearing >= max_pan)
    {
        bearing = max_pan;
        Serial.println("SERVO TILT MAX");
    }

    if (bearing <= min_pan)
    {
        bearing = min_pan;
        Serial.println("SERVO TILT LOW");
    }


    return bearing;
}
