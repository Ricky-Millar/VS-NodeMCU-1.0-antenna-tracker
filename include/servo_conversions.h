
#include <Servo.h>
#pragma once

double servo_movement_calculator(Servo servo, double bearing, int min_pan, int max_pan, int offset, bool invert);
double servo_tilt_calculator(Servo servo, double bearing, int min_pan, int max_pan, int offset, bool invert);