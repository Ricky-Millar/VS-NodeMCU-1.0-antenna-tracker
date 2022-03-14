
#include <Servo.h>
#pragma once

double servo_movement_calculator(double bearing, int min_pan, int max_pan, int offset, bool invert);
double servo_tilt_calculator(double bearing, int min_pan, int max_pan, int offset, bool invert);