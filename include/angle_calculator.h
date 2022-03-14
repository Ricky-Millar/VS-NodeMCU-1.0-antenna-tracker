#pragma once
typedef struct angles angles;

const double pi = 3.14159265358979;

void printDouble(double val, unsigned int precision);

double ConvertToRad(double degree);

double ConvertToDeg(double rad);

double angleFromCoordinate(double lat1, double long1, double lat2,
                           double long2);

double getBearingAngle(struct position current_pos, struct position initial_pos);

double getAltAngle(double current_lat, double current_lon, double current_alt, double init_lat, double init_lon, double init_alt);
angles getAnglesFromPos(struct position current_pos, struct position initial_pos);