#include <math.h>
#include <Arduino.h>

const double pi = 3.14159265358979;

void printDouble(double val, unsigned int precision)
{
  // prints val with number of decimal places determine by precision
  // NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
  // example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)
  Serial.print(int(val)); // prints the int part
  Serial.print(".");      // print the decimal point
  unsigned int frac;
  if (val >= 0)
    frac = (val - int(val)) * precision;
  else
    frac = (int(val) - val) * precision;
  Serial.println(frac, DEC);
}

double ConvertToRad(double degree)
{
  // Convert Degrees to Radians
  return (degree * (pi / 180));
}

double ConvertToDeg(double rad)
{
  // Convert Radians to Degrees
  return (rad * (180 / pi));
}

double angleFromCoordinate(double lat1, double long1, double lat2,
                           double long2)
{
  // Accepts 2 coordinate IN RADIANS and calulates the relative bearing from north.
  double dLon = (long2 - long1);
  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double brng = ConvertToDeg(atan2(y, x));
  return brng;
}

double getBearingAngle(double lat1, double long1, double lat2,
                       double long2)
{
  // input 2 coordinates and get the bearing between them relative to north
  // accepts values in degrees
  lat1 = ConvertToRad(lat1);
  long1 = ConvertToRad(long1);
  lat2 = ConvertToRad(lat2);
  long2 = ConvertToRad(long2);
  double bearing = angleFromCoordinate(lat1, long1, lat2, long2);
  Serial.print("brng: ");
  Serial.println(bearing);
  return bearing;
}

double getAltAngle(double init_lat, double init_lon, double current_lat,
                   double current_lon, double init_alt, double current_alt)
{

  init_lat = ConvertToRad(init_lat);
  init_lon = ConvertToRad(init_lon);
  current_lat = ConvertToRad(current_lat);
  current_lon = ConvertToRad(current_lon);

  double delta_lat = current_lat - init_lat;
  double delta_lon = current_lon - init_lon;
  double delta_alt = (current_alt / 1000 - init_alt / 1000); // converts from mm to meters
 //I stole this off stack exchange, I have no idea how it works but it is better that what I tried before
  long double ans = pow(sin(delta_lat / 2), 2) +
                    cos(init_lat) * cos(current_lat) *
                        pow(sin(delta_lon / 2), 2);

  ans = 2 * asin(sqrt(ans));

  // Radius of Earth is 6371km
  long double R = 6371000; // in meters

  double distance = ans * R;
  Serial.print("distance:    ");
  Serial.println(distance);
  Serial.print("delta-alt:  ");
  Serial.println(delta_alt);
  double alt_angle = atan(delta_alt / distance);
  Serial.print("alt_angle:   ");
   Serial.println(alt_angle);
  return ConvertToDeg(alt_angle);
}
// EXPERIMENTAL combines magnetometer output with relative bearing

// double getTrueAngle(double relative_bearing, double mag_bearing){
//PUT CODE THAT DOES SOMETHING COOL HERE//
// }
// Will probs need to unclude a calibration step