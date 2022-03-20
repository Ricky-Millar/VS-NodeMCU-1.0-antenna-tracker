#include <mavlink.h>
#include <ESP_EEPROM.h>
#include <ESP8266WiFi.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#include "angle_calculator.h"
#include "servo_conversions.h"
#ifndef STASSID
#define STASSID "tbs_tango2_E868E764BB2F";
#define STAPSK "If you have a password put it here";
#endif
int old_sat_num;
bool DEBUG = true;
const char *password = STAPSK;
const char *ssid = STASSID;
// the ip and port to connect to on the TSB Tango II to get a MAVlink stream
const char *host = "192.168.4.1";
const uint16_t port = 5760;
// comp and sys ID are used in mavlink messages so each system knows who its talking too.
const uint16_t CC_SYSID = 245;
const uint16_t CC_COMPID = 1;
double mag_offset = 0;
double pan_angle = 90;
double tilt_angle = 90;
bool is_first_loop = true;
int test = 1;
bool gps_fix = false;
WiFiClient client;
mavlink_message_t msg;
mavlink_status_t status;
uint8_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
Servo pan_servo;
Servo tilt_servo;
// bno055 stuff
unsigned long lastVcc;
unsigned long lastInfo;
ADC_MODE(ADC_VCC);
#define BNO055_I2C_ADDRESS 0x28
#define BNO055_MODE Adafruit_BNO055::OPERATION_MODE_NDOF
#define CLOCK_STRETCH_LIMIT 1000
bool bnoReady;
Adafruit_BNO055 bno(55, BNO055_I2C_ADDRESS);
adafruit_bno055_offsets_t calibrationData;

struct position
{
  double lat;
  double lon;
  double alt;
} current_pos, initial_pos;
struct angles
{
  double alt;
  double ber;
} current_angles;
typedef struct angles angles;
typedef struct position position;


unsigned long proscess_Started = 0;
const long proscess_interval = 100;
unsigned long sim_Started = 0;
const long sim_interval = 3000;

void process_data()
{
  unsigned long currentMillis = millis();
  if (currentMillis - proscess_Started >= proscess_interval) {
    proscess_Started = currentMillis;
  if (!is_first_loop)
  {
    /*---------------------------GETTING IMU DATA----------------------------------*/
    const imu::Vector<3> &vec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    Serial.printf("%s: %f\n", "Mag Offset: ", vec.x());
    // gets the osfett of the device to compensate for its rotation telative to north
    double mag_offset = vec.x();
    
    /*----------------------------SERVO WANGLING------------------------------------*/
    /*The initial coordinates and angle are compared with the drones coordinates to calculate
    the relitive angles between them, those numbers are then sent to the servo motors
    with a bit of fangling to make up for the servo motors being a bit shit*/

    current_angles = getAnglesFromPos(current_pos, initial_pos);
    pan_angle = servo_movement_calculator(current_angles.ber, 5, 175, 90, false, mag_offset);
    tilt_angle = servo_tilt_calculator(current_angles.alt, 10, 175, 90, false);

    pan_servo.write(pan_angle);
    tilt_servo.write(tilt_angle);
    if (DEBUG)
    {
      Serial.println("--------SERVO INFO----------");
      Serial.print("pan Angle:  ");
      Serial.println(pan_angle);
      Serial.print("Tilt Angle:  ");
      Serial.println(tilt_angle);
    }
  }
  }
}

void simulate_flight()
{
  is_first_loop = false;
  /*this is a test function that simulates a drone flying around my house to
  test the motor control code with known values

  this function is hiddin in the wifi part of the setup code, as it means I can loop
  it withought a wifi connection */
  delay(50);
  if (test >= 5)
  {
    test = 1;
  }
  // FOR TESTING WITH KNOWN VALUES
  initial_pos.lat = -43.576568;
  initial_pos.lon = 172.635182;
  initial_pos.alt = 100000;

  switch (test)
  {
  case 1:
    Serial.println("------EAST DOWN");
    // EAST and up -43.576560, 172.6378372
    current_pos.lat = -43.576560;
    current_pos.lon = 172.637837;
    current_pos.alt = 1110000;
    break;
  case 2:
    Serial.println("-------WEST UP");
    // WEST and down -43.576489, 172.6320872
    current_pos.lat = -43.576489;
    current_pos.lon = 172.632087;
    current_pos.alt = 1000;
    break;
  case 3:
    Serial.println("------SOUTH DOWN");
    // SOUTH and up  06271685, 4098379
    current_pos.lat = -43.589547;
    current_pos.lon = 172.634925;
    current_pos.alt = 10000;
    break;
  case 4:
    Serial.println("------NORTH UP");
    // NORTH and down -42.67257331397684, 172.71425128060898
    current_pos.lat = -42.672573;
    current_pos.lon = 172.714251;
    current_pos.alt = 500000;
    break;
  }
  unsigned long currentMillis = millis();
  if (currentMillis - sim_Started >= sim_interval) {
    sim_Started = currentMillis;
  test++;}
  process_data();
}

/*----------------------SETUP-----------------------*/

void setup()
{
  /*initiate servos*/
  // TODO: Tune the miliseconds on the servos
  pan_servo.attach(2, 500, 2500);  // D4  tests say 500-2500 but I get less buzz with 2400
  tilt_servo.attach(0, 500, 2500); // D3
  pan_servo.write(pan_angle);
  tilt_servo.write(tilt_angle);
  /*initiate serial*/
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // BNO005 SETUP
  lastInfo = lastVcc = millis();
  Wire.pins(D1, D2);
  // Enable clock stretching (required as the esp8266 has some funky stuff going on with I2C)
  Wire.setClockStretchLimit(CLOCK_STRETCH_LIMIT);
  // Sets and checks the BNO055 for Nine DOF mode, for orientation fusion data.
  if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    bnoReady = false;
  }
  else
  {
    Serial.println("Initialized BNO055!");
    bnoReady = true;
  }

  // LOAD FROM EEPROM This gets the previous bno055 calibration data, meaning the calibration proscess time can be cut down significantly
  EEPROM.begin(sizeof(adafruit_bno055_offsets_t));
  EEPROM.get(0, calibrationData);
  bno.setSensorOffsets(calibrationData);

  delay(200);
   Serial.print("calibrationData.accel_offset_x = ");
  Serial.println(calibrationData.accel_offset_x);
  Serial.print("calibrationData.accel_offset_y = ");
  Serial.println(calibrationData.accel_offset_y);
  Serial.print("calibrationData.accel_offset_z = ");
  Serial.println(calibrationData.accel_offset_z);
  Serial.print("calibrationData.gyro_offset_x = ");
  Serial.println(calibrationData.gyro_offset_x);
  Serial.print("calibrationData.gyro_offset_y = ");
  Serial.println(calibrationData.gyro_offset_y);
  Serial.print("calibrationData.gyro_offset_z = ");
  Serial.println(calibrationData.gyro_offset_z);
  Serial.print("calibrationData.mag_offset_z = ");
  Serial.println(calibrationData.accel_offset_z);
  Serial.print("calibrationData.mag_offset_x = ");
  Serial.println(calibrationData.gyro_offset_x);
  Serial.print("calibrationData.mag_offset_y = ");
  Serial.println(calibrationData.gyro_offset_y);
  Serial.print("calibrationData.accel_radius = ");
  Serial.println(calibrationData.accel_radius);
  Serial.print("calibrationData.mag_radius = ");
  Serial.println(calibrationData.mag_radius);

  // Wait for full calibration of bno055
  while (!bno.isFullyCalibrated())
  {
    Serial.println("spin me around");
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println("Calibration state:");
    Serial.printf("system: %d, gyro: %d, accel: %d, mag: %d\n",
                  system, gyro, accel, mag);
    delay(200);
  }

  // GET CALIBRATION DATA
  bno.getSensorOffsets(calibrationData);
  delay(200);
  Serial.print("calibrationData.accel_offset_x = ");
  Serial.println(calibrationData.accel_offset_x);
  Serial.print("calibrationData.accel_offset_y = ");
  Serial.println(calibrationData.accel_offset_y);
  Serial.print("calibrationData.accel_offset_z = ");
  Serial.println(calibrationData.accel_offset_z);
  Serial.print("calibrationData.gyro_offset_x = ");
  Serial.println(calibrationData.gyro_offset_x);
  Serial.print("calibrationData.gyro_offset_y = ");
  Serial.println(calibrationData.gyro_offset_y);
  Serial.print("calibrationData.gyro_offset_z = ");
  Serial.println(calibrationData.gyro_offset_z);
  Serial.print("calibrationData.mag_offset_z = ");
  Serial.println(calibrationData.accel_offset_z);
  Serial.print("calibrationData.mag_offset_x = ");
  Serial.println(calibrationData.gyro_offset_x);
  Serial.print("calibrationData.mag_offset_y = ");
  Serial.println(calibrationData.gyro_offset_y);
  Serial.print("calibrationData.accel_radius = ");
  Serial.println(calibrationData.accel_radius);
  Serial.print("calibrationData.mag_radius = ");
  Serial.println(calibrationData.mag_radius);
  // SAVE Calibration TO EEPROM
  bno.setSensorOffsets(calibrationData);
  EEPROM.put(0, calibrationData);
  boolean commit_ok = EEPROM.commitReset();
  Serial.println((commit_ok) ? "Commit (Reset) OK" : "Commit failed");

  /*connect to wifi*/
  WiFi.mode(WIFI_STA); // STA = station, e.g. client
  WiFi.begin(ssid);    //(ssid, password); if you need a password
  /*Wait for a connection*/
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    //simulate_flight(); //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TEST FUNCTION
  }
  /* confirm connection */
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/*----------------------MAIN LOOP-----------------------*/
void loop()
{
  /* Create a TCP connection with the port transmiting MAV data */
  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  /*confirm conection to the port*/
  if (!client.connect(host, port))
  {
    Serial.println("connection failed");
    delay(500);
    return;
  }
  /*sends a packet identifying the NodeMCU as a GCS and requesting positonal data*/
  // TODO: Work out why the message interval dosnt work. potentialy set interval command again
  mavlink_msg_data_stream_pack(CC_SYSID, CC_COMPID, &msg, MAV_DATA_STREAM_POSITION, 50, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  client.write(buf, len);
  // Keeps looping whilst wifi is conected.
  Serial.println("Contacting Drone");
  while (client.connected())
  {
    delay(100);
    /*---------------------------Message Handling-------------------------*/
    /*Checks if there are avaliable packets to be read, is so sends
    them to the mavlink library to be parsed, If the individual
      messages can be parsed they are then prosesed case by case*/
    while (client.available())
    {
      uint8_t ch = static_cast<char>(client.read());
      process_data();
      if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status))
      {
        // This switch will identify a message based on it's ID and then proscess it acordingly
        // for common messages : https://mavlink.io/en/messages/common.html
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {

          // checks for a gps fix before calculations
          if (gps_fix == false)
          {
            break;
          }
          mavlink_global_position_int_t gpi;
          mavlink_msg_global_position_int_decode(&msg, &gpi);

          /*The incoming lat/lon data is an int (eg. 72.324234 deg is 72324234) so we
          convert it to a double and add the decimal by dy dividing by a big old number*/
          current_pos.lat = (double)gpi.lat / (double)10000000;
          current_pos.lon = (double)gpi.lon / (double)10000000;
          current_pos.alt = (double)gpi.alt;
          if (DEBUG)
          {
            Serial.println("----GLOBAL_POSITION_INT------");
            Serial.print("lat: ");
            printDouble(current_pos.lat, 1000000);
            Serial.print("lon: ");
            printDouble(current_pos.lon, 1000000);
            Serial.print("alt: ");
            printDouble(current_pos.alt, 1000000);
          }
          // Saves initial gps coordinates as a reference
          if (is_first_loop)
          {
            initial_pos = current_pos;
            is_first_loop = false;
            break;
          }
        }

        /*----------------------------GPS SATILITE CHECKER-----------------------------*/
        /*this will check if the the drone has enough satilites to have a "3D fix" */
        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
          mavlink_gps_raw_int_t gri;
          mavlink_msg_gps_raw_int_decode(&msg, &gri);

          // fix type 3 is a 3d fix https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
          if (gri.fix_type == 3)
          {
            gps_fix = true;
          }
          else if (DEBUG && gri.satellites_visible != old_sat_num)
          {
            Serial.println("--------GPS_RAW_INT----------");
            Serial.print("Number of Satellites: ");
            Serial.println(gri.satellites_visible);
            Serial.print("Fix type: ");
            Serial.println(gri.fix_type);
            old_sat_num = gri.satellites_visible;
          }
          break;
        }
        break;
        }
      }
    }
  }

  /*if the code reaches this point, the conection has been lost and the loop will restart*/
  process_data(); // keeps servo compensating for device movement even with no new data.
  Serial.println();
  Serial.println("-Connection to TX Interupted, Waiting for Bananas-");
  /*Waits to avoid Ddos-ing the TX*/
  delay(300);
}
