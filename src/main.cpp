#include <mavlink.h>
#include <ESP8266WiFi.h>
#include <Servo.h>
#include "angle_calculator.h"
#ifndef STASSID
#define STASSID "tbs_tango2_E868E764BB2F"
#define STAPSK "If you have a password put it here"
#endif

const char *password = STAPSK;
const char *ssid = STASSID;
// the ip and port to connect to on the TSB Tango II to get a MAVlink stream
const char *host = "192.168.4.1";
const uint16_t port = 5760;
// comp and sys ID are used in mavlink messages so each system knows who its talking too.
const uint16_t CC_SYSID = 245;
const uint16_t CC_COMPID = 1;
double lat_deg;
double lon_deg;
double alt_mm;
double INITIAL_ALT = 0;
double INITIAL_LAT = -43.576568;
double INITIAL_LON = 172.635182;
double pan_angle = 90;
double tilt_angle = 90;
bool is_first_loop = true;
int test = 1;

WiFiClient client;
mavlink_message_t msg;
mavlink_status_t status;
uint8_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
Servo pan_servo;
Servo tilt_servo;

void simulate_flight()
{
  /*this is a test function that simulates a drone flying around my house to
  test the motor control code with known values

  this function is hiddin in the wifi part of the setup code, as it means I can loop
  it withought a wifi connection */
  delay(3000);
  if (test >= 5)
  {
    test = 1;
  }
  // FOR TESTING WITH KNOWN VALUES
  INITIAL_LAT = -43.576568;
  INITIAL_LON = 172.635182;
  INITIAL_ALT = 100000;

  switch (test)
  {
  case 1:
    Serial.println("------EAST DOWN");
    // EAST and up -43.576560, 172.6378372
    lat_deg = -43576560;
    lon_deg = 1726378372;
    alt_mm = 110000;
    break;
  case 2:
    Serial.println("-------WEST UP");
    // WEST and down -43.576489, 172.6320872
    lat_deg = -43576489;
    lon_deg = 1726320872;
    alt_mm = 90000;
    break;
  case 3:
    Serial.println("------SOUTH DOWN");
    // SOUTH and up  -44.07942798770487, 172.63619786574031
    lat_deg = -44079427;
    lon_deg = 172636197;
    alt_mm = 120000;
    break;
  case 4:
    Serial.println("------NORTH UP");
    // NORTH and down -43.578555, 172.635293
    lat_deg = -43578555;
    lon_deg = 172635293;
    alt_mm = 80000;
    break;
  }
  test++;

  pan_angle = getBearingAngle(lat_deg, lon_deg, INITIAL_LAT, INITIAL_LON) + 180;
  if (pan_angle >= 180)
  {
    pan_angle = 180;
    Serial.println("SERVO PAN MAX");
  }
  if (pan_angle <= 5)
  {
    pan_angle = 5;
    Serial.println("SERVO PAN LOW");
  }

  // tilt_angle = getAltAngle(INITIAL_LAT, INITIAL_LON, lat_deg, lon_deg, INITIAL_ALT, alt_mm);
  // if (tilt_angle >= 80)
  // {
  //   tilt_angle = 80;
  //   Serial.println("SERVO TILT MAX RANGE");
  // }
  // if (tilt_angle <= -80)
  // {
  //   tilt_angle = -80;
  //   Serial.println("SERVO TILT MAX RANGE");
  // }
  // tilt_servo.write(lround(90 - tilt_angle));
  pan_servo.write(lround(pan_angle));
  // Serial.print("TILT COMMAND:");
  // Serial.println(90 + tilt_angle);
  Serial.print("PAM COMMAND:");
  Serial.println(pan_angle);
}

/*----------------------SETUP-----------------------*/

void setup()
{
  /*initiate servos*/
  // TODO: Tune the miliseconds on the servos
  pan_servo.attach(2, 500, 2400);  // D4  tests say 500-2500 but I get less buzz with 2400
  tilt_servo.attach(0, 500, 2400); // D3
  pan_servo.write(pan_angle);
  tilt_servo.write(tilt_angle);
  /*initiate serial*/
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  /*connect to wifi*/
  WiFi.mode(WIFI_STA); // STA = station, e.g. client
  WiFi.begin(ssid);    //(ssid, password); if you need a password
  /*Wait for a connection*/
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    // simulate_flight(); //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< TEST FUNCTION
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
  mavlink_msg_data_stream_pack(CC_SYSID, CC_COMPID, &msg,
                               MAV_DATA_STREAM_POSITION, 50, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  client.write(buf, len);
  Serial.println("Contacting Drone");
  while (client.connected()) // Keeps looping whilst wifi is conected.
  {
    delay(50);
    /*---------------------------Message Handling-------------------------*/
    /*Checks if there are avaliable packets to be read, is so sends
    them to the mavlink library to be parsed, If the individual
      messages can be parsed they are then prosesed case by case*/
    while (client.available())
    {
      uint8_t ch = static_cast<char>(client.read());
      if (mavlink_parse_char(MAVLINK_COMM_0, ch, &msg, &status))
      {
        // This switch will identify a message based on it's ID and then proscess it acordingly
        // for common messages : https://mavlink.io/en/messages/common.html
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          mavlink_global_position_int_t gpi;
          mavlink_msg_global_position_int_decode(&msg, &gpi);
          Serial.println("----GLOBAL_POSITION_INT------");
          /*The incoming lat/lon data is an int (eg. 72.324234 deg is 72324234) so we
          convert it to a double and add the decimal by dy dividing by a big old number*/
          double lat_deg = (double)gpi.lat / (double)10000000;
          double lon_deg = (double)gpi.lon / (double)10000000;
          double alt_mm = (double)gpi.alt;
          Serial.print("lat: ");
          printDouble(lat_deg, 1000000);
          Serial.print("lon: ");
          printDouble(lon_deg, 1000000);
          Serial.print("alt: ");
          printDouble(alt_mm, 1000000);

          // Saves initial gps coordinates as a reference
          if (is_first_loop)
          {
            INITIAL_LAT = lat_deg;
            INITIAL_LON = lon_deg;
            INITIAL_ALT = alt_mm;
            is_first_loop = false;
            break;
          }
          else
          {

            /*----------------------------SERVO WANGLING------------------------------------*/
            /*The initial coordinates are compared with the drones coordinates to calculate
            the relitive angles between them, those numbers are then sent to the servo motors
            with a bit of fangling to make up for the servo motors being a bit shit*/
            // Serial.print("SERVO PAN:");
            // Serial.println(pan_angle);
            //  The servo accepts 0-180 as an input but only actualy moves 0-90 degrees, this makes 45 the equivilent of 0 degrees
            pan_angle = getBearingAngle(lat_deg, lon_deg, INITIAL_LAT, INITIAL_LON) + 180;
            if (pan_angle >= 180)
            {
              pan_angle = 180;
              Serial.println("SERVO PAN MAX");
            }
            if (pan_angle <= 5)
            {
              pan_angle = 5;
              Serial.println("SERVO PAN LOW");
            }

            tilt_angle = getAltAngle(INITIAL_LAT, INITIAL_LON, lat_deg, lon_deg, INITIAL_ALT, alt_mm);
            // Serial.print("SERVO TILT:");
            // Serial.println(tilt_angle);
            if (tilt_angle >= 80)
            {
              tilt_angle = 80;
              Serial.println("SERVO TILT MAX RANGE");
            }
            if (tilt_angle <= -80)
            {
              tilt_angle = -80;
              Serial.println("SERVO TILT MAX RANGE");
            }
            tilt_servo.write(lround(90 - tilt_angle));
            pan_servo.write(lround(pan_angle));
            Serial.print("TILT COMMAND:");
            Serial.println(90 + tilt_angle);
            Serial.print("PAM COMMAND:");
            Serial.println(pan_angle);
          }
          break;

          /*Get Raw satellite data */
          // TODO: why is this giving me random values? Use this to only start main loop once enough sats have been found
          // case MAVLINK_MSG_ID_GPS_RAW_INT:
          // mavlink_gps_raw_int_t gri;
          // mavlink_msg_gps_raw_int_decode(&msg, &gri);
          // Serial.println("--------GPS_RAW_INT----------");
          // Serial.print("Number of Satellites: "); Serial.println(gri.satellites_visible);
          // Serial.print("Fix type: "); Serial.println(gri.fix_type);
        }
      }
    }
  }
  /*if the code reaches this point, the conection has been lost and the loop will restart*/
  Serial.println();
  Serial.println("-Connection to TX Interupted, Waiting for Bananas-");
  /*Waits to avoid Ddos-ing the TX*/
  delay(30);
}
