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
const char *host = "192.168.4.1";
const uint16_t port = 5760;
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
WiFiClient client;
mavlink_message_t msg;
mavlink_status_t status;
uint8_t len;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
Servo pan_servo;
Servo tilt_servo;
/*----------------------SETUP-----------------------*/
void setup()
{
  /*initiate servos*/
  pan_servo.attach(2);  // D4
  tilt_servo.attach(0); // D3
  pan_servo.write(pan_angle);
  tilt_servo.write(tilt_angle);
  /*initiate serial*/
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  /*connect to wifi*/
  WiFi.mode(WIFI_STA); // STA for station which is client
  WiFi.begin(ssid);    //(ssid, password); if you need a password
  /*Wait for a connection*/
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
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
        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          mavlink_global_position_int_t gpi;
          mavlink_msg_global_position_int_decode(&msg, &gpi);
          Serial.println("----GLOBAL_POSITION_INT------");
          /*The incoming lat/lon data is an int (eg. 72.324234 deg is 72324234) so we
          convert it to a double and add the decimal for later mathsing*/
          double lat_deg = (double)gpi.lat / (double)10000000;
          double lon_deg = (double)gpi.lon / (double)10000000;
          double alt_mm = (double)gpi.alt;
          Serial.print("lat: ");
          printDouble(lat_deg, 1000000);
          Serial.print("lon: ");
          printDouble(lon_deg, 1000000);
          Serial.print("alt: ");
          printDouble(alt_mm, 1000000);
          pan_angle = getBearingAngle(lat_deg, lon_deg, INITIAL_LAT, INITIAL_LON);
          if (pan_angle >= 90){pan_angle = 90;}
          if (pan_angle <= -90){pan_angle = -90;}
          pan_servo.write(lround(90+pan_angle*2));
          //Saves initial gps coordinates as a reference, then uses them to calculate relitive altitude angle
          if (is_first_loop)
          {
            INITIAL_LAT = lat_deg;
            INITIAL_LON = lon_deg;
            INITIAL_ALT = alt_mm;
            is_first_loop = false;
          }
          else
          {
            tilt_angle = getAltAngle(INITIAL_LAT, INITIAL_LON, lat_deg, lon_deg, INITIAL_ALT, alt_mm);
            Serial.println("SERVOPITCH:");
            Serial.print(tilt_angle); 
            if (tilt_angle >= 90){tilt_angle = 90;}//The servo accepts 0-180 as an input but only actualy moves 0-90, this sets 45 as the centre
            if (tilt_angle <= -90){tilt_angle = -90;}
            tilt_servo.write(lround(90-tilt_angle*2));
          }
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

  /*Waits for short time, avoids Ddos-ing the TX*/
  delay(30);
}