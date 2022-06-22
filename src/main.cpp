#include <Arduino.h>
#include <stdint.h>
// Airspeed v3
#include <Wire.h>
#include <SPI.h>
#include <ILI9341_t3n.h>
#include <ili9341_t3n_font_ArialBold.h>
#include <ili9341_t3n_font_Arial.h>
// #include <TinyGPS++.h>
// #include "C:\Users\memde\Documents\PlatformIO\Projects\ASI_v3\src\DSEG7_bold_22.h"
#include <Encoder.h>

// NeoGPS
#include <NMEAGPS.h>
#include <GPSport.h>
NMEAGPS gps; // This parses the GPS characters
gps_fix fix; // This holds on to the latest values

// For the E-Hawk ASI and ALT these are the defaults
#define ILI9341_RST 8
#define ILI9341_DC 9
#define ILI9341_CS 10

#define CLK_PIN 2 //  rotary encoder channel A pin
#define DT_PIN 3  //  rotary encoder channel B pin
#define SW_PIN 4  //  rotary encoder switch pin 4.7k pullup
#define INC 1     // % to increment by with each encoder detent (use 1, 2, 4, 5, 10)

//  Custom colors
#define DARK_GREY_1 0x738E // Dark Grey 1
#define DARK_GREY_2 0x4208 // Dark Grey 2
#define DARK_BLUE_1 0x000A // Dark Blue 1

// DLVR airspeed sensor
#define DLVR_I2C_ADDR 0x28
#define STATUS_SHIFT 30
#define TEMPERATURE_SHIFT 5
#define TEMPERATURE_MASK ((1 << 11) - 1)
#define PRESSURE_SHIFT 16
#define PRESSURE_MASK ((1 << 14) - 1)
#define DLVR_OFFSET 8192.0f
#define DLVR_SCALE 16384.0f
#define FSS_inH2O 5.0f // full scale pressure, 2.0 for DLVR-L02D, 5.0 for DLVR-L05D

ILI9341_t3n tft = ILI9341_t3n(ILI9341_CS, ILI9341_DC, ILI9341_RST);
Encoder rotaryEnc(CLK_PIN, DT_PIN);

// airspeed and temperature variables
uint8_t raw_bytes[4];
float airspeed_sum = 0.0;
float airspeed_average = 0.0;
float press_inH2O_raw;
float press_inH2O;

float temp_float;
int temp = 0;
int temp_old = 0;
String tempUnit = " \xB0"
                  "C";

//  rolling average variables for airspeed
const int numReadings = 300;
uint32_t readings[numReadings]; // the readings from the analog input
int readIndex = 0;              // the index of the current reading
uint32_t total = 0;             // the running total
uint32_t average = 0;           // the average

unsigned long previousMillis = 0; // timer for getAirspeed
const long interval = 2;

//  GPS variables
bool gpsFix = false;
bool fixFlag = false;
bool portFlag = false;
int groundspeed = 0;
int groundspeed_old = -1;
int gpscourse = 0;
int gpscourse_old = -1;

long old_encSpeed = -999; //  old position of rotary encoder
long encSpeed = 0;
int airSpeed = 0;
int old_airSpeed = -1;
float airSpeed_ms = 0; // airspeed meters/sec
int airSpeed_mph = 0;
int old_airSpeed_mph = -1;
int groundSpeed = 0;
int old_groundSpeed = -1;
long timer_display_airSpeed = 0;
int delay_display_airSpeed = 0;
int delta = 1;
int tft_width, tft_height;
int airSpeed_x0 = 60;
int airSpeed_y0 = 10;
int groundSpeed_x0 = 224;
int groundSpeed_y0 = 50;

int track = 0;
int old_track = -1;
int gpsAlt = 0;
int old_gpsAlt = -1;
int OAT = 0;
int old_OAT = -1;
int satellites = 0;
int old_satellites = -1;

int hours = 0;
int old_hours = -1;
int minutes = 0;
int old_minutes = -1;
int seconds = 0;
int old_seconds = -1;

int font_width_72 = 56; //  digit shift amount for Arial_72_Bold
int font_width_40 = 33; //  digit shift amount for Arial_40_Bold
int font_width_32 = 26; //  digit shift amount for Arial_32_Bold
int font_width_24 = 18; //  digit shift amount for Arial_24_Bold
int font_width_16 = 16; //  digit shift amount for Arial_16_Bold
int font_width_14 = 12; //  digit shift amount for Arial_14_Bold
int font_width_12 = 8;  //  digit shift amount for Arial_12_Bold

//*****************GET AIRSPEED**********************
void get_airspeed()
{
  Wire.requestFrom(DLVR_I2C_ADDR, 4, true); // requests 4 bytes of data from the sensor

  if (Wire.available())
  {
    //    Serial.println("WIRE AVAILABLE");
    raw_bytes[0] = Wire.read();
    raw_bytes[1] = Wire.read();
    raw_bytes[2] = Wire.read();
    raw_bytes[3] = Wire.read();
  }

  uint32_t data = (raw_bytes[0] << 24) |
                  (raw_bytes[1] << 16) |
                  (raw_bytes[2] << 8) |
                  raw_bytes[3];

  if ((data >> STATUS_SHIFT))
  {
    // anything other then 00 in the status bits is an error
    // Serial.println("Status Error");
    // return (0);
  }

  uint32_t press_raw = (data >> PRESSURE_SHIFT) & PRESSURE_MASK;
  uint32_t temp_raw = (data >> TEMPERATURE_SHIFT) & TEMPERATURE_MASK;

  // running average
  total = total - readings[readIndex];
  readings[readIndex] = press_raw;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings)
  {
    readIndex = 0;
  }
  average = total / numReadings;

  press_inH2O_raw = 1.25f * 2.0f * FSS_inH2O * ((average - DLVR_OFFSET) / DLVR_SCALE); // pressure (inH2O) output transfer function

  press_inH2O = press_inH2O_raw - 0.002; // offset for static calibration
  if (press_inH2O < 0.0)
    press_inH2O = 0.0;

  temp_float = temp_raw * (200.0f / 2047.0f) - 50.0f;

  airSpeed_ms = (sqrt(2 * (press_inH2O * 249.1 / 1.225))); // airspeed m/s  (1 inH2O = 249.1 Pa)
  airSpeed_mph = airSpeed_ms * 2.237;
  // Serial.print(press_inH2O_raw,4);
  // Serial.print("\t");
  // Serial.print(press_inH2O, 4);
  // Serial.print("\t");
  // Serial.print(airSpeed_ms);
  // Serial.print("\t");
  // Serial.print(airSpeed_mph);
  // Serial.print("\t");
  // Serial.println(millis());

  OAT = int(temp_float);
}
//****************END GET AIRSPEED*****************

//*****************DISPLAY AIRSPEED**********************
void display_airSpeed()
{
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(Arial_72_Bold);
  // tft.setFont(DSEG7_bold_22);
  if (airSpeed_mph >= 0 && airSpeed_mph <= 9)
  {
    tft.setCursor(airSpeed_x0, airSpeed_y0);
  }
  if (airSpeed_mph >= 10 && airSpeed_mph <= 99)
  {
    tft.setCursor(airSpeed_x0 - font_width_72, airSpeed_y0);
  }
  if (airSpeed_mph < 10 && old_airSpeed_mph >= 10)
  {
    tft.setCursor(airSpeed_x0 - font_width_72, airSpeed_y0);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(0, 0, 110, 90);
    tft.print(1);
  }
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  if (airSpeed_mph != old_airSpeed_mph)
  {
    //  Serial.println(airSpeed);
    tft.setClipRect(3, 0, 110, 90);
    tft.print(airSpeed_mph);
    old_airSpeed_mph = airSpeed_mph;
  }
}
//****************END DISPLAY AIRSPEED*****************

//*****************DISPLAY GROUNDSPEED**********************
void display_groundSpeed()
{
  tft.setFont(Arial_32_Bold);
  // tft.setFont(DSEG7_bold_22);
  if (groundSpeed >= 0 && groundSpeed <= 9)
  {
    tft.setCursor(groundSpeed_x0, groundSpeed_y0);
    //  tft.setCursor(groundSpeed_x0, groundSpeed_y0, true);
  }
  if (groundSpeed >= 10 && groundSpeed <= 99)
  {
    tft.setCursor(groundSpeed_x0 - font_width_32, groundSpeed_y0);
  }
  if (groundSpeed < 10 && old_groundSpeed >= 10)
  {
    tft.setCursor(groundSpeed_x0 - font_width_32, groundSpeed_y0);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(184, 44, 134, 46);
    tft.print(1);
  }
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);

  if (groundSpeed != old_groundSpeed)
  {
    tft.setClipRect(184, 44, 134, 46);
    tft.print(groundSpeed);
    old_groundSpeed = groundSpeed;
  }
}
//****************END DISPLAY GROUNDSPEED*****************

//*****************DISPLAY TRACK**********************
void display_track()
{
  tft.setFont(Arial_24_Bold);
  tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
  // tft.setFont(DSEG7_bold_22);
  if (track >= 0 && track <= 9)
  {
    tft.setClipRect(8, 136, 100, 50);
    tft.setCursor(18, 140);
    tft.print(0);
    tft.setCursor(36, 140);
    tft.print(0);
    tft.setCursor(54, 140);
  }
  if (track >= 10 && track <= 99)
  {
    tft.setCursor(54 - font_width_24, 140);
  }
  if (track < 10 && old_track >= 10)
  {
    tft.setCursor(54 - font_width_24, 140);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(8, 126, 100, 50);
    tft.print(1);
    tft.setCursor(54 - font_width_24, 140);
    tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    tft.print(0);
  }
  if (track >= 100 && track <= 360)
  {
    tft.setCursor(18, 140);
  }
  if (track < 100 && old_track >= 100)
  {
    tft.setCursor(18, 140);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(8, 126, 100, 50);
    tft.print(1);
    tft.setCursor(18, 140);
    tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    tft.print(0);
  }
  tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
  if (track != old_track)
  {
    // Serial.println(track);
    // Serial.println("track");
    tft.setClipRect(8, 136, 100, 50);
    tft.print(track);
    old_track = track;
  }
}
//****************END DISPLAY TRACK*****************

//*****************DISPLAY GPS ALT**********************
void display_gpsAlt()
{
  tft.setFont(Arial_24_Bold);
  // tft.setFont(DSEG7_bold_22);
  if (gpsAlt >= 0 && gpsAlt <= 9)
  {
    tft.setCursor(166, 140);
  }
  if (gpsAlt >= 10 && gpsAlt <= 99)
  {
    tft.setCursor(166 - font_width_24, 140);
  }
  if (gpsAlt < 10 && old_gpsAlt >= 10)
  {
    tft.setCursor(166 - font_width_24, 140);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(107, 129, 100, 50);
    tft.print(1);
  }
  if (gpsAlt >= 100 && gpsAlt <= 999)
  {
    tft.setCursor(166 - 2*font_width_24, 140);
    // tft.print(track);
  }
  if (gpsAlt < 100 && old_gpsAlt >= 100)
  {
    tft.setCursor(166 - 2*font_width_24, 140);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(107, 129, 100, 50);
    tft.print(1);
    tft.setCursor(166 - font_width_24, 140);
    tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    //  tft.print(0);
  }
  if (gpsAlt >= 1000 && gpsAlt <= 9999)
  {
    tft.setCursor(112, 140); // leading zeros
    // tft.print(track);
  }
  if (gpsAlt < 1000 && old_gpsAlt >= 1000)
  {
    tft.setCursor(112, 140);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(107, 129, 100, 50);
    tft.print(1);
    tft.setCursor(130, 140);
    tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    //  tft.print(0);
  }

  /* test screen clear from 4 digits to 2 digits
  if (gpsAlt < 100 && old_gpsAlt >= 1000)
  {
    tft.setCursor(130, 140);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(122, 126, 100, 50);
    tft.print(1);
    tft.setCursor(130, 140);
    tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
    //  tft.print(0);
  }
*/

  tft.setTextColor(ILI9341_MAGENTA, ILI9341_BLACK);
  if (gpsAlt != old_gpsAlt)
  {
    //  Serial.println(gpsAlt);
    //  Serial.println("gpsAlt");
    tft.setClipRect(109, 129, 100, 50);
    tft.print(gpsAlt);
    old_gpsAlt = gpsAlt;
  }
}
//****************END DISPLAY GPS ALT*****************

//*****************DISPLAY OAT (Outside Air Temp)**********************
void display_OAT()
{

  tft.setFont(Arial_24_Bold);
  // tft.setFont(DSEG7_bold_22);
  if (OAT >= 0 && OAT <= 9)
  {
    tft.setCursor(250, 140);
  }
  if (OAT >= 10 && OAT <= 99)
  {
    tft.setCursor(250 - font_width_24, 140);
  }
  if (OAT < 10 && old_OAT >= 10)
  {
    tft.setCursor(250 - font_width_24, 140);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(230, 126, 100, 50);
    tft.print(1);
  }
  tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);

  if (OAT != old_OAT)
  {
    tft.setClipRect(230, 126, 100, 50);
    tft.print(OAT);
    old_OAT = OAT;
  }
}
//****************END DISPLAY OAT (Outside Air Temp)*****************

//*****************DISPLAY GPS Status**********************
void display_gpsStatus_backup()
{
  tft.setFont(Arial_16_Bold);

  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  if (satellites >= 0 && satellites <= 9)
  {
    tft.setCursor(56, 213);
  }
  if (satellites >= 10 && satellites <= 99)
  {
    tft.setCursor(56 - font_width_14, 213);
  }
  if (satellites < 10 && old_satellites >= 10)
  {
    tft.setCursor(56 - font_width_16, 213);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(56, 213, 40, 30);
    tft.print(1);
  }
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);

  if (satellites != old_satellites)
  {
    tft.setClipRect(56, 213, 40, 30);
    tft.print(satellites);
    old_satellites = satellites;
  }
}
//****************END DISPLAY GPS Status*****************


//*****************DISPLAY GPS Status**********************
void display_gpsStatus()
{
  tft.setFont(Arial_16_Bold);
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  tft.setCursor(56, 213);

  if (satellites != old_satellites)
  {
    if (satellites < 10 && old_satellites >= 10)
    {
    tft.setCursor(56, 213);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.setClipRect(56, 213, 40, 30);
    tft.print(old_satellites);
    tft.setCursor(56, 213);
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.print(satellites);
    }
    else {
    tft.setClipRect(56, 213, 40, 30);
    tft.print(satellites);
    
    }
    old_satellites = satellites;
  }
}
//****************END DISPLAY GPS Status*******************

//*****************DISPLAY Time**********************
void display_time() {
  tft.setFont(Arial_16_Bold); 
  if (seconds != old_seconds) {
    if (seconds <= 9) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(230+56, 188);
      tft.print("0");
      tft.setCursor(230+56 + 13, 188);
      tft.setClipRect(110+56, 188, 190, 30);
      tft.print(seconds);
    }
    else {
      tft.setFont(Arial_16_Bold);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setClipRect(110+56, 188, 190, 30);
      tft.setCursor(230+56, 188);
      tft.print(seconds);
    }
  }
  if (minutes != old_minutes) {
    if (minutes <= 9) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(201+56, 188);
      tft.print("0");
      tft.setCursor(201+56 + 13, 188);
      tft.setClipRect(110+56, 188, 190, 30);
      tft.print(minutes);
    }
    else {
      tft.setFont(Arial_16_Bold);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setClipRect(110+56, 188, 190, 30);
      tft.setCursor(201+56, 188);
      tft.print(minutes);
    }
  }
  if (hours != old_hours) {
    if (hours >= 0 && hours <= 9) { // Convert UTC to HST (24 hour)
      hours = hours + 14;
    }
    else {
      hours = hours - 10;
    }
    if (hours <= 9) {
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setCursor(172+56, 188);
      tft.print("0");
      tft.setCursor(172+56 + 13, 188);
      tft.setClipRect(110+56, 188, 190, 30);
      tft.print(hours);

    }
    else {
      tft.setFont(Arial_16_Bold);
      tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
      tft.setClipRect(110+56, 188, 190, 30);
      tft.setCursor(172+56, 188);
      tft.print(hours);
    }
  }

}
//*****************END DISPLAY Time******************


//*****************READ ENCODER****************************
void read_encoder()
{
  encSpeed = rotaryEnc.read() / 4;
  encSpeed = encSpeed * INC;

  if (encSpeed < 0)
  {
    encSpeed = 0;
    rotaryEnc.write(0);
  }
  if (encSpeed > 360)
  {
    encSpeed = 360;
    rotaryEnc.write(360 / INC * 4);
  }
  if (encSpeed != old_encSpeed)
  {
    old_encSpeed = encSpeed;
  }
  //  airSpeed = encSpeed;
  //  groundSpeed = encSpeed*0.9;
  //  track = 4 * encSpeed;
  // gpsAlt = 1999 * encSpeed;
  // if (gpsAlt > 9999)
  //  gpsAlt = 9999;
  //  OAT = encSpeed/2;
}
//****************END READ ENCODER*****************

//*****************SIMULATE AIRSPEED**********************
void sim_airSpeed()
{
  if (millis() - timer_display_airSpeed > delay_display_airSpeed)
  {
    display_airSpeed();
    airSpeed = airSpeed + delta * 3;
    if (airSpeed <= 0 || airSpeed >= 90)
    {
      delta = -delta;
    }
    timer_display_airSpeed = millis();
  }
}
//****************END DISPLAY AIRSPEED********************


//*****************DISPLAY X**********************
void display_X(int color)
{
  // Serial.println("display_X()");
  if (color == 1)   // No GPS Fix
  { 
  // Ground Speed X-out
    tft.setClipRect(184, 44, 134, 46);
    tft.drawLine(190, 45, 310, 88, ILI9341_RED);
    tft.drawLine(190, 46, 310, 89, ILI9341_RED);
    tft.drawLine(190, 48, 308, 90, ILI9341_RED);
    tft.drawLine(190, 88, 310, 46, ILI9341_RED);
    tft.drawLine(190, 87, 310, 45, ILI9341_RED);
    tft.drawLine(190, 86, 308, 44, ILI9341_RED);
  // Track X-out
    tft.setClipRect(8, 126, 100, 50);
    tft.drawLine(10, 128, 100, 174, ILI9341_RED);
    tft.drawLine(10, 129, 100, 175, ILI9341_RED);
    tft.drawLine(10, 130, 100, 176, ILI9341_RED);
    tft.drawLine(10, 177, 100, 130, ILI9341_RED);
    tft.drawLine(10, 176, 100, 129, ILI9341_RED);
    tft.drawLine(10, 175, 100, 128, ILI9341_RED);
  // GPS Altitude X-out
    tft.setClipRect(110, 126, 100, 50);
    tft.drawLine(109, 128, 208, 174, ILI9341_RED);
    tft.drawLine(109, 129, 208, 175, ILI9341_RED);
    tft.drawLine(109, 130, 208, 176, ILI9341_RED);
    tft.drawLine(109, 177, 208, 130, ILI9341_RED);
    tft.drawLine(109, 176, 208, 129, ILI9341_RED);
    tft.drawLine(109, 175, 208, 128, ILI9341_RED);

    tft.setClipRect(50, 188, 120, 40);
    tft.setFont(Arial_16_Bold);
    tft.setCursor(56, 188);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.print("Fix");
    tft.setCursor(56, 188);
    tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
    tft.print("No Fix");

  }

  if (color == 0)   // GPS Fix
  {
  // Ground Speed
    tft.setClipRect(184, 44, 134, 46);
    tft.drawLine(190, 45, 310, 88, ILI9341_BLACK);
    tft.drawLine(190, 46, 310, 89, ILI9341_BLACK);
    tft.drawLine(190, 48, 308, 90, ILI9341_BLACK);
    tft.drawLine(190, 88, 310, 46, ILI9341_BLACK);
    tft.drawLine(190, 87, 310, 45, ILI9341_BLACK);
    tft.drawLine(190, 86, 308, 44, ILI9341_BLACK);
    tft.setFont(Arial_16);
    tft.setCursor(groundSpeed_x0 + 40, groundSpeed_y0 + 8);
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.print("mph");

  // Track
    tft.setClipRect(8, 126, 100, 50);
    tft.drawLine(10, 128, 100, 174, ILI9341_BLACK);
    tft.drawLine(10, 129, 100, 175, ILI9341_BLACK);
    tft.drawLine(10, 130, 100, 176, ILI9341_BLACK);
    tft.drawLine(10, 177, 100, 130, ILI9341_BLACK);
    tft.drawLine(10, 176, 100, 129, ILI9341_BLACK);
    tft.drawLine(10, 175, 100, 128, ILI9341_BLACK);
    tft.drawCircle(80, 145, 4, ILI9341_WHITE); // track degree symbol
    tft.drawCircle(80, 145, 5, ILI9341_WHITE);
    display_track(); //update to overwrite black X

  // GPS Altitude
    tft.setClipRect(110, 126, 100, 50);
    tft.drawLine(109, 128, 208, 174, ILI9341_BLACK);
    tft.drawLine(109, 129, 208, 175, ILI9341_BLACK);
    tft.drawLine(109, 130, 208, 176, ILI9341_BLACK);
    tft.drawLine(109, 177, 208, 130, ILI9341_BLACK);
    tft.drawLine(109, 176, 208, 129, ILI9341_BLACK);
    tft.drawLine(109, 175, 208, 128, ILI9341_BLACK);
    display_gpsAlt(); //update to overwrite black X

  // Fix-NoFix
    tft.setClipRect(50, 188, 120, 40);
    tft.setFont(Arial_16_Bold);
    tft.setCursor(56, 188);
    tft.setTextColor(ILI9341_BLACK, ILI9341_BLACK);
    tft.print("No Fix");
    tft.setCursor(56, 188);
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.print("Fix");
  }
}
//****************END DISPLAY X********************

//*****************BOOT SCREEN**********************
void boot_screen()
{
}
//****************END BOOT SCREEN********************

//*****************SETUP**********************
void setup()
{
  Serial.begin(115200);
  gpsPort.begin(38400);
  Wire.begin();
  pinMode(SW_PIN, INPUT_PULLUP); //  can be just INPUT
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);

  boot_screen();

  // Static display items
  tft.setFont(Arial_8);
  tft.setCursor(260, 218);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("v1.0(L05D)");
  
  // airspeed
  tft_width = tft.width();
  tft_height = tft.height();

  tft.drawRect(2, 6, 180, 86, ILI9341_WHITE);
  tft.setFont(Arial_20_Bold);
  tft.setCursor(120, 20);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("IAS");
  tft.setFont(Arial_18_Bold);
  tft.setCursor(120, 50);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("mph");

  // ground speed
  tft.drawRect(186, 6, 130, 38, ILI9341_WHITE);
  tft.drawRect(186, 42, 130, 50, ILI9341_WHITE);
  tft.fillRect(187, 7, 128, 36, DARK_BLUE_1);

  tft.setFont(Arial_16);
  tft.setCursor(194, 14);
  tft.setTextColor(ILI9341_WHITE, DARK_BLUE_1);
  tft.print("Gnd Speed");

  tft.setCursor(groundSpeed_x0 + 40, groundSpeed_y0 + 8);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("mph");

  tft.fillRect(7, 101, 306, 26, DARK_GREY_2);
  tft.setFont(Arial_14);
  tft.setCursor(20, 105);
  tft.setTextColor(ILI9341_WHITE, DARK_GREY_2);
  tft.print("TRACK");
  tft.drawCircle(80, 145, 4, ILI9341_WHITE); // track degree symbol
  tft.drawCircle(80, 145, 5, ILI9341_WHITE);
  tft.setCursor(112, 105);
  tft.print("ALT (GPS)");
  tft.setCursor(240, 105);
  tft.print("OAT");
  tft.drawCircle(276, 143, 4, ILI9341_WHITE); // temperature degree symbol
  tft.drawCircle(276, 143, 5, ILI9341_WHITE);
  tft.setCursor(285, 140);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(Arial_16_Bold);
  tft.print("C");
  tft.setCursor(190, 145);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setFont(Arial_16_Bold);
  tft.print("ft");

  // bottom display boxes
  tft.drawRect(6, 100, 306, 80, ILI9341_WHITE);
  tft.drawLine(6, 127, 310, 127, ILI9341_WHITE);
  tft.drawLine(105, 100, 105, 178, ILI9341_WHITE);
  tft.drawLine(210, 100, 210, 178, ILI9341_WHITE);
/*
  // GPS Status Bars
  tft.drawRect(260, 230, 8, 10, ILI9341_RED);
  tft.drawRect(272, 222, 8, 18, ILI9341_RED);
  tft.drawRect(284, 216, 8, 24, ILI9341_RED);
  tft.drawRect(296, 208, 8, 32, ILI9341_RED);
  //  tft.drawRect(220, 200, 80, 30, ILI9341_WHITE);
  // tft.fillRect(187, 7, 128, 36, DARK_BLUE_1);
*/
  tft.setFont(Arial_16);
  tft.setCursor(4, 188);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("GPS:");

  tft.setFont(Arial_16);
  tft.setCursor(4, 212);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.print("Sats:");

  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); // colons for time display
  tft.setCursor(225+56, 188);
  tft.print(":");
  tft.setCursor(195+56, 188);
  tft.print(":");

// Temporary code to indicate "Fix" if present at power-up
// Improve by using flags
      tft.setFont(Arial_16_Bold);
      tft.setCursor(56, 188);
      tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      //tft.setClipRect(50, 188, 120, 40);
      tft.print("Fix");
      Serial.println ("testFix");
    
  
}
//*****************END SETUP**********************

//*****************MAIN LOOP**********************
void loop(void)
{
/*  
  while (!gpsPort.available())
  {
    //if (portFlag == false)
    //  {
    //    Serial.println("gps port not available");
    //    portFlag = !portFlag; 
    //  }
  }
*/  

  
  while (gps.available(gpsPort))
  {
    fix = gps.read();

  //  Serial.println(F("Location: "));
    if (fix.valid.location)
    {
      gpsFix = true;
      //Serial.println("gpsFix");
      //Serial.println(fix.valid.location);
      //Serial.print(fix.latitude(), 6);
      //Serial.print(',');
      //Serial.print(fix.longitude(), 6);
      if (fixFlag == true)
      {
        display_X(0);
        fixFlag = !fixFlag;
      }
    }
    else
    {
      gpsFix = false;
    //  Serial.print(fixFlag);
    //  Serial.println("  GPSfix = false");
      if (fixFlag == false)
      {
        display_X(1);
        groundSpeed = 0;
        fixFlag = !fixFlag;
        Serial.println("  GPSfix = false");
      }
    }

    
    if (fix.valid.time)
    {
        hours = fix.dateTime.hours;
        minutes = fix.dateTime.minutes;
        seconds = fix.dateTime.seconds;

        display_time();
        Serial.print( fix.dateTime.hours );
        Serial.print(":");
        Serial.print( fix.dateTime.minutes );
        Serial.print(":");
        Serial.println( fix.dateTime.seconds );
      
    }

    // Serial.print( F(", Speed: ") );
    if (fix.valid.speed)
    {
      //  Serial.print( fix.speed_mph() );
      groundSpeed = fix.speed_mph();
    }
    // Serial.print( F(", Heading: ") );
    if (fix.valid.heading)
    {
      //  Serial.print( fix.heading() );
      track = fix.heading();
    }
    //  Serial.print( F(", Satellites: ") );
    if (fix.valid.satellites)
    {
      //  Serial.print( fix.satellites );
      satellites = fix.satellites;
    }
    //  Serial.print( F(", Altitude: ") );
    if (fix.valid.altitude)
    {
      //  Serial.print( fix.altitude() );
      gpsAlt = fix.altitude() * 3.28; // meter to feet conversion factor
      if (gpsAlt >= 9999)
        gpsAlt = 9999;
      if (gpsAlt <= 0)
        gpsAlt = 0;  
    }
    //  Serial.println();
  }

  read_encoder();
  //  groundspeed = int(gps.speed.kmph()*0.621);
  // sim_airSpeed();
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    get_airspeed();
    previousMillis = currentMillis;
  }

  display_airSpeed();

  if (gpsFix == true)
  {
    display_groundSpeed();
    display_track();
    display_gpsAlt();
  }
  
  display_gpsStatus();
  display_OAT();
}
//*****************END MAIN LOOP**********************
