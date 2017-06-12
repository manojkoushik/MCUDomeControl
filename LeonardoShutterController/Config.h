#include <MsTimer2.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <EEPROM.h>
#include <avr/power.h>
#include <avr/sleep.h>


//                +----[PWR]------------------| μUSB |--+
//                |                           +------+  |
//                |         GND/RST2  [ ][ ]            |
//                |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |   C5 
//                |          5V/MISO2 [ ][ ]  A4/SDA[ ] |   C4 
//                |                             AREF[ ] |
//                |                              GND[ ] |
//                | [ ]N/C                    SCK/13[ ] |   B5  
//                | [ ]IOREF                 MISO/12[ ] |   .   
//                | [ ]RST                   MOSI/11[ ]~|   .   LS Close
//                | [ ]3V3    +----+              10[ ]~|   .   LS Open
//                | [ ]5v    -| A L|-              9[ ]~|   .   US Close
//                | [ ]GND   -| R E|-              8[ ] |   B0  US Open
//                | [ ]GND   -| D O|-                   |
//                | [ ]Vin   -| U N|-              7[ ] |   D7  LS DIR
//                |          -| I A|-              6[ ]~|   .   LS PWM
//Batt Volt       | [ ]A0    -| N R|-              5[ ]~|   .   US PWM
//                | [ ]A1    -| O D|-              4[ ] |   .   US DIR
//                | [ ]A2    -|   O|-         INT1/3[ ]~|   .   LS303 SCL
//                | [ ]A3     +----+          INT0/2[ ] |   .   LS303 SDA
//                | [ ]A4      RST SCK MISO     TX>1[ ] |   .   XBee RX
//                | [ ]A5      [ ] [ ] [ ]      RX<0[ ] |   D0  Xbee TX
//                |            [ ] [ ] [ ]              |
//                |  UNO_R3    GND MOSI 5V  ____________/
//                 \_______________________/

#define SERIALSPEED 57600
#define MYSERIAL Serial1
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define WATCHDOG_TIMEOUT 60000 // Watchdog times out in 1 minute
                               // after which we go to sleep till there's activity again

//index into motorconf
#define UPPERSHUTTER 0
#define LOWERSHUTTER 1

// Voltage divide to 5
// Measure voltage without load using multimeter
// Read analog value
// Const = Voltage/Analog Value
#define BATTVOLTCONST 0.015
#define BATTVOLTPIN A0

struct EepromVars {
  // Update version anytime you want to rewrite eeprom values after a flash
  // all vars have their own set functions, so you could change anything after flash as well
  byte                      VERSION =                   2;
  float                     AZOFFSET =                0.0;
  int                       VENTOPENTIME =          10000; //10 seconds in millis
  float                     BATTVOLTALERT =           9.5;
  byte                      OPENDIRECTION[2] = {LOW, LOW};
  adafruit_bno055_offsets_t CALIBDATA;
  bool                      CALIB =                 false;
};

enum shutterstate {
  shutterOpen = 0,
  shutterClosed,
  shutterOpening,
  shutterClosing,
  shutterError,
  shutterVent
};

struct motorConf {
  int duty[2] =            {255,255}; // duty cycle 0 to 255. 255 is 100%
  int pwmPin[2] =          {6, 5};
  int dirPin[2] =          {7, 4};
  int openSwitchPin[2] =   {8, 10};
  int closedSwitchPin[2] = {9, 11};
};

// Watchdog
bool          watchDogExpired = false;
unsigned long watchDogTimer =       0;

float domeAzimuth;
float battVoltage;
Adafruit_BNO055 compass = Adafruit_BNO055(55);
motorConf motors;
EepromVars eepromVars;
shutterstate shutterState[2] = {shutterOpen, shutterOpen};
