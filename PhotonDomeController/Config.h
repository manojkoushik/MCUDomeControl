#include <Wire.h> // Used to establied serial communication on the I2C bus
#include "SparkFunTMP102.h" // Used to send and recieve specific information from our sensor
                            // Get it from https://github.com/sparkfun/SparkFun_TMP102_Arduino_Library
#include <math.h>

//                                               +___+
//                                   +___________|USB|___________+
//                                   |           +---+           |
//                                   |                           |
//                                   |                           |
//                                   | [ ] VIN           3V3 [ ] |
//                                   | [ ] GND           RST [ ] |
//   XBee          (Shield PIN 1TX)  | [ ] TX          VBATT [ ] |
//   XBee          (Shield PIN 0RX)  | [ ] RX            GND [ ] |
//                 (Shield PIN 3)    | [ ] WKP/A7   [LED] D7 [ ] | Key Fob D (Open Shutter)  (Shield PIN 7)
//                 (Shield PIN A2)   | [ ] DAC/A6         D6 [ ] | Key Fob C (Clockwise)     (Shield PIN 4)
//                 (Shield PIN 8)    | [ ] A5  +______+   D5 [ ] | Key Fob B (Close Shutter) (Shield PIN 10)
//                 (Shield PIN 9)    | [ ] A4  |Photon|   D4 [ ] | Key Fob A (AntiClockwise) (Shield PIN 13)
//                 (Shield PIN A3)   | [ ] A3  +______+   D3 [ ] | Motor DIR                 (Shield PIN 12)
//                 (Shield PIN 2)    | [ ] A2             D2 [ ] | Motor PWM                 (Shield PIN 11)
//                 (Shield PIN A1)   | [ ] A1       [SCL] D1 [ ] | TMP102 SCL                (Shield PIN 6/A5)
//                 (Shield PIN A0)   | [ ] A0       [SDA] D0 [ ] | TMP102 SDA                (Shield PIN 5/A4)
//                                   |                           |
//                                   |                           |
//                                   \                           /
//                                    \                         /
//                                     \_______________________/


#define SERIALSPEED 57600
#define MYSERIAL  Serial1

//index into motorconf
#define UPPERSHUTTER 0
#define LOWERSHUTTER 1

// Timers for when to read sensors
//#define AZREAD 5000
#define AZREAD             100
#define AZREADPASSIVE      10000
#define SHUTTERREAD        2000
#define SHUTTERREADPASSIVE 60000
#define TEMPREAD           3000
#define BATTREAD           300000

#define ANTICLOCKWISEPIN 4
#define CLOCKWISEPIN     6
#define OSPIN            7
#define CSPIN            5
#define TMP36PIN        A0

#define SLEWTOLERANCE 0.25
#define SLOWSLEW         2

#define WATCHDOG_TIMEOUT 300000 // Watchdog times out after 1 minute

struct EepromVars {
  byte  VERSION =         1;
  float PARKAZ =      180.0;
  byte  CLOCKWISE =    HIGH;
  float TEMPOFFSET =    0.0;
  bool  AUTOVENT =     true;
  bool  AUTOAC =       true;
  float VENTON =       78.0;
  float VENTOFF =      74.0;
  float VENTOUTSIDE =  72.0;
  float ACON =         82.0;
  float ACOFF =        74.0;
  float BATTVOLTALERT = 9.5;
  bool  AUTOBATT     = true;
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
  // duty cycle 0 to 255. 255 is 100%
  int duty = 200;
  int pwmPin = 2;
  int dirPin = 3;
};

// Flags to shutter thread
bool openShutter =  false;
bool closeShutter = false;
bool ventShutter = false;

// Flags to slewing thread
bool slewActive = false;
bool parked =     false;

// Flags to batt thread
bool battCmd = false;

// Batt voltage. Automatic batt monitoring will close shutters if batt voltage
// gets too low
float battVoltage = 15.0;

// Set motor states using this flag when slew is active
bool motorMoving = false;

// Az to which we need to move
float commandAz = 0;
// Current Az
float currentAz = 0;
// Current Shutter States
shutterstate upperShutter, lowerShutter, shutter;

// Observatory inside temperature. Used for automatic venting and AC turn on
float obsTemp = 0;

// Watchdog vars
unsigned long watchDogTimer = 0;
bool watchDogExpired = false;

// commands to Weather Station
bool wsEnPub =      false;
bool wsDisPub =     false;
bool wsEnFlash =    false;
bool wsDisFlash =   false;
bool wsRainCmd =    false;
bool rainDetect =   false;
bool wsTempCmd =    false;
float outsideTemp = false;

// auto vent/ac status
bool ventOn = false;
bool acOn =   false;

EepromVars eepromVars;
motorConf motor;
TMP102 tempSensor(0x48);
