#include "Config.h"

/******************************************************************************************************/
// Watchdog
/******************************************************************************************************/

void watchDog(bool punch) {
  unsigned long now = millis();
  
  if (punch) {
    watchDogTimer = now;
    watchDogExpired = false;
    digitalWrite(LED_BUILTIN, HIGH);
    return;
  } 

  if (now - watchDogTimer >= WATCHDOG_TIMEOUT) {
    watchDogExpired = true;
    digitalWrite(LED_BUILTIN, LOW);
    
    set_sleep_mode(SLEEP_MODE_IDLE);

    sleep_enable();

    power_adc_disable();
    power_timer0_disable();
    power_timer2_disable();
    power_twi_disable();
    
    sleep_mode();
    
    sleep_disable();
    power_adc_enable();
    power_twi_enable();
    power_timer0_enable();
    power_timer2_enable();
  }
}

/******************************************************************************************************/
// BNO055 IMU Routines
/******************************************************************************************************/

void compassCalib() {
  sensors_event_t event;

  if (!eepromVars.CALIB) {
    while (!compass.isFullyCalibrated())
    {
        compass.getEvent(&event);
  
        MYSERIAL.print("X: ");
        MYSERIAL.print(event.orientation.x, 4);
        MYSERIAL.print("\tY: ");
        MYSERIAL.print(event.orientation.y, 4);
        MYSERIAL.print("\tZ: ");
        MYSERIAL.print(event.orientation.z, 4);
        uint8_t sys, gyro, accel, mag;
        sys = gyro = accel = mag = 0;
        compass.getCalibration(&sys, &gyro, &accel, &mag);

        /* The data should be ignored until the system calibration is > 0 */
        MYSERIAL.print("\t");
        if (!sys)
        {
            MYSERIAL.print("! ");
        }
    
        /* Display the individual values */
        MYSERIAL.print("Sys:");
        MYSERIAL.print(sys, DEC);
        MYSERIAL.print(" G:");
        MYSERIAL.print(gyro, DEC);
        MYSERIAL.print(" A:");
        MYSERIAL.print(accel, DEC);
        MYSERIAL.print(" M:");
        MYSERIAL.print(mag, DEC);
    
            /* New line for the next sample */
        MYSERIAL.println("");
  
        /* Wait the specified delay before requesting new data */
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }
    
    MYSERIAL.println(F("Fully calibrated. Storing calibration data..."));
    compass.getSensorOffsets(eepromVars.CALIBDATA);
    eepromVars.CALIB = true;
    EEPROM.put(0, eepromVars);    
  } else {
    compass.setSensorOffsets(eepromVars.CALIBDATA);
  }
}

void compassSetup() {
  if(!compass.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    MYSERIAL.print(F("No BNO055 detected ..."));
    while(1);
  }

  compass.setExtCrystalUse(true);

  delay(1000);

  // Force a calibration run on first use. Nothing else works without this
  compassCalib();
}

void compassHeading() {
  unsigned long now = millis();
  static unsigned long lastRead = 0;

  if (now - lastRead >= BNO055_SAMPLERATE_DELAY_MS) {
    lastRead = now;
    /* Get a new sensor event */
    sensors_event_t event;
    compass.getEvent(&event);

    domeAzimuth = (event.orientation.x*1.0) + (eepromVars.AZOFFSET*1.0);
    if (domeAzimuth < 0) domeAzimuth += 360.0;
    if (domeAzimuth >= 360) domeAzimuth -= 360.0;
  }
}

void setAzOffset(float realAz) {
  // wait for latest reading
  delay(200);

  //reset offset and get new reading
  eepromVars.AZOFFSET = 0;
  delay(200);
  compassHeading();

  // set new offset and update heading
  eepromVars.AZOFFSET = (realAz*1.0) - (domeAzimuth*1.0);
  delay(200);
  compassHeading();  
  EEPROM.put(0, eepromVars);
}

/******************************************************************************************************/
// motor Routines
/******************************************************************************************************/

void reverseMotorDir(int shutter) {
  eepromVars.OPENDIRECTION[shutter] = !eepromVars.OPENDIRECTION[shutter];
  EEPROM.put(0, eepromVars);      
}

void setVentOpenTime(int ventOpenTime) {
  eepromVars.VENTOPENTIME = ventOpenTime;
  EEPROM.put(0, eepromVars);      
}

// Get shutter states
shutterstate getShutterState() {
  bool USOpen = digitalRead(motors.openSwitchPin[UPPERSHUTTER]);
  bool USClosed = digitalRead(motors.closedSwitchPin[UPPERSHUTTER]);
  bool LSOpen = digitalRead(motors.openSwitchPin[LOWERSHUTTER]);
  bool LSClosed = digitalRead(motors.closedSwitchPin[LOWERSHUTTER]);
  bool USMotorRunning = digitalRead(motors.pwmPin[UPPERSHUTTER]);
  byte USMotorDir = digitalRead(motors.dirPin[UPPERSHUTTER]);
  bool LSMotorRunning = digitalRead(motors.pwmPin[LOWERSHUTTER]);
  byte LSMotorDir = digitalRead(motors.dirPin[LOWERSHUTTER]);

  if (USMotorRunning) {
    if (USMotorDir == eepromVars.OPENDIRECTION[UPPERSHUTTER]) shutterState[UPPERSHUTTER] = shutterOpening;
    else shutterState[UPPERSHUTTER] = shutterClosing;
  } else {
    if (USOpen && !USClosed) shutterState[UPPERSHUTTER] = shutterOpen;
    else if (USClosed && !USOpen) shutterState[UPPERSHUTTER] = shutterClosed;
    else if (USOpen && USClosed) shutterState[UPPERSHUTTER] = shutterError;
    else shutterState[UPPERSHUTTER] = shutterVent;
  }

  if (LSMotorRunning) {
    if (LSMotorDir == eepromVars.OPENDIRECTION[LOWERSHUTTER]) shutterState[LOWERSHUTTER] = shutterOpening;
    else shutterState[LOWERSHUTTER] = shutterClosing;
  } else {
    if (LSOpen && !LSClosed) shutterState[LOWERSHUTTER] = shutterOpen;
    else if (LSClosed && !LSOpen) shutterState[LOWERSHUTTER] = shutterClosed;
    else shutterState[LOWERSHUTTER] = shutterError;
  }
}

void openShutter(int shutter) {
  // shutter is already open. Call stopMotor to ack
  if (shutterState[shutter] == shutterOpen) {
    stopMotor(shutter);
    return;
  }

  if (shutterState[UPPERSHUTTER] == shutterOpening ||
    shutterState[UPPERSHUTTER] == shutterClosing ||
    shutterState[LOWERSHUTTER] == shutterOpening ||
    shutterState[LOWERSHUTTER] == shutterClosing) {
    switch(shutter) {
      case UPPERSHUTTER:
        MYSERIAL.println("DSR:OUS:ERR");
        break;
      case LOWERSHUTTER:
        MYSERIAL.println("DSR:OLS:ERR");
        break;
    }
    return;
  }
  
  digitalWrite(motors.dirPin[shutter], eepromVars.OPENDIRECTION[shutter]);
  analogWrite(motors.pwmPin[shutter], motors.duty[shutter]);
}

void closeShutter(int shutter) {
  // shutter is already closed. Call stopMotor to ack
  if (shutterState[shutter] == shutterClosed) {
    stopMotor(shutter);
    return;
  }

  if (shutterState[UPPERSHUTTER] == shutterOpening ||
    shutterState[UPPERSHUTTER] == shutterClosing ||
    shutterState[LOWERSHUTTER] == shutterOpening ||
    shutterState[LOWERSHUTTER] == shutterClosing) {
    switch(shutter) {
      case UPPERSHUTTER:
        MYSERIAL.println("DSR:CUS:ERR");
        break;
      case LOWERSHUTTER:
        MYSERIAL.println("DSR:CLS:ERR");
        break;
    }
    return;
  }

  digitalWrite(motors.dirPin[shutter], !eepromVars.OPENDIRECTION[shutter]);
  analogWrite(motors.pwmPin[shutter], motors.duty[shutter]);
}

void ventShutter() {
  // shutter is already venting turn off flag and return
  if (shutterState[UPPERSHUTTER] == shutterVent) {
    stopMotor(UPPERSHUTTER);
    return;
  }
  
  // Only operate upper vent if all shutters are closed
  if (shutterState[UPPERSHUTTER] == shutterClosed &&
    shutterState[LOWERSHUTTER] == shutterClosed ) {
    digitalWrite(motors.dirPin[UPPERSHUTTER], eepromVars.OPENDIRECTION[UPPERSHUTTER]);
    analogWrite(motors.pwmPin[UPPERSHUTTER], motors.duty[UPPERSHUTTER]);
    MsTimer2::start();
  } else {
    MYSERIAL.println("DSR:VS:ERR");    
  }
}

void stopVentOpen() {
  MsTimer2::stop();
  stopMotor(UPPERSHUTTER);  
}

void stopMotor(int shutter) {
  byte motorDir = digitalRead(motors.dirPin[shutter]);
  
  analogWrite(motors.pwmPin[shutter], LOW);
  getShutterState();

  switch(shutter) {
    case UPPERSHUTTER:
      switch (shutterState[shutter]) {
        case shutterOpen:
          MYSERIAL.println("DSR:OUS:OK");
          break;
        case shutterClosed:
          MYSERIAL.println("DSR:CUS:OK");
          break;
        case shutterVent:
          MYSERIAL.println("DSR:VS:OK");
          break;        
      }
      break;
    case LOWERSHUTTER:
      switch (shutterState[shutter]) {
        case shutterOpen:
          MYSERIAL.println("DSR:OLS:OK");
          break;
        case shutterClosed:
          MYSERIAL.println("DSR:CLS:OK");
          break;
      }
      break;
  }
}

void checkMotors() {
  static byte USOS = 0xff;
  static byte USCS = 0xff;
  static byte LSOS = 0xff;
  static byte LSCS = 0xff;

  // switch debounce
  delay(10);
  
  // Static variables uninitialized. First run
  if (USOS == 0xff || USCS == 0xff || LSOS == 0xff || LSCS == 0xff) {
    USOS = digitalRead(motors.openSwitchPin[UPPERSHUTTER]);
    USCS = digitalRead(motors.closedSwitchPin[UPPERSHUTTER]);
    LSOS = digitalRead(motors.openSwitchPin[LOWERSHUTTER]);
    LSCS = digitalRead(motors.closedSwitchPin[LOWERSHUTTER]);
    // first run, might be after an unexpected reset
    // stop all motors
    stopMotor(UPPERSHUTTER);
    stopMotor(LOWERSHUTTER);
    return;
  }

  // stop motors anytime one of the limit switches are triggered  
  if ((USOS == 0 && digitalRead(motors.openSwitchPin[UPPERSHUTTER]) == 1) ||
      (USCS == 0 && digitalRead(motors.closedSwitchPin[UPPERSHUTTER]) == 1)) {
    stopMotor(UPPERSHUTTER);
  }
  
  if ((LSOS == 0 && digitalRead(motors.openSwitchPin[LOWERSHUTTER]) == 1) ||
      (LSCS == 0 && digitalRead(motors.closedSwitchPin[LOWERSHUTTER]) == 1)) {
    stopMotor(LOWERSHUTTER);
  }
  
  // get new switch states
  USOS = digitalRead(motors.openSwitchPin[UPPERSHUTTER]);
  USCS = digitalRead(motors.closedSwitchPin[UPPERSHUTTER]);
  LSOS = digitalRead(motors.openSwitchPin[LOWERSHUTTER]);
  LSCS = digitalRead(motors.closedSwitchPin[LOWERSHUTTER]);
}

void motorSetup() {
  // set pin modes
  pinMode(motors.pwmPin[UPPERSHUTTER], OUTPUT);
  pinMode(motors.dirPin[UPPERSHUTTER], OUTPUT);

  pinMode(motors.pwmPin[LOWERSHUTTER], OUTPUT);
  pinMode(motors.dirPin[LOWERSHUTTER], OUTPUT);

  pinMode(motors.openSwitchPin[UPPERSHUTTER], INPUT_PULLUP);
  pinMode(motors.openSwitchPin[LOWERSHUTTER], INPUT_PULLUP);
  
  pinMode(motors.closedSwitchPin[UPPERSHUTTER], INPUT_PULLUP);
  pinMode(motors.closedSwitchPin[LOWERSHUTTER], INPUT_PULLUP);
}

/******************************************************************************************************/
// Main Routines
/******************************************************************************************************/

void readEeprom() {
  byte version;
  EEPROM.get(0, version);
  if (version != eepromVars.VERSION) {
      EEPROM.put(0, eepromVars);
  } else {
    EEPROM.get(0, eepromVars);
  }
}

void setup() {
  
  MYSERIAL.begin(SERIALSPEED);
  readEeprom();
  motorSetup();
  Wire.begin();
  pinMode(BATTVOLTPIN, INPUT);
  compassSetup();
  MsTimer2::set(eepromVars.VENTOPENTIME, stopVentOpen);

  // We don't need SPI or Timer1
  // Disable for some power savings
  power_spi_disable();
  power_timer1_disable();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  compassHeading();
  readBattVoltage();
  getShutterState();
  processCmd();
  checkMotors();
  watchDog(false);
}

void readBattVoltage() {
    battVoltage = (float) analogRead(BATTVOLTPIN) * BATTVOLTCONST;
}

void setBattVolt(float volt) {
  eepromVars.BATTVOLTALERT = volt;
  EEPROM.put(0, eepromVars);
}

/******************************************************************************************************/
// Input/Output Routines
/******************************************************************************************************/
void processCmd() {
  String command = "";
  if (MYSERIAL.available() > 0) {
    command = MYSERIAL.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    String recipient = command.substring(0,command.indexOf(':'));
    recipient.trim();

    command = command.substring(command.indexOf(':') + 1);
    command.trim();
    
    if (recipient.equals("DS")) {
      // We are being spoken to. Punch the Watchdog so we stay awake
      watchDog(true);
      
      if (command.equals("?")) {
          printHelp();

      /**************************************************************    
       *     Upper Shutter Commands
       **************************************************************/
      } else if (command.equals("US")) {
        MYSERIAL.println("DSR:US:" + String(shutterState[UPPERSHUTTER]));
      } else if (command.equals("USOS")) {
        MYSERIAL.println("DSR:USOS:" + String(digitalRead(motors.openSwitchPin[UPPERSHUTTER])));
      } else if (command.equals("USCS")) {
        MYSERIAL.println("DSR:USCS" + String(digitalRead(motors.closedSwitchPin[UPPERSHUTTER])));
      } else if (command.equals("OUS")) {
        openShutter(UPPERSHUTTER);  
      } else if (command.equals("CUS")) {
        closeShutter(UPPERSHUTTER);
      } else if (command.equals("VS")) {
        ventShutter();
      } else if (command.equals("SUS")) {
        stopMotor(UPPERSHUTTER);
      } else if (command.equals("RUS")) {
        reverseMotorDir(UPPERSHUTTER);
        MYSERIAL.println("DSR:RUS:OK");

      /**************************************************************    
       *     Lower Shutter Commands
       **************************************************************/

      } else if (command.equals("LS")) {
        MYSERIAL.println("DSR:LS:" + String(shutterState[LOWERSHUTTER]));
      } else if (command.equals("LSOS")) {
        MYSERIAL.println("DSR:LSOS:" + String(digitalRead(motors.openSwitchPin[LOWERSHUTTER])));
      } else if (command.equals("LSCS")) {
        MYSERIAL.println("DSR:LSCS:" + String(digitalRead(motors.closedSwitchPin[LOWERSHUTTER])));
      } else if (command.equals("OLS")) {
        openShutter(LOWERSHUTTER);  
      } else if (command.equals("CLS")) {
        closeShutter(LOWERSHUTTER);
      } else if (command.equals("SLS")) {
        stopMotor(LOWERSHUTTER);
      } else if (command.equals("RLS")) {
        reverseMotorDir(LOWERSHUTTER);
        MYSERIAL.println("DSR:RLS:OK");

      /**************************************************************    
       *     Combined Shutter Commands
       **************************************************************/

      } else if (command.equals("SS")) {
        stopMotor(UPPERSHUTTER);
        stopMotor(LOWERSHUTTER);
      } else if (command.equals("S")) {
        MYSERIAL.print("DSR:US:" + String(shutterState[UPPERSHUTTER]));
        MYSERIAL.println(":LS:" + String(shutterState[LOWERSHUTTER]));
      } else if (command.equals("RS")) {
        reverseMotorDir(UPPERSHUTTER);
        reverseMotorDir(LOWERSHUTTER);
        MYSERIAL.println("DSR:RS:OK");

      /**************************************************************    
       *     Az and Batt Commands
       **************************************************************/
      } else if (command.equals("AZ")) {
        MYSERIAL.println(String("DSR:AZ:") + domeAzimuth);
      } else if (command.equals("BATT")) {
        MYSERIAL.println(String("DSR:BATT:") + battVoltage);
      } else if (command.startsWith("SAZ")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setAzOffset(command.toFloat());
        MYSERIAL.println("DSR:SAZ:OK");

      /**************************************************************    
       *     EEPROM Commands
       **************************************************************/

      } else if (command.startsWith("EEPROM")) {
        MYSERIAL.print(F("Version: ")); MYSERIAL.println(String(eepromVars.VERSION));
        MYSERIAL.print(F("AzOffset: ")); MYSERIAL.println(String(eepromVars.AZOFFSET));
        MYSERIAL.print(F("VentOpenTime: ")); MYSERIAL.println(String(eepromVars.VENTOPENTIME));
        MYSERIAL.print(F("BattVoltAlert: ")); MYSERIAL.println(String(eepromVars.BATTVOLTALERT));
        MYSERIAL.print(F("Shutter OpenDir: US:")); MYSERIAL.print(String(eepromVars.OPENDIRECTION[0]));
        MYSERIAL.print(F(" LS:")); MYSERIAL.println(String(eepromVars.OPENDIRECTION[1]));
        MYSERIAL.print(F("BNO055 Calibrated: ")); MYSERIAL.println(String(eepromVars.CALIB));
        MYSERIAL.println(F("BNO055 Calibration Data: ")); 
        MYSERIAL.print(F("-->Acc Offset X  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.accel_offset_x));
        MYSERIAL.print(F("-->Acc Offset Y  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.accel_offset_y));
        MYSERIAL.print(F("-->Acc Offset Z  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.accel_offset_z));
        MYSERIAL.print(F("-->Gyro Offset X :")); MYSERIAL.println(String(eepromVars.CALIBDATA.gyro_offset_x));
        MYSERIAL.print(F("-->Gyro Offset Y :")); MYSERIAL.println(String(eepromVars.CALIBDATA.gyro_offset_y));
        MYSERIAL.print(F("-->Gyro Offset Z :")); MYSERIAL.println(String(eepromVars.CALIBDATA.gyro_offset_z));
        MYSERIAL.print(F("-->Mag Offset X  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.mag_offset_x));
        MYSERIAL.print(F("-->Mag Offset Y  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.mag_offset_y));
        MYSERIAL.print(F("-->Mag Offset Z  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.mag_offset_z));
        MYSERIAL.print(F("-->Mag Offset Z  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.mag_offset_z));
        MYSERIAL.print(F("-->Accel Radius  :")); MYSERIAL.println(String(eepromVars.CALIBDATA.accel_radius));
        MYSERIAL.print(F("-->Mag Radius    :")); MYSERIAL.println(String(eepromVars.CALIBDATA.mag_radius));
      } else if (command.startsWith("SBATT")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setBattVolt(command.toFloat());
        MYSERIAL.println("DSR:SBATT:OK");
      } else if (command.startsWith("SVT")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setVentOpenTime(command.toInt());
        MYSERIAL.println("DSR:SVT:OK");
      } else if (command.startsWith("CALIB")) {
        eepromVars.CALIB = false;
        compassCalib();
      }          
    }
  }
}

void printHelp() {
  MYSERIAL.println(F("All commands begin with \"DS:\" and end in \\n"));
  MYSERIAL.println(F("US=UpperShutter, LS=LowerShutter"));
  MYSERIAL.println(F("ShutterStates:0=Open,1=Closed,2=Opening,3=Closing,4=Error,5=Vent"));
  MYSERIAL.println(); 
  MYSERIAL.println(F("-->?: Help!"));
  MYSERIAL.print(F("-->US: US State. Cur=")); MYSERIAL.println(String(shutterState[UPPERSHUTTER]));
  MYSERIAL.print(F("-->USOS: US Open Switch State. Cur=")); MYSERIAL.println(String(digitalRead(motors.openSwitchPin[UPPERSHUTTER])));
  MYSERIAL.print(F("-->USCS: USClosed Switch State. Cur=")); MYSERIAL.println(String(digitalRead(motors.closedSwitchPin[UPPERSHUTTER])));
  MYSERIAL.println(F("-->OUS: Open US. ASSUMES LS IS SAFE"));
  MYSERIAL.println(F("-->CUS: Close US. ASSUMES LS IS SAFE"));
  MYSERIAL.println(F("-->VS: Vent Shutter. WILL ONLY TRIGGER IF US IS CLOSED"));
  MYSERIAL.println(F("-->SUS: Stop US Motor"));
  MYSERIAL.println();
  MYSERIAL.print(F("-->LS: LS State. Cur=")); MYSERIAL.println(String(shutterState[LOWERSHUTTER]));
  MYSERIAL.print(F("-->LSOS: LS Open Switch State. Cur=")); MYSERIAL.println(String(digitalRead(motors.openSwitchPin[LOWERSHUTTER])));
  MYSERIAL.print(F("-->LSCS: LS Closed Switch State. Cur=")); MYSERIAL.println(String(digitalRead(motors.closedSwitchPin[LOWERSHUTTER])));
  MYSERIAL.println(F("-->OLS: Open LS. ASSUMES US IS SAFE"));
  MYSERIAL.println(F("-->CLS: Close LS. ASSUMES US IS SAFE"));
  MYSERIAL.println(F("-->SLS: Stop LS Motor"));
  MYSERIAL.println();       
  MYSERIAL.print(F("-->S: Get ShutterState. Cur= US:")); MYSERIAL.print(String(shutterState[UPPERSHUTTER]) + 
  MYSERIAL.print(F(" LS:"))); MYSERIAL.println(String(shutterState[LOWERSHUTTER]));
  MYSERIAL.println(F("-->SS: Stop Both Shutter Motors"));
  MYSERIAL.println();
  MYSERIAL.print(F("-->AZ: Get Current Dome Azimuth. Cur=")); MYSERIAL.println(String(domeAzimuth, 2));
  MYSERIAL.print(F("-->BATT: Get Current Battery Voltage. Cur=")); MYSERIAL.println(String(battVoltage, 2));
  MYSERIAL.println();
  MYSERIAL.println(F("Commands to set EEPROM Vars:"));
  MYSERIAL.println(F("-->EEPROM: Dump EEPROM Variables"));
  MYSERIAL.print(F("-->SAZ: Sync to Dome Azimuth. Cur Offset=")); MYSERIAL.println(String(eepromVars.AZOFFSET));
  MYSERIAL.print(F("-->SVT: Set how far Upper Shutter should be opened for venting. Millis. Cur=")); MYSERIAL.println(String(eepromVars.VENTOPENTIME));
  MYSERIAL.println(F("-->RUS: Reverse US Motor Direction"));
  MYSERIAL.println(F("-->RLS: Reverse LS Motor Direction"));
  MYSERIAL.println(F("-->RS: Reverse Both Motor Directions"));
  MYSERIAL.println(F("-->SBATT: Set Battery Voltage alert level"));
  MYSERIAL.println(F("-->CALIB: Calibrate BNO055 9DOF Sensor"));
}

/******************************************************************************************************/
// Debug Routines
/******************************************************************************************************/

void readSwitches() {
  bool USOpen = digitalRead(motors.openSwitchPin[UPPERSHUTTER]);
  bool USClosed = digitalRead(motors.closedSwitchPin[UPPERSHUTTER]);
  bool LSOpen = digitalRead(motors.openSwitchPin[LOWERSHUTTER]);
  bool LSClosed = digitalRead(motors.closedSwitchPin[LOWERSHUTTER]);
  
  MYSERIAL.print("UO: ");
  MYSERIAL.print(USOpen);
  MYSERIAL.print(" UC: ");
  
  MYSERIAL.print(USClosed);
  MYSERIAL.print(" LO: ");
  
  MYSERIAL.print(LSOpen);
  MYSERIAL.print(" LC: ");
  
  MYSERIAL.println(LSClosed);
}
