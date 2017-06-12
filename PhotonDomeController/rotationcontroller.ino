#include "Config.h"

/******************************************************************************************************/
// Watchdog
/******************************************************************************************************/
void watchDog(bool punch) {
    SingleThreadedSection st;
    unsigned long now = millis();
    
    if (punch) {
        watchDogTimer = now;
        RGB.color(100,0,0);
        if (watchDogExpired) {
            // Wake up rotation controller            
            // give time for the shutter controller to wakeup
            stPrintln("DS:WKUP");
            delay(500);
            watchDogExpired = false;
        }
        return;
    } 

    if (now - watchDogTimer >= WATCHDOG_TIMEOUT) {
        watchDogExpired = true;
        RGB.color(0,0,0);
    }
}

/******************************************************************************************************/
// EEPROM Routines
/******************************************************************************************************/

void reverseMotorDir() {
    eepromVars.CLOCKWISE = !eepromVars.CLOCKWISE;
    EEPROM.put(0, eepromVars);
}

void setPark(float parkPos) {
    eepromVars.PARKAZ = parkPos;
    EEPROM.put(0, eepromVars);
}

void setTempOffset(float actualTemp) {
    eepromVars.TEMPOFFSET = actualTemp - tmp102();
    obsTemp = actualTemp;
    EEPROM.put(0, eepromVars);
}

void setAutoVent(bool enable) {
    eepromVars.AUTOVENT = enable;
    EEPROM.put(0, eepromVars);
}

void setAutoBatt(bool enable) {
    eepromVars.AUTOBATT = enable;
    EEPROM.put(0, eepromVars);
}

void setAutoAC(bool enable) {
    eepromVars.AUTOAC = enable;
    EEPROM.put(0, eepromVars);
}

void setVentOn(float threshold) {
    eepromVars.VENTON = threshold;
    EEPROM.put(0, eepromVars);
}

void setVentOff(float threshold) {
    eepromVars.VENTOFF = threshold;
    EEPROM.put(0, eepromVars);
}

void setVentOutside(float threshold) {
    eepromVars.VENTOUTSIDE = threshold;
    EEPROM.put(0, eepromVars);
}

void setACOn(float threshold) {
    eepromVars.ACON = threshold;
    EEPROM.put(0, eepromVars);
}

void setACOff(float threshold) {
    eepromVars.ACOFF = threshold;
    EEPROM.put(0, eepromVars);
}

void readEeprom() {
    byte version;
    EEPROM.get(0, version);
    if (version != eepromVars.VERSION) EEPROM.put(0, eepromVars);
    else EEPROM.get(0, eepromVars);
}

/******************************************************************************************************/
// Cloud support routines
/******************************************************************************************************/

int cOpenShutter(String cmd) {
    processCmd("OS", false);
}

int cCloseShutter(String cmd) {
    processCmd("CS", false);
}

int cVentShutter(String cmd) {
    processCmd("VS", false);
}

int cAutoVentOn(String cmd) {
    setAutoVent(true);
    Particle.publish("AutoVentOn");
}

int cAutoVentOff(String cmd) {
    setAutoVent(false);    
    Particle.publish("AutoVentOff");
}

int cAutoACOn(String cmd) {
    setAutoAC(true);
    Particle.publish("AutoACOn");
}

int cAutoACOff(String cmd) {
    setAutoAC(false);
    Particle.publish("AutoACOff");
}

int cPark(String cmd) {
    processCmd("P", false);
}

int cWSEnablePublish(String cmd) {
    wsEnPub = true;
}

int cWSDisablePublish(String cmd) {
    wsDisPub = true;
}

int cWSEnableFlash(String cmd) {
    wsEnFlash = true;
}

int cWSDisableFlash(String cmd) {
    wsDisFlash = true;
}

/******************************************************************************************************/
// Setup Routines
/******************************************************************************************************/

void keyfobSetup() {
    pinMode(ANTICLOCKWISEPIN, INPUT);    
    pinMode(CLOCKWISEPIN, INPUT);    
    pinMode(CSPIN, INPUT);    
    pinMode(OSPIN, INPUT);    
}

void tempSensorSetup() {
    tempSensor.begin();
    // set the Conversion Rate (how quickly the sensor gets a new reading)
    //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
    tempSensor.setConversionRate(3);
    //set Extended Mode.
    //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
    tempSensor.setExtendedMode(0);
    
    pinMode(TMP36PIN, INPUT);
}

void motorSetup() {
    pinMode(motor.pwmPin, OUTPUT);
    pinMode(motor.dirPin, OUTPUT);
}

void setup() {
    MYSERIAL.begin(SERIALSPEED);
    
    readEeprom();
    Wire.begin();
    motorSetup();
    tempSensorSetup();
    
    RGB.control(true);
    RGB.color(100,0,0);
    
    Thread *shutterState = new Thread("shutterState", readShutterState);
    Thread *shutter = new Thread("shutter", shutterThread);
    Thread *az = new Thread("az", readAz);
    Thread *batt = new Thread("batt", readBatt);
    Thread *dome = new Thread("dome", moveDome);
    Thread *temp = new Thread("temperature", readTemp);
    Thread *cloud = new Thread("cloud", cloudThread);

    // Register functions on the cloud
    bool success = Particle.function("OpenShutter", cOpenShutter);
    success = Particle.function("CloseShutter", cCloseShutter);
    success = Particle.function("VentShutter", cVentShutter);
    success = Particle.function("AutoVentOn", cAutoVentOn);
    success = Particle.function("AutoVentOff", cAutoVentOff);
    success = Particle.function("AutoACOn", cAutoACOn);
    success = Particle.function("AutoACOff", cAutoACOff);
    success = Particle.function("Park", cPark);
    success = Particle.function("WSEnPublish", cWSEnablePublish);
    success = Particle.function("WSDisPublish", cWSDisablePublish);
    success = Particle.function("WSEnFlash", cWSEnableFlash);
    success = Particle.function("WSDisFlash", cWSDisableFlash);
}

void loop() {
    Particle.process();
    watchDog(false);
}

/******************************************************************************************************/
// Motor routines
/******************************************************************************************************/

void rotateClockwise() {
  if (motorMoving) return;
  digitalWrite(motor.dirPin, eepromVars.CLOCKWISE);
  startMotor();
}

void rotateAntiClockwise() {
    if (motorMoving) return;
    digitalWrite(motor.dirPin, !eepromVars.CLOCKWISE);
    startMotor();
}

void startMotor() {
    analogWrite(motor.pwmPin, motor.duty, 20000);
    motorMoving = true;
}

void stopMotor() {
    analogWrite(motor.pwmPin, LOW);
    motorMoving = false;
}

void moveDome() {
    byte antiClockWise = 0;
    byte clockWise = 0;

    while (true) {
        // If motor is moving, keep punching watch dog
        if (motorMoving) watchDog(true);
        
        if (slewActive) {
            // Slew command is active
            
            // Slow down if close to target
            if (abs(commandAz - currentAz) < SLOWSLEW) analogWrite(motor.pwmPin, motor.duty/2, 20000);
            
            // Stop if slew done
            if (abs(commandAz - currentAz) <= SLEWTOLERANCE) {
                slewActive = false;
                stopMotor();
            }
            
            // Start slew if motor is not on
            if (!motorMoving) {
                if (commandAz > currentAz) {
                    if (commandAz - currentAz < currentAz + 360 - commandAz) rotateClockwise();
                    else rotateAntiClockwise();
                } else {
                    if (currentAz - commandAz < commandAz + 360 - currentAz) rotateAntiClockwise();        
                    else rotateClockwise();
                }
            }        
       } else {
            // Key fob will only respond if a slew is not active
            byte newAntiClockWise = digitalRead(ANTICLOCKWISEPIN);
            byte newClockWise = digitalRead(CLOCKWISEPIN);
            
            // if button pressed, turn dome
            if (antiClockWise == 0 &&  newAntiClockWise == 1) {
                //If A is pressed rotate anticlockwise
                if (!motorMoving) rotateAntiClockwise();
            } else if (clockWise == 0 &&  newClockWise == 1) {
                //If C is pressed rotate clockwise
                if (!motorMoving) rotateClockwise();
            // If motor is currently moving (and slew is not active)
            } else if (motorMoving) {
                // If button not continued to be held, stop motor
                if (!(antiClockWise == 1 &&  newAntiClockWise == 1) &&
                    !(clockWise == 1 &&  newClockWise == 1)) {
                    analogWrite(motor.pwmPin, motor.duty/2, 20000);
                    stopMotor();
                }
            }
            
            clockWise = newClockWise; 
            antiClockWise = newAntiClockWise;
       }
    }
}

/******************************************************************************************************/
// Supporting Thread for cloud functions to Weather Station
/******************************************************************************************************/

void cloudThread() {
    while (true) {
        if (wsEnPub) {
            stPrintln("WS:EnPublish:1");
        }

        if (wsDisPub) {
            stPrintln("WS:EnPublish:0");
        }

        if (wsEnFlash) {
            stPrintln("WS:EF");
        }
        if (wsDisFlash) {
            stPrintln("WS:DF");
        }

        if (wsEnPub || wsDisPub || wsEnFlash || wsDisFlash) delay(5000);
        else delay(60000);
    }
}

/******************************************************************************************************/
// Azimuth Thread
/******************************************************************************************************/

void readAz() {
    static unsigned long lastRead = millis();
    
    while (true) {
        unsigned long now = millis();
        
        if (!watchDogExpired) {
            unsigned long loopTimer = AZREAD;
            if (!slewActive) loopTimer = AZREADPASSIVE;
            
            if (now - lastRead >= loopTimer) {
                lastRead = now;
                processCmd("AZ", false);
                if (slewActive) parked = false;
                else if (abs(currentAz - eepromVars.PARKAZ) < SLEWTOLERANCE) parked = true; 
            }
        }
    }
}

/******************************************************************************************************/
// Batt Thread
/******************************************************************************************************/

void readBatt() {
    static unsigned long lastRead = millis();
    
    while (true) {
        unsigned long now = millis();
        
        if (shutter != shutterClosed) {
            if (now - lastRead >= BATTREAD) {
                lastRead = now;
                battCmd = true;
                processCmd("BATT", false);
                while (battCmd) delay(1000);
                if (battVoltage < eepromVars.BATTVOLTALERT) {
                    Particle.publish("LOWSHUTTERBATT", String(battVoltage, 2));
                    if (eepromVars.AUTOBATT) {
                        closeShutter = true;
                    }
                }
            }
        }
    }
}

/******************************************************************************************************/
// Shutter Threads
/******************************************************************************************************/

void readShutterState() {
    static unsigned long lastRead = millis();

    delay(SHUTTERREAD);
    processCmd("LS", false);
    delay(SHUTTERREAD);
    processCmd("US", false);
    delay(SHUTTERREAD);
    shutter = shutterError;
    if (upperShutter == shutterOpen && lowerShutter == shutterOpen) shutter = shutterOpen;
    if (upperShutter == shutterClosed && lowerShutter == shutterClosed) shutter = shutterClosed;
    if (upperShutter == shutterVent && lowerShutter == shutterClosed) shutter = shutterVent;
    if (upperShutter == shutterOpening || lowerShutter == shutterOpening) shutter = shutterOpening;
    if (upperShutter == shutterClosing || lowerShutter == shutterClosing) shutter = shutterClosing;

    while (true) {
        unsigned long now = millis();

        if (!watchDogExpired) {
            unsigned long loopTimer = SHUTTERREAD;
            if (!openShutter && !closeShutter) loopTimer = SHUTTERREADPASSIVE;
    
            if (now - lastRead >= loopTimer) {
                lastRead = now;
                processCmd("LS", false);
                delay(SHUTTERREAD);
                processCmd("US", false);
                delay(SHUTTERREAD);
                shutter = shutterError;
                if (upperShutter == shutterOpen && lowerShutter == shutterOpen) shutter = shutterOpen;
                if (upperShutter == shutterClosed && lowerShutter == shutterClosed) shutter = shutterClosed;
                if (upperShutter == shutterVent && lowerShutter == shutterClosed) shutter = shutterVent;
                if (upperShutter == shutterOpening || lowerShutter == shutterOpening) shutter = shutterOpening;
                if (upperShutter == shutterClosing || lowerShutter == shutterClosing) shutter = shutterClosing;
            }
        }
    }
}

void shutterThread() {
    while (true) {
        
        // if no other commands active, keyfob can be used to open and close
        if (digitalRead(OSPIN) && (shutter == shutterClosed || shutter == shutterVent)) {
            watchDog(true);
            openShutter = true;
        } else if (digitalRead(CSPIN) && (shutter == shutterOpen || shutter == shutterVent)) {
            watchDog(true);
            closeShutter = true;
        }

        if (openShutter) {
            processCmd("OUS", false);
            while (upperShutter != shutterOpen) {
                watchDog(true);
                delay(10);
            }
            processCmd("OLS", false);
            while (lowerShutter != shutterOpen) {
                watchDog(true);
                delay(10);
            }
            openShutter = false;
            Particle.publish("ShutterOpen");
        }

        if (closeShutter) {
            processCmd("CLS", false);
            while (lowerShutter != shutterClosed) {
                watchDog(true);
                delay(10);
            }
            processCmd("CUS", false);
            while (upperShutter != shutterClosed) {
                watchDog(true);
                delay(10);
            }
            closeShutter = false;
            Particle.publish("ShutterClosed");
        }

        if (ventShutter) {
            processCmd("VS", false);
            while (shutter != shutterVent) {
                watchDog(true);
                delay(10);
            }
            ventShutter = false;
            ventOn = true;
            Particle.publish("ShutterVenting");
        }
    }
}

/******************************************************************************************************/
// Obs Temp Thread
/******************************************************************************************************/
float tmp102() {
    tempSensor.wakeup();
    float temp = tempSensor.readTempF();
    tempSensor.sleep();
    return temp;
}

float tmp36() {
    int reading = analogRead(TMP36PIN);
    float voltage = reading * 3.3;
    voltage /= 4096.0; 

    //converting from 10 mv per degree wit 500 mV offset
    //to degrees ((voltage - 500mV) times 100)
    float temperatureC = (voltage - 0.5) * 100;  
    float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
    
    return temperatureF;
}

// Logic: 3 different temp set points
// X (lowest) : start venting to try and bring down the temperature
// Y (highest) : close vent and start AC
// Z (middle) : Just so we are not running AC all the time, turn off AC when we drop below this

void readTemp() {
    float oldTemp = 0;
    while (true) {
        obsTemp = tmp36() + eepromVars.TEMPOFFSET;

        if (obsTemp <= eepromVars.VENTOFF && ventOn) {
            watchDog(true);
            Particle.publish("OBSTEMP", String(obsTemp, 2));
            closeShutter = true;
            while(shutter != shutterClosed) delay(1000);
            ventOn = false;
            Particle.publish("VENTOFF");
        }        

        // if temperature rises above set point
        // and auto vent is enabled and ac is not on
        // start venting

        if (obsTemp >= eepromVars.VENTON && obsTemp < eepromVars.ACON && !acOn) {
            if (eepromVars.AUTOVENT) {
                // make sure it's not raining if opening vent
                // or if vent is already open
                
                unsigned long now = millis();
                wsRainCmd = true;
                rainDetect = true;
                // Try to check rain condition for 1 minute
                // if weatherstation inactive, assume it's raining
                while (wsRainCmd && (millis() - now < 60000)) {
                    stPrintln("WS:R");
                    delay(5000);
                }

                wsTempCmd = true;
                outsideTemp = 100.0;
                now = millis();
                // Try to check outside temp for 1 minute
                // if weatherstation inactive, assume it's super hot
                // Venting only makes sense if outside temp is not too high
                while (wsTempCmd && (millis() - now < 60000)) {
                    stPrintln("WS:T");
                    delay(5000);
                }
                
                if (!ventOn && !rainDetect && outsideTemp < eepromVars.VENTOUTSIDE) {
                    watchDog(true);
                    Particle.publish("OBSTEMP", String(obsTemp, 2));
                    ventShutter = true;
                    while(shutter != shutterVent) delay(1000);
                    Particle.publish("VENTON");
                }
                
                if (ventOn && rainDetect) {
                    watchDog(true);
                    Particle.publish("OBSTEMP", String(obsTemp, 2));
                    closeShutter = true;
                    while(shutter != shutterClosed) delay(1000);
                    ventOn = false;
                    Particle.publish("VENTOFF");
                }
            }
        }

        // If ac is on, turn it off when we drop below set point
        // saves some energy, especially if using a big ac unit
        if (obsTemp <= eepromVars.ACOFF && acOn) {
            Particle.publish("OBSTEMP", String(obsTemp, 2));        
            Particle.publish("ACOFF");
            acOn = false;
        } 

        // If temperature above set point
        // and auto ac enabled, turn on ac (close vent if it's open)
        if (obsTemp >= eepromVars.ACON && !acOn) {
            if (eepromVars.AUTOAC) {
                Particle.publish("OBSTEMP", String(obsTemp, 2));
                if (ventOn) closeShutter = true;
                while (shutter != shutterClosed) delay(1000);
                ventOn = false;
                Particle.publish("ACON");
                acOn = true;
            }
        } 

        oldTemp = obsTemp;
        delay(TEMPREAD);
    }
}

/******************************************************************************************************/
// Command Routines
/******************************************************************************************************/

// Single threaded to avoid clobbering serial
void stPrint(String resp) {
    SingleThreadedSection st;
    MYSERIAL.print(resp);
    MYSERIAL.flush();
}

// Single threaded to avoid clobbering serial
void stPrintln(String resp) {
    SingleThreadedSection st;
    MYSERIAL.println(resp);
    MYSERIAL.flush();
}

void serialEvent1() {
    String command = "";
    command = MYSERIAL.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    String recipient = command.substring(0,command.indexOf(':'));
    recipient.trim();

    command = command.substring(command.indexOf(':') + 1);
    command.trim();
    
    if (recipient.equals("DC")) {
        processCmd(command, true);
    }
    
    if (recipient.equals("DSR")) {
        rcvDSR(command);
    }

    if (recipient.equals("WSR")) {
        rcvWSR(command);
    }
}

void processResp(String resp, bool r) {
    if (r) stPrintln(resp);
}

void processCmd(String command, bool resp) {
    watchDog(true);
    
    //********Help******************************************
    if (command.equals("?")) {
        printHelp();

    //*******Shutter Commands*******************************
    } else if (command.equals("SS")) {
        openShutter = false;
        closeShutter = false;
        processResp("DS:SS", resp);
    } else if (command.equals("OS")) {
        if (shutter == shutterClosed || shutter == shutterVent) openShutter = true;
        else processResp("DCR:OS:ERR", resp);
    } else if (command.equals("CS")) {
        if (shutter == shutterOpen || shutter == shutterVent) closeShutter = true;
        else processResp("DCR:CS:ERR", resp);
    } else if (command.equals("VS")) {
        if (shutter == shutterClosed) stPrintln("DS:VS");
        else processResp("DCR:VS:ERR", resp);
    } else if (command.equals("S")) {
        if (shutter == shutterVent) processResp("DCR:S:" + String(shutterOpen), resp);
        else processResp("DCR:S:" + String(shutter), resp);
    } else if (command.equals("OUS")) {
        stPrintln("DS:OUS");
    } else if (command.equals("OLS")) {
        stPrintln("DS:OLS");
    } else if (command.equals("CUS")) {
        stPrintln("DS:CUS");
    } else if (command.equals("CLS")) {
        stPrintln("DS:CLS");
    } else if (command.equals("LS")) {
        stPrintln("DS:LS");
    } else if (command.equals("US")) {
        stPrintln("DS:US");

    //********************Slew commands*********************
    } else if (command.startsWith("SLEWAZ")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        commandAz = command.toFloat();
        slewActive = true;
    } else if (command.equals("P")) {
        commandAz = eepromVars.PARKAZ;
        slewActive = true;
    } else if (command.equals("STOPSLEW")) {
        slewActive = false;

    //*************Set/Get commands related to slew**********
    } else if (command.startsWith("SETP")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setPark(command.toFloat());
        processResp("DCR:SETP:OK", resp);
    } else if (command.equals("GETP")) {
        processResp("DCR:GETP:" + String(parked), resp);
    } else if (command.equals("GETSLEW")) {
        processResp("DCR:GETSLEW:" + String(slewActive), resp);
    } else if (command.startsWith("SAZ")) {
        if (!slewActive) {
            command = command.substring(command.indexOf(':') + 1);
            command.trim();
            processResp("DS:SAZ:" + String(command.toFloat(),2), resp);
        }
    } else if (command.equals("AZ")) {
        stPrintln("DS:AZ");
        processResp("DCR:AZ:" + String(currentAz,2), resp);
    //********************Sensor Read Commands***************
    } else if (command.equals("T")) {
        processResp("DCR:T:" + String(obsTemp,2), resp);
    } else if (command.equals("BATT")) {
        stPrintln("DS:BATT");
        processResp("DCR:BATT:" + String(battVoltage,2), resp);

    //********************Auto Commands**********************
    } else if (command.equals("AUTOVENT")) {
        processResp("DCR:AUTOVENT:" + String(ventOn), resp);
    } else if (command.equals("AUTOAC")) {
        processResp("DCR:AUTOAC:" + String(acOn), resp);

    //********************EEPROM Commands********************
    } else if (command.equals("REV")) {
        reverseMotorDir();
    } else if (command.startsWith("SETAUTOVENT")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setAutoVent((bool)command.toInt());
    } else if (command.startsWith("SETAUTOAC")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setAutoAC((bool)command.toInt());
    } else if (command.startsWith("SETAUTOBATT")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setAutoBatt((bool)command.toInt());
    } else if (command.startsWith("SETVENTON")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setVentOn(command.toFloat());
    } else if (command.startsWith("SETVENTOFF")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setVentOff(command.toFloat());
    } else if (command.startsWith("SETVENTOUTSIDE")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setVentOutside(command.toFloat());
    } else if (command.startsWith("SETACON")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setACOn(command.toFloat());
    } else if (command.startsWith("SETACOFF")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setACOff(command.toFloat());
    } else if (command.startsWith("SETT")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        setTempOffset(command.toFloat());
    }
}

void rcvWSR(String command) {
    //********************Cloud function response********************
    if (command.equals("EnPublish:OK")) {
        if (wsEnPub) {
            wsEnPub = false;
            Particle.publish("WeatherStationPublishEnabled");
        }
        if (wsDisPub) {
            wsDisPub = false;
            Particle.publish("WeatherStationPublishDisabled");
        }
    } else if (command.equals("EF:OK")) {
        wsEnFlash = false;
        Particle.publish("WeatherStationFlashEnabled");
    } else if (command.equals("DF:OK")) {
        wsDisFlash = false;
        Particle.publish("WeatherStationFlashDisabled");
    } else if (command.equals("DF:OK")) {
        wsDisFlash = false;
    } else if (command.startsWith("R:")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        rainDetect = (bool)command.toInt();
        wsRainCmd = false;
    } else if (command.startsWith("T:")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        outsideTemp = command.toFloat();
        wsTempCmd = false;
    }
}

void rcvDSR(String command) {
    //********************Slew command response********************
    if (command.startsWith("AZ")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        currentAz = command.toFloat();
    } else if (command.equals("BATT")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        battVoltage = command.toFloat();
    } else if (command.equals("SAZ:OK")) {
        stPrintln("DCR:SAZ:OK");
    
    //********************Shutter commands*************************
    } else if (command.equals("VS:OK")) {
        stPrintln("DCR:VS:OK");
        Particle.publish("ShutterVent");
    } else if (command.equals("SS:OK")) {
        stPrintln("DCR:SS:OK");
    } else if (command.startsWith("LS")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        lowerShutter = (shutterstate)command.toInt();
    } else if (command.startsWith("US")) {
        command = command.substring(command.indexOf(':') + 1);
        command.trim();
        upperShutter = (shutterstate)command.toInt();
    }
}

void printHelp() {
    stPrintln(F("All commands begin with \"DC:\""));
    stPrintln(F("All commands end in \"\\n\""));
    stPrintln(F("ShutterStates: 0=Open, 1=Closed, 2=Opening, 3=Closing, 4=Error, 5=Vent(Non-Ascom state)"));
    stPrintln(""); 
    stPrintln(F("-->?: Print this help"));
    stPrintln("");
    stPrintln(F("-->SS: Stop All Shutters"));
    stPrintln(F("-->OS: Open All Shutters"));
    stPrintln(F("-->CS: Close All Shutters"));
    stPrintln(F("-->VS: Vent Upper Shutter"));
    stPrint(F("-->S: Get shutter state. Cur=")); stPrintln(String(shutter));
    stPrintln("");
    stPrintln(F("-->P: Park the dome"));
    stPrintln(F("-->SLEWAZ: Slew dome to Az"));
    stPrintln(F("-->GETSLEW: Get current slew status"));
    stPrint(F("-->SETP: Set Park Position Az. Cur=")); stPrintln(String(eepromVars.PARKAZ, 2));
    stPrintln(F("-->GETP: Get parked status"));
    stPrintln(F("-->SAZ: Sync Current AZ to reported position"));
    stPrintln(F("-->STOPSLEW: Stop any active Dome Slew"));
    stPrintln("");
    stPrint(F("-->AZ: Get current Az. Cur=")); stPrintln(String(currentAz,2));
    stPrint(F("-->LS: Get Lower Shutter State. Cur=")); stPrintln(String(lowerShutter));
    stPrint(F("-->US: Get Upper Shutter State. Cur=")); stPrintln(String(upperShutter));
    stPrint(F("-->T: Get Observatory Temperature. Cur=")); stPrintln(String(obsTemp, 2));
    stPrint(F("-->AUTOVENT: Is venting on through Auto Vent?. Cur=")); stPrintln(String(ventOn));
    stPrint(F("-->AUTOAC: Is AC on through Auto AC. Cur=")); stPrintln(String(acOn));
    stPrint(F("-->BATT: Get Shutter Battery Level. Cur=")); stPrintln(String(battVoltage, 2));
    stPrintln("");
    stPrint(F("-->REV: Reverse Motor Dir. Clockwise Cur=")); stPrintln(String(eepromVars.CLOCKWISE));
    stPrint(F("-->SETT: Set actual temperature. Cur=")); stPrintln(String(obsTemp, 2));
    stPrint(F("-->SETAUTOVENT: En(1)/Disable(0) Auto Vent. Cur=")); stPrintln(String(eepromVars.AUTOVENT));
    stPrint(F("-->SETAUTOAC: En(1)/Disable(0) Auto AC. Cur=")); stPrintln(String(eepromVars.AUTOAC));
    stPrint(F("-->SETAUTOBATT: Should we close shutters if battery gets too low? Cur=")); stPrintln(String(eepromVars.AUTOBATT));
    stPrint(F("-->SETVENTON: Set temperature above which venting happens. Cur=")); stPrintln(String(eepromVars.VENTON, 2));
    stPrint(F("-->SETVENTOFF: Set temperature below which venting stops. Cur=")); stPrintln(String(eepromVars.VENTON, 2));
    stPrint(F("-->SETVENTOUTSIDETEMP: Set outside temperature below which venting happens. Cur=")); stPrintln(String(eepromVars.VENTON, 2));
    stPrintln(F("-->SETACON: Set temperature above which AC will kick in."));
    stPrint(F("            Shutter will be closed if venting/open. Cur=")); stPrintln(String(eepromVars.ACON, 2));
    stPrint(F("-->SETACOFF: Set temperature below which AC will shutoff. Cur=")); stPrintln(String(eepromVars.ACOFF, 2));
}
