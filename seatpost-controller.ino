//Define initial values
String version = "4.24.1"; // version number (month.day.rev)
int testMode = 1; // current mode (0=no test running,1=ISO 4210 test,2=CEN 14781 test)
int cycleCount = 0;
int cycleTarget = 100000;
int stateTimeout[5] = {500,500,500,500,500}; // time (ms) before automatic state change
float windowExcursionLimit = 1.13; // multiplier used to determine allowed excursion from reference positions/deflection
bool webUpdateFlag = false;
bool webDetailsFlag = false;

//UBIDOTS CODE
#include "HttpClient.h"  // if using webIDE, use this: #include "HttpClient/HttpClient.h"
#define WEB_DEFLECTION_AVG "56a64dd97625425302aa9070"
#define WEB_CYCLES "56a64dc3762542519d338aea"
#define WEB_TEST_STATUS "56dd89df7625421de36183f1"
#define WEB_S1_POSITION_AVG "56dde4b2762542312aab510f"
#define WEB_S2_POSITION_AVG "56dde487762542303d6c2c9d"
#define WEB_ERROR_COUNT "56e6d36076254201620523d3"

HttpClient http;
// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
    { "Content-Type", "application/json" },
    { "X-Auth-Token: OYSl9VDEwl684lvbuL6PHXs0kWKse4" },  // Seatpost Test Rig Token
    { NULL, NULL } // NOTE: Always terminate headers with NULL
};

http_request_t request;
http_response_t response;
//UBIDOTS CODE

int nStates = 0; // number of states in the testMode

// Define digital input pins
int pausePin = D2; //
int displayTogglePin = D3; // toggle switch for display
int errorResetPin = D5;

// Define digital output pins
int rearRelayPin = D4; // relay for rear cylinder solenoid
int frontRelayPin = D6; // relay for front cylinder solenoid
int compressorRelayPin = D7; // pin for compressor on/off relay

// Define analog pins
int pressurePosPin = A4; //analog pin associated with pressure sensor Vout+
int pressureNegPin = A5; //analog pin associated with pressure sensor Vout-
int rearDucerPin = A1; //analog pin associated with the rear transducer
int frontDucerPin = A0; // analog pin associated with the front transducer

// Define digital as analog pins
int displayToggle2Pin = A3; // analog pin associated with display toggle #2

// Initialize measured values
int rearDucerPosBits = 0; // value used to store voltage in bits
int frontDucerPosBits = 0; // value used to store voltage in bits
int pressurePos = 0; // value used to store pressure sensor Vout+
int pressureNeg = 0; // value used to store pressure sensor Vout-

// Initialize calculated values
int pressureBits = 0; // value used to store Vout+ minus Vout-
float pressureReading = 0;  // scaled version of pressureBits in psi
float tankPressurePSI = 80; // value used to store tank pressure in PSI
float pressurePSI = 0; // value used to store pressure in PSI
float measuredForce = 0; // = pressurePSI * area in^2
float rearDucerPosInch = 0; // value used to store position in inches from zero position
float frontDucerPosInch = 0; // value used to store position in inches from zero position

// Define system hardware constants
float pullArea = 4.6019; // mcm part 6498K488
float pushArea = 4.9087; // mcm part 6498K488
int LCDrefreshRate = 300; //LCD refresh rate (ms)
int dashboardRefreshRate = 60000; // dashboard refresh rate (ms)
int maxResolution = 4095;

// Define I2C addresses
int pressureSensorAddress = 40;//same as 0x28
int dataLoggerAddress = 8; // address of dataLogger

// Digital Pressure Sensor Constants
byte mask = 63; // 00111111
int countsMax = 14745; //bits
int countsMin = 1638; //bits
int sensorMax = 150; //psi
int sensorMin = 0; //psi
// Compressor limits
int tankMin = 75; // tank pressure at which the compressor turns on
int tankMax = 110; // tank pressure at which the compressor turns off

// Initialize system/state variables
int i = 0;
long timeLeft = 0; // estimated time remaining to reach cycleTarget
long lastLCDupdate = 0; //time of last LCD update
long lastI2Cupdate = 0; // time of last I2C status check
long lastDashboardUpdate = 0; // time of last dashboard update
long compressorStartTime = 0; // time in ms that compressor was turned on
long compressorStopTime = 0; // time in ms that compressor was turned off
long compressorOnTime = 0; // # of ms compressor was on
long compressorOffTime = 0; // # of ms compressor was off
float compressorDutyCycle = 0.5;
float forceSetting = 0; // current force setting
bool dataLoggerStatus = false;
bool pressureSensorStatus = false;
bool pauseButtonState = false;
bool resetButtonState = false;
bool paused = true;
bool errorFlag = false;
int errorCount = 0; // count of consecutive errors
int errorLog = 0; // total count of all errors (including non-consecutive)
int lastErrorLog = 0; // errorLog at last web update
String errorMsg = "";
int displayMode = 0; // can be mode 0, 1, or 2

int currentState = 0;  // current state (0=null,1 & 2 are mode-dependent)
long stateStartTime = 0; // start time (ms) of current state
long stateTime = 0; // elapsed time (ms) in current state
int timeoutDelay = 0; // add-on delay for when resetError button is pressed during normal operation.
float period = 0; // Sum of all timeout states used in the current mode
float refRearPos[5] = {0,0,0,0,0}; // position measured in state i during most recent calibration
float refFrontPos[5] = {0,0,0,0,0}; // position measured in state i during most recent calibration
float refRearDeflection; // deflection measured during most recent calibration
float refFrontDeflection; // deflection measured during most recent calibration
float refRearMin[5] = {0,0,0,0,0};
float refFrontMin[5] = {0,0,0,0,0};
float refRearMax[5] = {0,0,0,0,0};
float refFrontMax[5] = {0,0,0,0,0};
float rearPos[5] = {0,0,0,0,0}; // most recent measured ducer position in state i
float frontPos[5] = {0,0,0,0,0}; // most recent measured ducer position in state i
float rearPosAvg[5] = {0,0,0,0,0}; // running average of position in state i
float frontPosAvg[5] = {0,0,0,0,0}; // running average of position in state i
float rearDeflection; // rear deflection
float frontDeflection; // front deflection
float deflection; // Total deflection
float deflectionAvg; // deflection running average
float deflectionMax; // Total deflection limit

// Flags for performing functions
bool testRelays = false;
bool calibratePressure = false;
bool calibrateWindow = false;
bool generateFvD = false; // generates force vs displacement graph
bool statusUpdate = true;
bool useI2C = true;
bool compressorOn = false;

//Define Constants
char ESC = 0xFE;

void setup()
{
    Wire.begin(); // join i2c bus (address optional for master)
    Serial1.begin(9600); // Initialize serial line for LCD
    InitializeLCD(); // Initialize LCD

    // Configure digital pins as input or output
    pinMode(compressorRelayPin, OUTPUT);
    pinMode(frontRelayPin, OUTPUT);
    pinMode(rearRelayPin, OUTPUT);
    pinMode(displayTogglePin, INPUT);
    pinMode(pausePin, INPUT);
    pinMode(errorResetPin, INPUT);

    // Declare a Particle.functions so that we can access them from the cloud.
    Particle.function("run",WebRunFunction);
    Particle.function("timeout",WebSetTimeout);

    // Ensure no relays are open and pressure is at 0
    KillAll();
    forceSetting=0;
    delay(500);

    // Initialize neutral values
    ReadInputPins();
    compressorStartTime = millis();
    compressorStopTime = millis();

    //UBIDOTS CODE
    request.hostname = "things.ubidots.com";
    request.port = 80;

    if(testMode==2){ // tbd deleteme
      WebSetTimeout("600");
    }
}// Setup


void loop()
{
    ReadInputPins();
    RunCalibrations();// runs functions enabled through webhooks

    bool stateChange = false;
    currentState = 1; // start in state 1.  state 0 is always an off state (not expected to be used)
    SetState(currentState);

    if(cycleCount>=cycleTarget) paused=true; //pause test if cycleTarget is reached

    while(!paused && !pauseButtonState && currentState<=nStates && testMode>0){
        ReadInputPins();
        stateChange = CheckStateConditions(currentState);
        if(stateChange){

            // Before changing state, update position
            rearPos[currentState] = rearDucerPosInch;
            frontPos[currentState] = frontDucerPosInch;

            if(!paused){
                currentState = currentState+1;
                SetState(currentState);
                stateStartTime = millis();
            }
            else break;
        }
        stateTime = millis()-stateStartTime;

        PrintStatusToLCD("Run");

        delay(1);//tbd deleteme
    }

    // Update deflection & position total and check against windows
    rearDeflection = rearPos[2]-rearPos[1];
    frontDeflection = frontPos[4]-frontPos[3];
    deflection = rearDeflection + frontDeflection;
    if(deflectionAvg==0) deflectionAvg = deflection; // if it's the first time through, set deflectionAvg = deflection
    else deflectionAvg = 0.9*deflectionAvg + 0.1*deflection; // otherwise use running average
    if(rearPosAvg[1]==0) rearPosAvg[1] = rearPos[1];
    else rearPosAvg[1] = 0.9*rearPosAvg[1] + 0.1*rearPos[1];
    if(rearPosAvg[2]==0) rearPosAvg[2] = rearPos[2];
    else rearPosAvg[2] = 0.9*rearPosAvg[2] + 0.1*rearPos[2];
    if(rearPosAvg[3]==0) rearPosAvg[3] = rearPos[3];
    else rearPosAvg[3] = 0.9*rearPosAvg[3] + 0.1*rearPos[3];
    if(rearPosAvg[4]==0) rearPosAvg[4] = rearPos[4];
    else rearPosAvg[4] = 0.9*rearPosAvg[4] + 0.1*rearPos[4];
    if(frontPosAvg[1]==0) frontPosAvg[1] = frontPos[1];
    else frontPosAvg[1] = 0.9*frontPosAvg[1] + 0.1*frontPos[1];
    if(frontPosAvg[2]==0) frontPosAvg[2] = frontPos[2];
    else frontPosAvg[2] = 0.9*frontPosAvg[2] + 0.1*frontPos[2];
    if(frontPosAvg[3]==0) frontPosAvg[3] = frontPos[3];
    else frontPosAvg[3] = 0.9*frontPosAvg[3] + 0.1*frontPos[3];
    if(frontPosAvg[4]==0) frontPosAvg[4] = frontPos[4];
    else frontPosAvg[4] = 0.9*frontPosAvg[4] + 0.1*frontPos[4];

    // Check deflection against limits
    if(deflection > deflectionMax){
        errorCount++;
        errorMsg = "Total defl error";
    }
    else if(rearPos[1] < refRearMin[1]){
        errorCount++;
        errorMsg = "S1 MIN error";
    }
    else if(rearPos[1] > refRearMax[1]){
        errorCount++;
        errorMsg = "S1 MAX error";
    }
    else if(rearPos[2] < refRearMin[2]){
        errorCount++;
        errorMsg = "S2 MIN error";
    }
    else if(rearPos[2] > refRearMax[2]){
        errorCount++;
        errorMsg = "S2 MAX error";
    }
    else errorCount = 0;

    // Only error out if there have been 2 consecutive errors
    if(errorCount > 1){
        paused = true;
        errorFlag = true;
    }
    else if(errorCount==1) errorLog++;

    PrintStatusToLCD("Run");

    UpdateDashboard();

    if(paused || pauseButtonState){
        PauseAll();
    }

}// loop


//------------------------------------------------------------------------
bool CheckStateConditions(int currentState){

    switch (currentState) { // these cases are all identical now, but could imagine state-specific conditions for changing state
    case 1:
        if(stateTime > stateTimeout[currentState] + timeoutDelay) return true;
        break;
    case 2:
        if(stateTime > stateTimeout[currentState] + timeoutDelay) return true;
        break;
    case 3:
        if(stateTime > stateTimeout[currentState] + timeoutDelay) return true;
        break;
    case 4:
        if(stateTime > stateTimeout[currentState] + timeoutDelay) return true;
        break;
    }

    return false;
}// CheckStateConditions


//------------------------------------------------------------------------
void SetState(int currentState){
    if(testMode==1){ // ISO 4210 test stage 1
        switch (currentState) {
        case 1:
            KillAll();
            break;
        case 2:
            PushRear();
            cycleCount++;
            break;
        }
    }
    else if(testMode==2){ // CEN 14781 test stage 1
      switch (currentState) {
        case 1:
          KillAll();
          break;
        case 2:
          PushRear();
          break;
        case 3:
          KillAll();
          break;
        case 4:
          PushFront();
          cycleCount++;
          break;
      }
    }
    else {
        KillAll();
    }
}// SetState


//------------------------------------------------------------------------
void SetMode(int testMode){
    switch (testMode)
    {
    case 0:
      testMode = 0;
      nStates = 0;
      break;

    case 1:  // ISO test stage 1
      testMode = 1;
      nStates = 2;
      break;

    case 2:  // CEN 14781 stage 1
      testMode = 2;
      nStates = 4;
      break;

    default:
      testMode = 0;
      nStates = 0;
      KillAll();
      break;
    }

    calibrateWindow = true;
}// SetMode

//------------------------------------------------------------------------
void InitializeLCD(){
    // Initialize LCD module
    Serial1.write(ESC);
    Serial1.write(0x41);
    Serial1.write(ESC);
    Serial1.write(0x51);
    // Set Contrast
    Serial1.write(ESC);
    Serial1.write(0x52);
    Serial1.write(40);
    // Set Backlight
    Serial1.write(ESC);
    Serial1.write(0x53);
    Serial1.write(8);

    ClearLCD();
    lastLCDupdate = millis();
}// InitializeLCD


//------------------------------------------------------------------------
void ClearLCD(){
    Serial1.write(ESC);
    Serial1.write(0x51);
}// ClearLCD


//------------------------------------------------------------------------
void WriteLineToLCD(String msg, int row){
    // Place cursor on correct row
    Serial1.write(ESC);
    Serial1.write(0x45);
    if(row==1) Serial1.write(0x00);
    else if(row==2) Serial1.write(0x40);
    else if(row==3) Serial1.write(0x14);
    else if(row==4) Serial1.write(0x54);
    else return;

    // Write only 20 characters
    Serial1.write(msg.substring(0,20));
}// WriteLineToLCD


//------------------------------------------------------------------------
// Reads input pins and updates the appropriate variables
void ReadInputPins(){

    // Check to see if I2C hardware is responding
    if(useI2C && millis()-lastI2Cupdate > LCDrefreshRate){
        dataLoggerStatus = CheckI2C(dataLoggerAddress);
        pressureSensorStatus = CheckI2C(pressureSensorAddress);
        lastI2Cupdate = millis();
        pressurePSI = ReadDigitalPressureSensor();
        measuredForce = pressurePSI * pullArea; //TBD update once pressure sensor 2 is working
    }

    // Check tank pressure and turn on/off as necessary
    tankPressurePSI = 0.1 * ReadAnalogPressureSensor() + (0.9) * tankPressurePSI; // poor man's running average (alpha factor of 0.03)

    if(tankPressurePSI < tankMin && !compressorOn){
        digitalWrite(compressorRelayPin,HIGH);
        compressorOn = true;
        compressorStartTime = millis();
        compressorOffTime = compressorStopTime - compressorStartTime;
    }
    else if(tankPressurePSI > tankMax && compressorOn){
        digitalWrite(compressorRelayPin,LOW);
        compressorOn = false;
        compressorStopTime = millis();
        compressorOnTime = compressorStartTime - compressorStopTime;
        compressorDutyCycle = 0.7 * (compressorOnTime) / (compressorOnTime + compressorOffTime) + 0.3 * compressorDutyCycle;
    }

    // Read transducer (position) pins
    rearDucerPosBits = analogRead(rearDucerPin);
    frontDucerPosBits = analogRead(frontDucerPin);

    rearDucerPosInch = rearDucerPosBits * 0.00144177; // based on 150mm ducer/4096 bits = .00144177"/bit
    frontDucerPosInch = frontDucerPosBits * 0.00144177; // based on 150mm ducer/4096 bits = .00144177"/bit

    // Read Digital Inputs
    pauseButtonState = digitalRead(pausePin);
    resetButtonState = digitalRead(errorResetPin);
    int displayState2 = 0;
    if(analogRead(displayToggle2Pin) > 500) displayState2 = 1;
    else displayState2 = 0;
    displayMode = digitalRead(displayTogglePin) + 2*displayState2;
    if(resetButtonState) timeoutDelay = 700;
    else timeoutDelay = 0;

    // update timeLeft
    if(testMode==2){
      period = stateTimeout[1]+stateTimeout[2]+stateTimeout[3]+stateTimeout[4];
    }
    else {
      period = stateTimeout[1]+stateTimeout[2];
    }
    timeLeft =  ( (cycleTarget-cycleCount) * period )/ 60000.0;
}// ReadInputPins


//------------------------------------------------------------------------
void RunCalibrations(){
    if(testRelays){
        TestRelays();
        testRelays = false;
    }

    if(calibrateWindow){
        KillAll();

        // run through each state and record the final deflection to set limits
        int i;
        for(i=1;i<=nStates;i++){
            PrintStatusToLCD("Calibrate State " + String(i));
            SetState(i);
            delay(stateTimeout[i]);
            ReadInputPins();
            refRearPos[i] = rearDucerPosInch;
            refFrontPos[i] = frontDucerPosInch;
        }

        refRearDeflection = refRearPos[2]-refRearPos[1];
        refFrontDeflection = refFrontPos[4]-refFrontPos[3];

        deflectionAvg = refRearDeflection + refFrontDeflection; // reset deflectionAvg
        deflectionMax = (refRearDeflection + refFrontDeflection) * windowExcursionLimit;

        // Update deflection window based on the current mode
        for(i=1;i<=nStates;i++){
            refRearMin[i] = refRearPos[i]-(refRearDeflection*(windowExcursionLimit-1));
            refRearMax[i] = refRearPos[i]+(refRearDeflection*(windowExcursionLimit-1));
            refFrontMin[i] = refFrontPos[i]-(refFrontDeflection*(windowExcursionLimit-1));
            refFrontMax[i] = refFrontPos[i]+(refFrontDeflection*(windowExcursionLimit-1));
        }

        calibrateWindow = false;
        KillAll();
        delay(2000);
    }
}// RunCalibrations


//------------------------------------------------------------------------
// Prints measured and state variables to LCD
int PrintStatusToLCD(String origMsg){
    String msg = "";

    if(pauseButtonState) origMsg += "+PAUSE BUTTON ON";

    if(millis()-lastLCDupdate > LCDrefreshRate){
        if(displayMode==0){
            msg = "#" + String(cycleCount) + "/" + String(cycleTarget);
            msg += " " + String(timeLeft) + "min";
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            if(useI2C) msg = origMsg + " M:" + String(testMode) + " S:" + String(currentState);
            else msg = origMsg + " | I2C Off";
            msg += "         ";
            WriteLineToLCD(msg,2);

            msg = "Defl " + String(deflection,3);
            msg += "/" + String(deflectionMax,3);
            msg += "         ";
            WriteLineToLCD(msg,3);

            msg = "R: " + String(rearDeflection,2);
            msg += " F: " + String(frontDeflection,2);
            msg += "         ";
            WriteLineToLCD(msg,4);

            lastLCDupdate = millis();
        }
        else if(displayMode==1){
            msg = "V" + version + " M" + String(testMode);
            msg += " " + String(timeLeft) + "min";
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            msg = "Tank:" + String(tankMin);
            msg += "<" + String(tankPressurePSI,1);
            msg += "<" + String(tankMax);
            msg += "|" + String(compressorDutyCycle,1);
            msg += "         ";
            WriteLineToLCD(msg,2);

            msg = "P: " + String(period,0);
            msg += "ms         ";
            WriteLineToLCD(msg,3);

            msg = "Win: " + String(windowExcursionLimit,2);
            msg += "         ";
            WriteLineToLCD(msg,4);

            lastLCDupdate = millis();
//Pressure sensor diagnostic
        }/*
        else if(displayMode==2){
            msg = "M" + String(testMode);
            msg += " #" + String(cycleCount) + "/" + String(cycleTarget);
            msg += " " + String(timeLeft) + "min";
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            msg = "Pressure Diagnostic";
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,2);

            msg = "P+"+String(pressurePos)+" P-"+String(pressureNeg)+ "Bits" + String(pressureBits);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,3);

            msg = "psi " + String(pressureReading,1) + "avg " + String(tankPressurePSI,1);  // scaled version of pressureBits in psi
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,4);

            lastLCDupdate = millis();
        }*/

//Excursion window diagnostic
        else if(displayMode==2){
            msg = "Pos 1:" + String(rearPos[1],3) + " 2:" + String(rearPos[2],3);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            msg = "1:"+String(refRearMin[1],3) + "<" + String(refRearPos[1],3) + "<" + String(refRearMax[1],3);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,2);

            msg = "2:"+String(refRearMin[2],3) + "<" + String(refRearPos[2],3) + "<" + String(refRearMax[2],3);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,3);

            msg = "D:" + String(deflection,3) + "/" + String(deflectionAvg,3) + "<" + String(deflectionMax,3);
            msg += "         ";
            WriteLineToLCD(msg,4);

            lastLCDupdate = millis();
        }

    }
    return 1;
}// PrintStatusToLCD


//------------------------------------------------------------------------
// Prints msg to i2c
int PrintMsg(String msg){
    if(useI2C){
        String msgChunk = "";
        while(strlen(msg) > 0){ //transmit in 32 byte chunks
            msgChunk = msg.substring(0,31); // extract 32 byte chunk
            Wire.beginTransmission(dataLoggerAddress); // transmit to dataLoggerAddress
            Wire.write(msgChunk);        // send chunk
            Wire.endTransmission();    // stop transmitting
            msg.remove(0,31);           // remove chunk
            delay(2); // prevents lost data?
        }

        // send end of line character
        Wire.beginTransmission(dataLoggerAddress);
        Wire.write((byte)0x0D);
        Wire.endTransmission();
    }
    return 1;
}// PrintMsg


//------------------------------------------------------------------------
void PrintDiagnostic(String stateStr){
    // Message Header "Time,SystemState,cycleCount,currentState,ForceSet,PSIreading(P1),rearDucerPosBits,rearDucerPosInch"
    String msg = "";

    msg += String(Time.now()) + ",";
    msg += stateStr + ",";
    msg += String(cycleCount) + ",";
    msg += String(currentState) + ",";
    msg += String(forceSetting,2) + ",";
    msg += String(pressurePSI,2) + ",";
    msg += String(rearDucerPosBits) + ",";
    msg += String(rearDucerPosInch,3);

    PrintMsg(msg);
}// PrintDiagnostic


//------------------------------------------------------------------------
void TestRelays(){
  digitalWrite(frontRelayPin,HIGH);
  delay(500);
  digitalWrite(frontRelayPin,LOW);
  delay(500);
  digitalWrite(rearRelayPin,HIGH);
  delay(500);
  digitalWrite(rearRelayPin,LOW);
  delay(500);

  KillAll();

  testRelays = false;
}// TestRelays


//------------------------------------------------------------------------
void TestMsg(String msg){
    PrintMsg(msg);
    ClearLCD();
    Serial1.write(msg);

    return;
}// TestMsg


//------------------------------------------------------------------------
float ReadDigitalPressureSensor(){
    uint16_t byte1 = 0; // stores 1st byte
    uint8_t byte2 = 0;// stores 2nd byte
    float sum = 0;

    byte mask = 63; // 00111111
    int countsMax = 14745; //bits
    int countsMin = 1638; //bits
    int sensorMax = 150; //psi
    int sensorMin = 0; //psi

    // record two bytes of data
    Wire.requestFrom(pressureSensorAddress,2);
    if(Wire.available())
        byte1 = Wire.read();
    if(Wire.available())
        byte2 = Wire.read();
    while(Wire.available()) Wire.read(); //clear out remaining bytes (bytes 3 & 4 are garbage)

    byte1 = byte1 & mask; // eliminate bits 7 and 8 (diagnostic)
    byte1 <<=8; // shift 8 bits over
    sum = byte1 + byte2; // add the two to get the 14 bit value

    float pressureReading;
    pressureReading = (sum-countsMin);
    pressureReading = pressureReading/(countsMax-countsMin);
    pressureReading = pressureReading*(sensorMax-sensorMin);

    return pressureReading;
}// ReadDigitalPressureSensor


//------------------------------------------------------------------------
float ReadAnalogPressureSensor(){

    // Read pressure pins
    pressurePos = analogRead(pressurePosPin);
    pressureNeg = analogRead(pressureNegPin);

    // Calculate pressure in PSI
    pressureBits = pressurePos - pressureNeg;
//    pressureBitsAvg = 0.1 * (pressurePos - pressureNeg) + (0.9) * pressureBitsAvg; // poor man's running average (alpha factor of 0.1)

    pressureReading = (pressureBits-3) *1.495;// minus 3 is for zero offset.  scaling factor should be 0.923 is psi/bit based on Digikey P/N 480-5548-ND, but empirically it's 1.495 psi/bit
    if(pressureReading < 0) pressureReading = 0; // system can't go vacuum

    return pressureReading;
}// ReadAnalogPressureSensor


//------------------------------------------------------------------------
int CheckI2C(int address){
    int error;

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) return 1;
    else return 0;
}// CheckI2C

//------------------------------------------------------------------------
// Series of functions for controlling cylinders
void PushRear(){
    digitalWrite(rearRelayPin,HIGH);
}
void PushFront(){
    digitalWrite(frontRelayPin,HIGH);
}
void KillRear(){
    digitalWrite(rearRelayPin,LOW);
}
void KillFront(){
    digitalWrite(frontRelayPin,LOW);
}


//------------------------------------------------------------------------
// Turns off all relays & turns pressure to minimum
void KillAll(){
  digitalWrite(rearRelayPin,LOW);
  digitalWrite(frontRelayPin,LOW);
}// KillAll


//===================================================================================================================
//Pauses cycles, turns off relays, continues to update screen, waits for un-pause
void PauseAll(){
    //turn off all relays
    KillAll();

    // wait for error flag to be reset
    while(errorFlag){
        ReadInputPins();
        if(resetButtonState){
            errorFlag = false;
            if(pauseButtonState) paused = false; // if you reset the error with the pause button down, it clears the "web pause".
        }
        UpdateDashboard();
        PrintStatusToLCD(errorMsg);
        delay(200);
    }

    // wait for pause button to be toggled off
    while(paused || pauseButtonState){
        ReadInputPins();
        UpdateDashboard();
        PrintStatusToLCD("Paused");
        if(resetButtonState && pauseButtonState) paused = false; // if you hit the reset error button while in the paused state, it clears the "web pause".
        delay(200);
    }

    // Reset timers
    compressorStartTime = millis();
    compressorStopTime = millis();
}// PauseAll


//===================================================================================================================
// Spark.functions (registered in Setup() above)
// Spark.functions always take a string as an argument and return an integer.

int WebRunFunction(String command) {
    if(command=="calibrateWindow"){
        calibrateWindow = true;
        return 1;
    }
    else if(command=="diagnostic") {
        PrintDiagnostic("");
        return 1;
    }
    else if(command=="pause"){
        if(paused) paused = false;
        else paused = true;
        return paused;
    }
    else if(command=="I2C"){
        if(useI2C) useI2C = false;
        else useI2C = true;
        return useI2C;
    }
    else if(command=="web"){
        if(webUpdateFlag) webUpdateFlag = false;
        else webUpdateFlag = true;
        return webUpdateFlag;
    }
    else if(command=="webDetails"){
        if(webDetailsFlag) webDetailsFlag = false;
        else webDetailsFlag = true;
        return webDetailsFlag;
    }
    else if(command=="testRelays") {
        testRelays = true;
        return 1;
    }
    else if(command=="neutral") {
        KillAll();
        return 1;
    }
    else if(command.substring(0,4)=="mode"){
        command = command.substring(4,5);
        testMode = command.toInt();
        SetMode(testMode);
        return testMode;
    }
    else if(command.substring(0,5)=="count"){
        command = command.substring(5);
        cycleCount = command.toInt();
        return cycleCount;
    }
    else if(command.substring(0,6)=="target"){
        command = command.substring(6);
        cycleTarget = command.toInt();
        return cycleTarget;
    }
    else if(command.substring(0,4)=="tmax"){
        command = command.substring(4);
        tankMax = command.toInt();
        return tankMax;
    }
    else if(command.substring(0,4)=="tmin"){
        command = command.substring(4);
        tankMin = command.toInt();
        return tankMin;
    }
    else if(command.substring(0,7)=="webRate"){
        command = command.substring(7);
        dashboardRefreshRate = command.toInt();
        return dashboardRefreshRate;
    }
    else if(command.substring(0,6)=="window"){
        command = command.substring(6);
        int windowPerc = command.toInt();
        windowExcursionLimit = 1 + windowPerc/100.0;

        // Update deflection window based on new limit
        deflectionMax = (refRearDeflection + refFrontDeflection) * windowExcursionLimit;
        int i;
        for(i=1;i<=nStates;i++){
          refRearMin[i] = refRearPos[i]-(refRearDeflection*(windowExcursionLimit-1));
          refRearMax[i] = refRearPos[i]+(refRearDeflection*(windowExcursionLimit-1));
          refFrontMin[i] = refFrontPos[i]-(refFrontDeflection*(windowExcursionLimit-1));
          refFrontMax[i] = refFrontPos[i]+(refFrontDeflection*(windowExcursionLimit-1));
        }
        return windowExcursionLimit;
    }
    else if(command=="resetError"){
        errorFlag = false;
        errorMsg = "";
        return 1;
    }
    else if(command=="status"){
        return cycleCount;
    }
    else{
        TestMsg(command);
        return 99;
    }

    return -1;
}// WebRunFunction


//------------------------------------------------------------------------
int WebSetTimeout(String tStr){
    if(testMode == 2){ // for CEN test, not all even states
      stateTimeout[0] = tStr.toInt();
      stateTimeout[1] = tStr.toInt()*0.4;
      stateTimeout[2] = tStr.toInt();
      stateTimeout[3] = tStr.toInt()*0.65;
      stateTimeout[4] = tStr.toInt()*0.75;
    }
    else{
      stateTimeout[0] = tStr.toInt();
      stateTimeout[1] = tStr.toInt();
      stateTimeout[2] = tStr.toInt();
      stateTimeout[3] = tStr.toInt();
      stateTimeout[4] = tStr.toInt();
    }
    return stateTimeout[0];
}// WebSetTimeout


//------------------------------------------------------------------------
void UpdateDashboard(){
    if(!webUpdateFlag) return;

    int elapsedTime = millis()-lastDashboardUpdate;

    if(paused && elapsedTime > dashboardRefreshRate*5){
      request.body = "[";
      request.body += "{ \"variable\":\""WEB_TEST_STATUS"\", \"value\": "+String(!paused)+" }";
      request.body += ", {\"variable\":\""WEB_CYCLES"\", \"value\": "+String(cycleCount)+" }";
      request.body += "]";
      request.path = "/api/v1.6/collections/values/";
      http.post(request, response, headers);
      lastDashboardUpdate = millis();
      }
    else if(elapsedTime > dashboardRefreshRate){
      request.body = "[";
      request.body += "{\"variable\":\""WEB_CYCLES"\", \"value\": "+String(cycleCount)+" }";
      request.body += ", { \"variable\":\""WEB_DEFLECTION_AVG"\", \"value\": "+String(deflectionAvg,3)+" }";
      request.body += ", { \"variable\":\""WEB_TEST_STATUS"\", \"value\": "+String(!paused)+" }";
      if(webDetailsFlag || errorLog > lastErrorLog){ // update details if there was an error, or if the webDetails flag is on
        lastErrorLog = errorLog;
        if(testMode==1){
          request.body += ", { \"variable\":\""WEB_S1_POSITION_AVG"\", \"value\": "+String(rearPosAvg[1],3)+" }";
        }
        else if(testMode==2){
          request.body += ", { \"variable\":\""WEB_S1_POSITION_AVG"\", \"value\": "+String(frontPosAvg[4],3)+" }";
        }
        request.body += ", { \"variable\":\""WEB_S2_POSITION_AVG"\", \"value\": "+String(rearPosAvg[2],3)+" }";
        request.body += ", { \"variable\":\""WEB_ERROR_COUNT"\", \"value\": "+String(errorLog)+" }";
      }
      request.body += "]";
      request.path = "/api/v1.6/collections/values/";
      http.post(request, response, headers);
      lastDashboardUpdate = millis();
    }
/*    else if(errorLog > lastErrorLog){
      request.body = "[";
      request.body += "{\"variable\":\""WEB_CYCLES"\", \"value\": "+String(cycleCount)+" }";
      request.body += ", { \"variable\":\""WEB_DEFLECTION_AVG"\", \"value\": "+String(deflection,3)+" }";
      request.body += ", { \"variable\":\""WEB_TEST_STATUS"\", \"value\": "+String(!paused)+" }";
      request.body += ", { \"variable\":\""WEB_S1_POSITION_AVG"\", \"value\": "+String(rearPos[1],3)+" }";
      request.body += ", { \"variable\":\""WEB_S2_POSITION_AVG"\", \"value\": "+String(rearPos[2],3)+" }";
      request.body += ", { \"variable\":\""WEB_ERROR_COUNT"\", \"value\": "+String(errorLog)+" }";
      request.body += "]";
      request.path = "/api/v1.6/collections/values/";
      http.post(request, response, headers);
      lastDashboardUpdate = millis();
      lastErrorLog = errorLog;
    }
*/
}// UpdateDashboard
