
//#define DEBUG 1
#define EI_NOTPORTC //disabling EnableInterrupts on all pins but 7 and 8 - encoder library conflict
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <Encoder.h>
#include <SM.h>

//State machines definisiton


SM smCalibrate(&smCalibrate_Check); //State machine for calibration
SM smMode(&smMode_CheckH, &smMode_Check); //State machine for mode (0-Idle, 1-Manual, 2-Auto);
SM smPos(&smPos_motorStopH, &smPos_motorStop); //State machine for rudder position and motor control


// Pin definitions
const byte CalibratePin = 8;
const byte EncoderPin1 = 2;
const byte EncoderPin2 = 3;

const byte ReadyPin = 6;

const byte MotorCCWPin = 10;
const byte MotorCWPin = 11;


// calibration
boolean isCalibrated = false;
//Absolute position of encoder in the moment of calibration - units are in raw endocer units (~4100 one rev)
// Should be calibrated so that 0 iz center
long calibrationFix = 20; 

// motor stop boundaries
long MotorBoundary = 2000;

boolean OnCWBounday = false;
boolean OnCCWBounday = false;

//motor timeout settings

int MotorTimeoutRange = 20000; //time in ms how long is motor allowed to run before firing and error
boolean MotorTimeout = false;

//motor status [0-off, 1-cw, 2-ccw]
byte MotorStatus = 0;

long positionTolerance = 20; //+-position tolerance (delta) - allows motor to stop in encoder is in +-tolerance

//starting mode

byte Mode = 0; //starting mode should be 0 - 1 is for debug purposes until cnotroll unit is there. 

// erroe signals 0-no error, 1 - motor timeout, 2 - Motor boundary cw, 3 - Motor boundary ccw, 
byte LastError = 0;

long rudderPos = -999;

long commandPos = 30;

int mapRudder = 0;
int mapCommandPos = 0;

void sendRudder(boolean force = false);
void sendCommandPos(boolean force = false);

Encoder rudder(EncoderPin1,EncoderPin2);


//serial read things
String inputString = "";         // a string to hold incoming data
String inputCommand = "";
boolean commandSet = false;
boolean stringComplete = false;  // whether the string is complete


void setup() {
  Serial.begin(9600);
  
  
  //init moror control pins
  pinMode(MotorCCWPin, OUTPUT);
  digitalWrite(MotorCCWPin, LOW);
  pinMode(MotorCWPin, OUTPUT);
  digitalWrite(MotorCWPin, LOW);
  
  //init calibration pins and set interrupt
  pinMode(CalibratePin, INPUT_PULLUP);  // See http://arduino.cc/en/Tutorial/DigitalPins
  pinMode(ReadyPin, OUTPUT);
  digitalWrite(ReadyPin, LOW);
  enableInterrupt(CalibratePin, calibrationFunction, CHANGE);

  inputString.reserve(200);
  
}

void loop(){

  
  EXEC(smCalibrate);
  //getSerialCommand();
  
  
  
}
/**
 * smCalibrate states
 */
State smCalibrate_Check() {
  if(isCalibrated) smCalibrate.Set(smCalibrate_Calibrated);
}

State smCalibrate_Calibrated() {
  getRudder();
  EXEC(smMode); //start position machine if it is calibrated
}

State smMode_CheckH() {
  motorStop();
  sendMode();
  #ifdef DEBUG
    Serial.println("Idle mode");
  #endif
}

State smMode_Check() {
  
  if(Mode == 1) smMode.Set(smMode_ManualH, smMode_Manual);
}

State smMode_ManualH() {
  #ifdef DEBUG
    Serial.println("Manual mode");
  #endif
  sendMode();
  
}

State smMode_Manual() {
 
  EXEC(smPos); //start position machine if it is calibrated 
 
 if(Mode == 0) {
    //position state machine to motor stop so timer doesn't run out
    smPos.Set(smPos_motorStopH, smPos_motorStop);
    
    smMode.Set(smMode_CheckH, smMode_Check);
  }
}

/**
 * smPos state machine states
 */
State smPos_motorStopH() {
  #ifdef DEBUG
    Serial.println("SM pos started");
  #endif
  motorStop();
  sendMotorStatus();
}

State smPos_motorStop() {
  boolean cwAllow, ccwAllow;

  checkBoundary();

  cwAllow = !MotorTimeout && !OnCWBounday;
  ccwAllow = !MotorTimeout && !OnCCWBounday;

  
  if(cwAllow && rudderPos > (commandPos + positionTolerance)) smPos.Set(smPos_motorCWH, smPos_motorCW);
  if(ccwAllow && rudderPos < (commandPos - positionTolerance)) smPos.Set(smPos_motorCCWH, smPos_motorCCW);
}

State smPos_motorCWH() {
  motorCW();
  sendMotorStatus();
}

State smPos_motorCW() {
  //boundary check
  if(rudderPos <= MotorBoundary*-1) smPos.Set(smPos_CWBoundaryH, smPos_CWBoundary);
  
  if(rudderPos <= (commandPos + positionTolerance)) smPos.Set(smPos_motorStopH, smPos_motorStop);

  //timer for excessive motor use
  if(smPos.Timeout(MotorTimeoutRange)) smPos.Set(smPos_motorTimeoutH, smPos_motorTimeout);
}

State smPos_motorCCWH() {
  motorCCW();
  sendMotorStatus();
}

State smPos_motorCCW() {
  //boundary check
  if(rudderPos > (MotorBoundary)) smPos.Set(smPos_CCWBoundaryH, smPos_CCWBoundary);

  if(rudderPos >= (commandPos - positionTolerance)) smPos.Set(smPos_motorStopH, smPos_motorStop);

  //timer for excessive motor use
  if(smPos.Timeout(MotorTimeoutRange)) smPos.Set(smPos_motorTimeoutH, smPos_motorTimeout);

  
}

State smPos_motorTimeoutH() {
  MotorTimeout = true;
  LastError = 1;
  #ifdef DEBUG
    Serial.println("Motor timer stop");
  #endif
  sendError();
}

State smPos_motorTimeout() {
  smPos.Set(smPos_motorStopH, smPos_motorStop);
}

State smPos_CWBoundaryH() {
  OnCWBounday = true;
  LastError = 2;
  #ifdef DEBUG
    Serial.println("Motor CW stop");
  #endif
  sendError();
}

State smPos_CWBoundary() {  
  smPos.Set(smPos_motorStopH, smPos_motorStop);
}

State smPos_CCWBoundaryH() {
  OnCCWBounday = true;
  LastError = 3;
  #ifdef DEBUG
    Serial.println("Motor CCW stop");
  #endif
  sendError();
}

State smPos_CCWBoundary() {
  smPos.Set(smPos_motorStopH, smPos_motorStop);
}


void checkBoundary() {
  if(rudderPos < MotorBoundary) OnCCWBounday = false;
  if(rudderPos > MotorBoundary*-1) OnCWBounday = false;
}

//serial read things

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    String allowedChar = "cmg";

//    if(!commandSet && allowedChar.indexOf(inChar) > -1) {
//      commandSet = true;
//      inputCommand = inChar;
//    } else if (commandSet) {
//      inputString += inChar;  
//    }
//    if (inChar == '\n') {
//      stringComplete = true;
//    }
    if (allowedChar.indexOf(inChar) > -1) {
      inputCommand = inChar;
      boolean incomingString = true;
      
      while (incomingString) {
        
        delay(2);
        
        inChar = (char)Serial.read();
        
        if (inChar == '\n') {
          incomingString = false;
        } else {
          inputString += inChar;
        }
      }
      stringComplete = true;
      commandSet = true;
      getSerialCommand();
    }
  }
}

void motorCCW() {
  digitalWrite(MotorCWPin, LOW);
  digitalWrite(MotorCCWPin, HIGH);
  MotorStatus = 2;
}
void motorCW() {
  digitalWrite(MotorCCWPin, LOW);
  digitalWrite(MotorCWPin, HIGH);
  MotorStatus = 1;
}

void motorStop() {
  digitalWrite(MotorCCWPin, LOW);
  digitalWrite(MotorCWPin, LOW);
  MotorStatus = 0;
}

/**
 * reads encoder and returns position
 */
void getRudder() {
  long newRudder;
  newRudder = rudder.read();
  if (newRudder != rudderPos) {
    rudderPos = newRudder;
    sendRudder();
    #ifdef DEBUG
      Serial.print("Rudder position = ");
      Serial.print(newRudder);
      Serial.println();
    #endif
  }
}

/**
 * Interupt function that sets global variable calibrated
 */
void calibrationFunction() {
  if (!isCalibrated) {
       rudder.write(calibrationFix);
       #ifdef DEBUG
        Serial.print("kalibrisan na ");
        Serial.println(calibrationFix);
       #endif
       isCalibrated = true;   
       digitalWrite(ReadyPin, HIGH);
    }
  sendCalibrated();
  }

void sendCalibrated() {
  if(isCalibrated) {
    Serial.println("b1");
  } else {
    Serial.println("b0");
  }
}

void sendMotorStatus() {
  Serial.print("s");
  Serial.println(MotorStatus);
}
  
void sendError() {
  Serial.print("e");
  Serial.println(LastError);
}

void sendMode() {
  Serial.print("m");
  Serial.println(Mode); 
}

void sendRudder(boolean force) {
  int limit = MotorBoundary - 100;
  int r = map(rudderPos, limit*-1, limit, 0, 255);
  if (r != mapRudder && !force) {
    mapRudder = r;
    Serial.print("p");
    Serial.println(r);   
  } else if (force) {
    Serial.print("p");
    Serial.println(r);   
  }
  
  //Serial.println(rudderPos);
}

void sendCommandPos(boolean force) {
  int limit = MotorBoundary - 100;
  int r = map(commandPos, limit*-1, limit, 0, 255);
  if (r != mapCommandPos && !force) {
    mapCommandPos = r;
    Serial.print("c");
    Serial.println(r);   
  } else if (force) {
    Serial.print("c");
    Serial.println(r);   
  }
  
}

void sendAll() {
  sendCalibrated();
  sendMotorStatus();
  sendError();
  sendMode();
  sendRudder(true);
  sendCommandPos(true);
}

void getSerialCommand() {
  if (stringComplete && commandSet) {
    if(inputCommand == "g") {
      sendAll();
    } else if (inputCommand == "m") {
      //mode change
      byte newMode = inputString.toInt();
      //check if in range
      if(newMode >= 0 && newMode <= 2 && isCalibrated) {
        Mode = newMode;
        #ifdef DEBUG
          Serial.print("mode = ");
          Serial.println(Mode);
        #endif 
      }
    } else if (inputCommand == "c") {
      //command possition change
      int newCP = inputString.toInt();
    //check if value is valid
      if(newCP >= 0 && newCP <= 255) {
        int limit = MotorBoundary - 100;
        commandPos = map(newCP, 0, 255, limit*-1, limit);
        sendCommandPos();
        #ifdef DEBUG
          Serial.print("command possiotn = ");
          Serial.println(commandPos);
        #endif  
      }
    }

    inputString = "";
    inputCommand = "";
    stringComplete = false;
    commandSet = false;
  }
}

