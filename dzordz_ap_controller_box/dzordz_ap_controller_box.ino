

//#define DEBUG 1
#define EI_NOTPORTC //disabling EnableInterrupts on all pins but 7 and 8 - encoder library conflict
#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <Encoder.h>
#include <LiquidCrystal595.h>
#include <SM.h>


//pin definitions
const byte EncoderPin1 = 2;
const byte EncoderPin2 = 3;

const byte EncoderButtonPin = 4;
const byte ModeManualPin = 7;


Encoder eCommand(EncoderPin1, EncoderPin2);

LiquidCrystal595 lcd(13,12,11); 

SM smCtrl(&sIdleH, &sIdle);

//default values of all serial commands
byte isCalibrated = 0;
byte Mode = 0;
byte MotorStatus = 0;
byte LastError = 0;
int RudderPos = 0;


//motor status glyphs
byte cLeft[8] = {
  B00011,
  B00111,
  B01111,
  B11111,
  B11111,
  B01111,
  B00111,
  B00011,
};
byte cRight[8] = {
  B11000,
  B11100,
  B11110,
  B11111,
  B11111,
  B11110,
  B11100,
  B11000,
};

byte cStop[8] = {
  B00000,
  B00000,
  B11110,
  B11110,
  B11110,
  B11110,
  B00000,
  B00000,
};


int commandPos  = 0;
long encPos = -999;

//serial read things
String inputString = "";         // a string to hold incoming data
String inputCommand = "";
boolean commandSet = false;
boolean stringComplete = false;  // whether the string is complete


void setup() {
  Serial.begin(9600);

  pinMode(EncoderButtonPin, INPUT_PULLUP);
  enableInterrupt(EncoderButtonPin, resetPosition, RISING);

  pinMode(ModeManualPin, INPUT_PULLUP);
  enableInterrupt(ModeManualPin, requestModeManual, CHANGE);
  

  lcd.createChar(0,cLeft);
  lcd.createChar(1,cRight);
  lcd.createChar(2,cStop);
  lcd.begin(16,2);             // 16 characters, 2 rows
  lcd.clear();
  
}


void loop() {
  
  //getCommandPos();
  EXEC(smCtrl);
  //getSerialCommand();
  //lcdRefresh();
}

/**
 * Statemachine states
 */

State sIdleH() {

  //read status of command box
  Serial.println("g");
  lcdRefresh(); 

}


State sIdle() {
  getCommandPos();
  if (LastError == 1) smCtrl.Set(sErrorH, sError);
  if (isCalibrated ==1 && Mode == 1) smCtrl.Set(sManualH, sManual);
}



State sManualH() {
  //read status of command box
  Serial.println("g");
  lcdRefresh(); 

}

State sManual() {
  getCommandPos();
  if (LastError == 1) smCtrl.Set(sErrorH, sError);
  if (Mode == 0) smCtrl.Set(sIdleH, sIdle);

}


State sErrorH() {
  lcdRefresh();
}


State sError() {

}

void getCommandPos() {
  long newPos;
  newPos = eCommand.read();
  if (newPos < -504) eCommand.write(-504);
  if (newPos > 504) eCommand.write(504);
  if (newPos != encPos) {
    commandPos = newPos/4;
    encPos = newPos;
    lcdCommandPos();
    sendCommandPos();
    #ifdef DEBUG
      Serial.println(newPos);
    #endif
  }
}

/**
 * LCD printing
 */


// command position print

void lcdCommandPos() {
  if(commandPos < 0) {
    lcd.setCursor(0,1);
    lcd.write(byte(0));
    lcd.setCursor(1,1);
    lcd.print(commandPos*-1);
  } else if (commandPos > 0) {
    lcd.setCursor(0,1);
    lcd.write(byte(1));
    lcd.setCursor(1,1);
    lcd.print(commandPos);
  } else {
    lcd.setCursor(0,1);
    lcd.print(" 0  ");
  }

}

// Motor Status print

void lcdMotorStatus() {
  switch (MotorStatus) {
    case 0:
      lcd.setCursor(4,1);
      lcd.print("|");
      lcd.write(byte(2));
      lcd.print("|");
    break;
    case 1:
      lcd.setCursor(4,1);
      lcd.print("|");
      lcd.write(byte(0));
      lcd.print("|");
    break;
    case 2:
      lcd.setCursor(4,1);
      lcd.print("|");
      lcd.write(byte(1));
      lcd.print("|");
    break;
  }
}

// rudder position print

void lcdRudderPos() {
  if(RudderPos < 0) {
    lcd.setCursor(7,1);
    lcd.write(byte(0));
    lcd.setCursor(8,1);
    lcd.print("   ");
    lcd.setCursor(8,1);
    lcd.print(RudderPos*-1);
  } else if (RudderPos > 0) {
    lcd.setCursor(7,1);
    lcd.write(byte(1));
    lcd.setCursor(8,1);
    lcd.print("   ");
    lcd.setCursor(8,1);
    lcd.print(RudderPos);
  } else {
    lcd.setCursor(7,1);
    lcd.print(" 0  ");
  }

}

// calibration print

void lcdIsCalibrated() {
  switch (isCalibrated) {
    case 0:
      lcd.setCursor(0,0);
      lcd.print("[X]Calib");
    break;
    case 1:
      lcd.setCursor(0,0);
      lcd.print("Ok      ");
    break;
  }
}

// mode print
void lcdMode() {
  switch (Mode) {
    case 0:
      lcd.setCursor(12,0);
      lcd.print("idle");
    break;
    case 1:
      lcd.setCursor(12,0);
      lcd.print("man ");
    break;
    case 2:
      lcd.setCursor(12,0);
      lcd.print("auto");
    break;
  }
}

//error print

void lcdError() {
  switch (LastError) {
    case 1:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ERROR!");
      lcd.setCursor(0,1);
      lcd.print("Motor Timeout!");
    break;
  }
}

//refresh all

void lcdRefresh() {
  lcd.clear();
  lcdCommandPos();
  lcdMotorStatus();
  lcdRudderPos();
  lcdIsCalibrated();
  lcdMode();
  lcdError();
}

void sendCommandPos() {
  
  int c = map(commandPos,-126, 126, 0, 255);

  Serial.print("c");
  Serial.println(c);

}

void resetPosition() {
  eCommand.write(0);
}

void requestModeManual() {
  if (digitalRead(ModeManualPin) == HIGH) {
    Serial.println("m1");
    #ifdef DEBUG
      Serial.println("manual mode");
    #endif  
  } else {
    Serial.println("m0");
    #ifdef DEBUG
      Serial.println("idle mode");
    #endif
  }
  
}




//serial read things

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    String allowedChar = "bsemp";
//    if(!commandSet && allowedChar.indexOf(inChar) > -1) {
//      commandSet = true;
//      inputCommand = inChar;
//    } else if (commandSet) {
//      inputString += inChar;
//      //Serial.println(inputString);
//  
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

void getSerialCommand() {
  if (stringComplete && commandSet) {
      
    #ifdef DEBUG
      Serial.print("command = ");
      Serial.println(inputCommand);
      Serial.print("value = ");
      Serial.println(inputString);
    #endif

    if(inputCommand == "b") {
      //calibration data
      byte newCalibration = inputString.toInt();
      if (newCalibration == 0 || newCalibration == 1) {
        isCalibrated = newCalibration;
        lcdIsCalibrated();
        #ifdef DEBUG
          Serial.print("Calibrate = ");
          Serial.println(isCalibrated);
        #endif
      }
    } else if (inputCommand == "s") {
      //motor status
      byte newStatus = inputString.toInt();
      //check if in range
      if(newStatus >= 0 && newStatus <= 2) {
        MotorStatus = newStatus;
        lcdMotorStatus();
        #ifdef DEBUG
          Serial.print("MotorStatus = ");
          Serial.println(MotorStatus);
        #endif 
      }
    } else if (inputCommand == "e") {
      //error signal
      byte newError = inputString.toInt();
      //check if in range
      if(newError >= 0 && newError <= 5) {
        LastError = newError;
        lcdError();
        #ifdef DEBUG
          Serial.print("LastError = ");
          Serial.println(LastError);
        #endif 
      }
    } else if (inputCommand == "m") {
      //mode change
      byte newMode = inputString.toInt();
      //check if in range
      if(newMode >= 0 && newMode <= 2) {
        Mode = newMode;
        lcdMode();
        #ifdef DEBUG
          Serial.print("mode = ");
          Serial.println(Mode);
        #endif 
      }
    } else if (inputCommand == "p") {
      //rudder possition change
      int newCP = inputString.toInt();
    //check if value is valid
      if(newCP >= 0 && newCP <= 255) {
        RudderPos = map(newCP, 0, 255, -126, 126);
        //RudderPos = newCP;
        lcdRudderPos();
        #ifdef DEBUG
          Serial.print("rudder possiotn = ");
          Serial.println(RudderPos);
        #endif  
      }
    }

    inputString = "";
    inputCommand = "";
    stringComplete = false;
    commandSet = false;
  }
}
