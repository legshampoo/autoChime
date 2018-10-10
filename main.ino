#include <SparkFunAutoDriver.h>
#include <SPI.h>

#define dSPIN_BUSYN 5

AutoDriver board(0, 10, 6, dSPIN_BUSYN);  //  (boardIndex, CSN, RESETN, BUSYN

// motor settings
int maxSpeed = 400;
int ambientSpeed = 20;
int acceleration = 400;
int deceleration = 400;
int minStepSize = 10;
int maxStepSize = 100;

const long timeInterval = 2000;  // how long to activate chime when triggered
unsigned long previousMillis = 0;  // vars for timer 
long randNumber;  

// to keep track of motor state
long motorPosition = 0;
byte dir = FWD;

//serial input 
const byte numChars = 32;
char receivedChars[numChars];  // an array to store the received serial data
const char s[2] = ",";  //delimiter for parsing strings
boolean newData = false;

//motion detector stuff
int pirInputPin = 2;  // input pin for PIR sensor
int pirState = LOW;  // start assuming no motion detected
int val = 0;  //variable for reading the pin status

// handles which mode is currently running
boolean runAlpha = false;  //test mode to make sure basic functions are working
boolean motionDetected = false;  
boolean ambientMode = true;

void setup() {
  Serial.begin(9600);
  Serial.println("Init");

  motorPinSetup();
  pirPinSetup();
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);

  dSPINConfig();

  randomSeed(analogRead(0));  

}

void loop() {
  serialListen();   //listen for serial commands
  commandParser();    //parse serial commands
  getPosition(board);   //track motor position
  motionDetector(board);    //listen for motion detection
  commandExecute();   //execute motor action based on current state
}

void commandExecute(){
  if(runAlpha){
    alpha(board);
  }
  if(motionDetected){
    chime(board);
  }
  if(!motionDetected || ambientMode){
    ambient(board);
  }
}


// test function to check motor
void alpha(AutoDriver b){
  int stepSize = 100;
  byte dir = FWD;
  b.setMaxSpeed(500);

  if(digitalRead(dSPIN_BUSYN) == LOW){
    return;
  }else{
    if(motorPosition >= stepSize / 2){
      dir = REV;
      b.goToDir(REV, 0);
    }
    if(motorPosition <= 0){
      dir = FWD;
      b.goToDir(FWD, stepSize / 2);
    }
  }
  
}


// ambient mode for motor when nothing is happening
void ambient(AutoDriver b){
  while(digitalRead(dSPIN_BUSYN) == LOW);
  b.run(FWD, ambientSpeed);
}


// motor action when motion is detected
void chime(AutoDriver b){  
  b.setMaxSpeed(maxSpeed);
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= timeInterval){
    ambientMode = true;
    return;
  }else{
    ambientMode = false;
    if(digitalRead(dSPIN_BUSYN) == LOW){
      return;
    }else{
      randNumber = random(minStepSize, maxStepSize);
      if(dir == FWD){
        dir = REV;
      }else{
        dir = FWD;
      }
      
      b.move(dir, randNumber);
    }
  }
}


// listen for incoming serial data
void serialListen(){
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while(Serial.available() > 0 && newData == false){
    rc = Serial.read();

    if(rc != endMarker){
      receivedChars[ndx] = rc;
      ndx++;
      if(ndx >= numChars){
        ndx = numChars -1;
      }
    }else{
      receivedChars[ndx] = '\0';  //terminate the string
      ndx = 0;
      newData = true;
    }
  }
}


// parse incoming serial commands
void commandParser(){
  if(newData == true){
    Serial.print("Command: ");
    Serial.println(receivedChars);

    char *token;
    int i = 0;
    char *cmd[4];

    token = strtok(receivedChars, s);

    while(token != NULL){
      cmd[i++] = token;
      token = strtok(NULL, s);
    }

    String command = cmd[0];
    String motor = cmd[1];
    byte dir = atoi(cmd[2]);
    long steps = atoi(cmd[3]);

    if(command == "STOP"){
      board.softStop();
      runAlpha = false;
      motionDetected = false;
      ambientMode = false;
    }
    if(command == "A"){
      runAlpha = true;
      motionDetected = false;
    }

    newData = false;
  }
}


// dummy check for new serial data
void showNewData(){
  if(newData == true){
    Serial.print("This just in ... ");
    Serial.println(receivedChars);
    newData = false;
  }
}


// keeps track of motor position in case we need to use it later
void getPosition(AutoDriver b){
  motorPosition = b.getPos();

  //Serial.print("MotorPosition: ");
  //Serial.println(motorPosition);
}


// setup the motor params
void dSPINConfig(void){

  Serial.println("Running dSPIN Setup");
  
  board.SPIPortConnect(&SPI);

  board.configSyncPin(BUSY_PIN, 0);  // BUSY pin low during operations, second param ignored?
  board.configStepMode(STEP_FS_4);  //0 microsteps per step
                                  //STEP_FS_X Enable microstepping with X microsteps per full step. X can be 2, 4, 8, 16, 32, 64, or 128
                                  
  board.setMaxSpeed(maxSpeed);  // 10000 steps/s max
  
  //board.setFullSpeed(10000);  //microstep below 10000 steps/s
  board.setAcc(acceleration);  //accelerate at 10000 steps/s/s
  board.setDec(deceleration);
  board.setSlewRate(SR_530V_us);  //Upping the edge speed increases torque
  board.setOCThreshold(OC_750mA);  //OC threshold 750mA
  board.setPWMFreq(PWM_DIV_2, PWM_MUL_2);  // 31.25kHz PWM freq
  board.setOCShutdown(OC_SD_DISABLE);  //don't shutdown on OC
  board.setVoltageComp(VS_COMP_DISABLE);  //don't compensate for motor V
  board.setSwitchMode(SW_USER);  //Switch is not hard stop

  // we want 16MHz  external osc, 16MHz out (per docs)
  //  this is for comms with multiple boards, see docs when needed
  //board.setOscMode(EXT_16MHZ_OSCOUT_INVERT);  //from tutorial
  board.setOscMode(INT_16MHZ);  
  
  board.setAccKVAL(128);  //we'll tinker with these later, if needed
  board.setDecKVAL(128);
  board.setRunKVAL(128);
  board.setHoldKVAL(32);  //this controls the holding current, keep it low

  board.resetPos();

  Serial.println("Motor Ready!");

}

// set up pins for motor driver
void motorPinSetup(void){
  pinMode(6, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(13, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(6, LOW);
  digitalWrite(6, HIGH);
}


// set up pin for motion sensor
void pirPinSetup(){
  pinMode(pirInputPin, INPUT);  // pir motion sensor pin
}


// listen for motion sensor
void motionDetector(AutoDriver b){
  val = digitalRead(pirInputPin); // read motion input value
  
  if(val == HIGH){
    if(pirState == LOW){
      Serial.println("Motion Detected");
      pirState = HIGH;
      motionDetected = true;
      b.softStop();
      previousMillis = millis();
    }
  }else{
    if(pirState == HIGH){
      Serial.println("Motion Ended");
      pirState = LOW;
      motionDetected = false;
      
      b.softStop();
    } 
  }
}

