#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>

int limitOben=2688;
int limitUnten=13;

const int pinGateRef = 3;

const int pinButtonDown = 4; // gruen
const int pinButtonSet = 5; // blau

const int pinReference = 6; // weiss:refernzfahrt
const int pinTaster = 7;
const int pinButtonUp = 8; // rot
const int pinManualMode = 11; // gelb: Kipp-Schalter


Servo servo1;


/* *********   */
#define SERVO_HOLD 90
#define SERVO_FORW 0
#define SERVO_BACK 180
#define MOVING_TIME 150

class Catcher{
  
  bool isDriving = false;
  bool wasSeen = false;
  bool isOpen = false;  
  long previousMillisGate = 0;
  bool initDone = false;
  int initDirection = SERVO_FORW;
  int  direction = SERVO_HOLD;
  Servo theServo;
  int _servoPin;
  int _refPin;

  void runToInit()
  {
   if( digitalRead( _refPin ) == HIGH)
     theServo.write(initDirection);
   else{
     theServo.write(SERVO_HOLD);
     isOpen = false;
     initDone=true;
   }    
  }

  bool shouldMove( bool shouldOpen )
  {
   if( isDriving == false)
   {
     if( shouldOpen == true && isOpen == false){
       direction = SERVO_FORW;
     }else if(shouldOpen == false && isOpen == true){
       direction = SERVO_BACK;
     }
  
     if( direction == SERVO_HOLD){
       theServo.write(direction);
       return false;
     }      
   }
   return true; 
  }

public:
  Catcher(int servoPin, int refPin)
  {
    _servoPin = servoPin;
    _refPin = refPin;
  }
    
  void setup()
  {
    theServo.attach(_servoPin);
    pinMode(_refPin, INPUT_PULLUP);    
  }

  void drive(bool shouldOpen)
  {
   if( initDone == false)
      return runToInit();   

    if( shouldMove(shouldOpen) == false)
      return;

   if( isDriving == true && digitalRead(pinGateRef) == LOW)
     wasSeen = true;
  
   long m = millis();
   if( isDriving == false){
     previousMillisGate = m;
     wasSeen = false;
     isDriving = true;
     if( direction == SERVO_FORW )
       Serial.println("Open SERVO_FORW");
      else
       Serial.println("Open SERVO_BACK");
  
    }else if( m - previousMillisGate >= MOVING_TIME || (digitalRead(_refPin) == LOW && direction == SERVO_BACK)){
     Serial.println( m - previousMillisGate );
     if( direction == SERVO_FORW ){
       isOpen = true;
       Serial.println("Gate open");
     }else{
       isOpen = false;
       Serial.println("Gate closed");
       if( digitalRead(_refPin) == HIGH){
         if(wasSeen == false)
           initDirection = SERVO_BACK;
          else
          initDirection = SERVO_FORW;
         initDone = false;
       }
     }
     isDriving = false;
     direction = SERVO_HOLD;
    }
  
    theServo.write(direction);        
    
  }
  
private:


};





Catcher theCatcher(9,pinGateRef);



#define AB FORWARD
#define AUF BACKWARD


#define HIGHSPEED 800
#define LOWSPEED 100
#define MANUAL_RAMPE 200

bool puls1000ms = false;


Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *motor = AFMS.getStepper(200, 2);

int position = 0;

#define REF_STATE_UNKNOWN 0
#define REF_STATE_LIFTOUT 1
#define REF_STATE_LIFTOUT_SPACE 2
#define REF_STATE_SEARCH 3
#define REF_STATE_DONE 4



byte statusRefPointDrive = REF_STATE_UNKNOWN;


bool checkRefPoint(){
  static int countLiftOut = 0;
  if (statusRefPointDrive == REF_STATE_DONE)
    return true;
    
   int refSwitch = digitalRead( pinReference ); 
    
  if (statusRefPointDrive == REF_STATE_UNKNOWN && refSwitch == LOW ){
    statusRefPointDrive = REF_STATE_LIFTOUT;
  }else if(statusRefPointDrive == REF_STATE_LIFTOUT && refSwitch == HIGH){
    statusRefPointDrive = REF_STATE_LIFTOUT_SPACE;
    countLiftOut = 0;    
  }else if(statusRefPointDrive == REF_STATE_LIFTOUT_SPACE)  {
    if(puls1000ms)
      countLiftOut++; 
    if( countLiftOut > 2)
      statusRefPointDrive = REF_STATE_SEARCH;    
  }else if(statusRefPointDrive == REF_STATE_SEARCH && refSwitch == LOW){
    statusRefPointDrive = REF_STATE_DONE; 
    position = 0;
  }else if(statusRefPointDrive == REF_STATE_UNKNOWN && refSwitch == HIGH ){
    statusRefPointDrive = REF_STATE_SEARCH; 
  }

  if( statusRefPointDrive == REF_STATE_LIFTOUT_SPACE || statusRefPointDrive == REF_STATE_LIFTOUT){
    motor->setSpeed(LOWSPEED);
    motor->step(1, AUF, DOUBLE);
   }else if (statusRefPointDrive == REF_STATE_SEARCH)  {
    motor->setSpeed(LOWSPEED);
    motor->step(1, AB, DOUBLE);    
   }else{
    motor->setSpeed(0);
    motor->release();
   }   
  
  return false;
}

void resetRefPoint(){
  statusRefPointDrive = REF_STATE_UNKNOWN;
}

void setup() {

  Serial.begin(9600);

  AFMS.begin();
  servo1.attach(10);
  pinMode(pinButtonUp, INPUT_PULLUP);
  pinMode(pinButtonDown, INPUT_PULLUP);
  pinMode(pinButtonSet, INPUT_PULLUP);
  pinMode(pinManualMode, INPUT_PULLUP);
  pinMode(pinReference, INPUT_PULLUP);

  theCatcher.setup();

}

void driveManual() {

  static int manuallyDriven = 0;
  int speed = LOWSPEED;
  if ( manuallyDriven > MANUAL_RAMPE)
    speed = HIGHSPEED;

  if ( digitalRead( pinButtonUp ) == LOW && position < limitOben) {
    motor->setSpeed(speed);
    motor->step(1, AUF, DOUBLE);
    manuallyDriven++;
    position++;
  } else if ( digitalRead( pinButtonDown ) == LOW && position > limitUnten) {
    motor->setSpeed(speed);
    motor->step(1, AB, DOUBLE);
    manuallyDriven++;
    position--;
  } else {
    motor->setSpeed(0);
    motor->release();
    manuallyDriven = 0;
  }

  if ( digitalRead( pinButtonSet ) == LOW) {
    servo1.write(0);
  }else{
    servo1.write(90);
  }
  
}

void logStatus() {

    Serial.print("UP:");
    Serial.print(digitalRead(pinButtonUp));
    Serial.print(" DN:");
    Serial.print(digitalRead(pinButtonDown));
    Serial.print(" ST:");
    Serial.print(digitalRead(pinButtonSet));
    Serial.print(" RF:");
    Serial.print(digitalRead(pinReference));
    Serial.print(" TS:");
    Serial.print(digitalRead(pinTaster));

    if ( digitalRead(pinManualMode) == LOW)
      Serial.print(" Mode:M");
    else
      Serial.print(" Mode:A");

    Serial.print(" Pos:");
    Serial.println(position);
 
}



void updateTimer(){
  static long previousMillis = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= 1000) {
    previousMillis = currentMillis;
    puls1000ms = true;
  }else{
    puls1000ms = false; 
  }

}



  
void loop() {

  updateTimer();
  
  if ( digitalRead( pinManualMode ) == LOW) {
    driveManual();
    theCatcher.drive( digitalRead( pinButtonSet ) == LOW);
    resetRefPoint();
  }else{
    //Automatic
    if( checkRefPoint() ){
        
    }
    
  }
  
 if( puls1000ms)
    logStatus();
}


