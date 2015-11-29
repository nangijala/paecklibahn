#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>

int oben=2688;
int unte=0;

const int pinReserve = 3;

const int pinButtonDown = 4; // gruen
const int pinButtonSet = 5; // blau

const int pinReference = 6; // weiss:refernzfahrt
const int pinTaster = 7;
const int pinButtonUp = 8; // rot
const int pinManualMode = 11; // gelb: Kipp-Schalter

Servo servo1;

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

}

void driveManual() {

  static int manuallyDriven = 0;
  int speed = LOWSPEED;
  if ( manuallyDriven > MANUAL_RAMPE)
    speed = HIGHSPEED;

  if ( digitalRead( pinButtonUp ) == LOW) {
    motor->setSpeed(speed);
    motor->step(1, AUF, DOUBLE);
    manuallyDriven++;
    position++;
  } else if ( digitalRead( pinButtonDown ) == LOW) {
    motor->setSpeed(speed);
    motor->step(1, AB, DOUBLE);
    manuallyDriven++;
    position--;
  } else {
    motor->setSpeed(0);
    manuallyDriven = 0;
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
    resetRefPoint();
  }else{
    //Automatic
    if( checkRefPoint() ){
        
    }
    
  }
  
  if( puls1000ms)
    logStatus();
}


