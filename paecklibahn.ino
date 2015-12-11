#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Servo.h>
#include <Timer.h>

int limitOben=2691;
int limitUnten=13;

#define MOVES_BEFORE_REF 15
#define TIMEOUT_DURATION 8000

// Debug level logging
#define VERBOSE false

// Application loggin 
#define STATUSLOGGING false

const int pinRes1 = 1;
const int pinRes2 = 2;
const int pinGateRef = 3;    // Position of catcher
const int pinButtonDown = 4; // Green button
const int pinButtonSet = 5; // Blue button
const int pinReference = 6; // weiss:refernzfahrt
const int pinTaster = 7; // Checks load
const int pinButtonUp = 8; // Red button
const int pinServoCatcher = 9;
const int pinServoPusher = 10;
const int pinManualMode = 11; // gelb: Kipp-Schalter

Timer timer;
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
     if( VERBOSE ){
       if( direction == SERVO_FORW )
         Serial.println("Open SERVO_FORW");
        else
         Serial.println("Open SERVO_BACK");
     }
    }else if( m - previousMillisGate >= MOVING_TIME || (digitalRead(_refPin) == LOW && direction == SERVO_BACK)){
     if( VERBOSE )  Serial.println( m - previousMillisGate );
     if( direction == SERVO_FORW ){
       isOpen = true;
       if( VERBOSE ) Serial.println("Gate open");
     }else{
       isOpen = false;
       if( VERBOSE ) Serial.println("Gate closed");
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

};

Catcher theCatcher(pinServoCatcher,pinGateRef);



#define AB FORWARD
#define AUF BACKWARD
#define DONTMOVE RELEASE

#define HIGHSPEED 800
#define LOWSPEED 100



Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *motor = AFMS.getStepper(200, 2);

int position = 0;


void terminateLiftOut();

class RefPointSearch{

public:
  int state = RefPointSearch::STATE_UNKNOWN;
  
  RefPointSearch(int pin){
    _pin = pin;
  }

  void reset()
  {
    state = STATE_UNKNOWN;
    if( isMoving)
      moveLift();
  }
  
  void handeLiftOutTimer()
  {
    timerId = -1;
    if( state == STATE_LIFTOUT_SPACE )
      state = STATE_SEARCH;
  }
  
  bool checkState()
  {
    refSwitchTouched = digitalRead(_pin) == LOW ? true : false;
    if( state == STATE_UNKNOWN ){
      if( refSwitchTouched)
        state = STATE_LIFTOUT;
       else
        state = STATE_SEARCH;
    }else if( state == STATE_LIFTOUT){
      if( refSwitchTouched ){
        state = STATE_LIFTOUT_SPACE;
        timerId = timer.after( 1500, terminateLiftOut);
      }
    }else if( state == STATE_LIFTOUT_SPACE){
      // Do nothing, wait for timer
    }else if( state == STATE_SEARCH){
      if( refSwitchTouched){
        position = 0;
        state = STATE_GOTO_LOAD;
      }
    }else if(state == STATE_GOTO_LOAD){
      if( position >= limitUnten)
        state = STATE_DONE;
    }else if( state == STATE_DONE){
      return true;
    }
    moveLift();
    return state == STATE_DONE ? true : false;
  }


  
protected:

  void moveLift()
  {

    isMoving = true;    
    if( state == STATE_LIFTOUT_SPACE || state == STATE_LIFTOUT || state == STATE_GOTO_LOAD){
      motor->setSpeed(LOWSPEED);
      motor->step(1, AUF, DOUBLE);      
      if( state == STATE_GOTO_LOAD)
        position ++;
     }else if (state == STATE_SEARCH)  {
      motor->setSpeed(LOWSPEED);
      motor->step(1, AB, DOUBLE);    
     }else{
      motor->setSpeed(0);
      motor->release();
      isMoving = false;
     }      
  }

  int _pin;
  bool refSwitchTouched = false;
  bool isMoving = false;
  int timerId = -1;
  
  const int STATE_UNKNOWN = 0;
  const int STATE_LIFTOUT = 1;
  const int STATE_LIFTOUT_SPACE = 2;
  const int STATE_SEARCH = 3;
  const int STATE_GOTO_LOAD = 4;
  const int STATE_DONE = 5;
  
};



RefPointSearch refPoint( pinReference );

void terminateLiftOut(){
  refPoint.handeLiftOutTimer();
}



class Pusher
{
  public:

  Pusher(int pin)
  {
    _pinNr = pin; 
  }



  void drive( bool shouldKick ){
    if ( shouldKick ) {

      servo1.write(0);
    }else if(!shouldKick){
      servo1.write(90);
    }    
  }

  
  void driveManual()
  {

    int buttonStatus = digitalRead( _pinNr );
    if ( buttonStatus == LOW) {
      servo1.write(0);

    }else if(buttonStatus == HIGH   ){
      // button not pressed           
      servo1.write(90);
    }
      
  }


  protected:
  int _pinNr;
};


Pusher topPusher( pinButtonSet );



#define MANUAL_RAMPE 200


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
  topPusher.driveManual();
  
}


#define AUTO_RAMPE 50

#define AUT_STATE_UNKNOWN 0
#define AUT_STATE_LOWER 1
#define AUT_STATE_LOAD 2
#define AUT_STATE_WAIT_FOR_LOAD 3
#define AUT_STATE_RAISE 10
#define AUT_STATE_UNLOAD 11
#define AUT_STATE_WAIT_FOR_UNLOAD 12
#define AUT_STATE_MAKE_TIMEOUT 20
#define AUT_STATE_REF_POINT 30
#define AUT_STATE_WAIT 40
#define AUT_STATE_ERROR 99


class AutoPilot{
  public:
  int state=AUT_STATE_UNKNOWN;
  int oldState = -1;
  int moveCounter = 0;
  
  AutoPilot(int sensorPin, Catcher *theCatcher, Pusher *thePusher)
  {
    state = AUT_STATE_UNKNOWN;
    _sensorPin = sensorPin;
    _theCatcher = theCatcher;
    _thePusher = thePusher;
  }
  
  void updateFromTimer()
  {
    if( state == AUT_STATE_WAIT_FOR_LOAD)
    {
      if( TIMEOUT_DURATION <= 0 || timeOutEnabled == false){
        state = AUT_STATE_RAISE;
      }else{
        timer.after(TIMEOUT_DURATION,updatePilotFromTimer);
        state = AUT_STATE_MAKE_TIMEOUT;
       }
    }else if( state == AUT_STATE_MAKE_TIMEOUT ){
      state = AUT_STATE_RAISE;      
    }else if( state == AUT_STATE_WAIT_FOR_UNLOAD){
      shouldPusherKick = false;
      state = AUT_STATE_LOWER;
    }else if( state == AUT_STATE_REF_POINT){
      refPoint.reset();
      drive(DONTMOVE);
      state = AUT_STATE_WAIT;
    }
  }
  
  void updateManual()
  {
    if( timerId != -1){
      timer.stop(timerId);
      timerId = -1;
    }
    shouldPusherKick = false;
    state = AUT_STATE_UNKNOWN;
    moveCounter = 0;
    refDriveRequested = false;
  }
  
  void update()
  {
    oldState = state;
    isLoaded = digitalRead(_sensorPin) == LOW ? true : false;
    if( state == AUT_STATE_UNKNOWN )
      state = findFirstStep();
      
    switch( state ){
      case AUT_STATE_LOWER:
        stepLower();
      break;

      case AUT_STATE_LOAD:
        stepLoad();
      break;

      case AUT_STATE_WAIT_FOR_LOAD:
        stepWaitForLoad();
      break;

      case AUT_STATE_RAISE:
        stepRaise();
      break;
            
      case AUT_STATE_UNLOAD:
        stepUnLoad();
      break;
      
      case AUT_STATE_WAIT_FOR_UNLOAD:
        stepWaitForUnLoad();
      break;  

      case AUT_STATE_REF_POINT:
        stepPrepareRefPoint();
       break;
       
      default:
      ;
    };
    if( oldState != state && VERBOSE){
      Serial.print( "State Changed: "); 
      Serial.print( oldState );
      Serial.print( " -> " );
      Serial.println( state );
     }
    _theCatcher->drive(shouldCatcherOpen);
    _thePusher->drive(shouldPusherKick);

  }

protected:

  int findFirstStep(){
    if( isLoaded == true)
      return AUT_STATE_RAISE;
    else{
      return AUT_STATE_LOWER;
    }
   }


   void stepLower()
   {
      if( drive(AB) == true){
        moveCounter++;
        if( moveCounter >= MOVES_BEFORE_REF || refDriveRequested == true){
          refDriveRequested = false;
          timer.after(1500,updatePilotFromTimer);
          state = AUT_STATE_REF_POINT;
        }else
          state = AUT_STATE_LOAD;
      }
   }

  void stepPrepareRefPoint()
  {
    if( drive(AUF) == true)
      state = AUT_STATE_ERROR;
  }
   
  void stepLoad()
  {
    shouldCatcherOpen = true;
    if( isLoaded )
    {
      timer.after(100,updatePilotFromTimer);
      state = AUT_STATE_WAIT_FOR_LOAD;
    }
  }
  
  void stepWaitForLoad()
  {
    shouldCatcherOpen = false;
  }
  
  void stepRaise()
  {
    if( drive(AUF) == true)
      state = AUT_STATE_UNLOAD;
  }

  void stepUnLoad()
  {
    shouldPusherKick = true;
    if( isLoaded == false )
    {
      timerId = timer.after(1000,updatePilotFromTimer);
      state = AUT_STATE_WAIT_FOR_UNLOAD;
    }    
  }



  void stepWaitForUnLoad()
  {
    if( isLoaded == true){
        if( timerId != -1){
          timer.stop(timerId);
          timerId = -1;
        }      
       state = AUT_STATE_UNLOAD;
    }
  }

  
  boolean drive( int direction) {
  
    int speed = LOWSPEED;
    if ( manuallyDriven > AUTO_RAMPE)
      speed = HIGHSPEED;
  
    if ( direction == AUF && position < limitOben) {
      motor->setSpeed(speed);
      motor->step(1, AUF, DOUBLE);
      manuallyDriven++;
      position++;
    } else if ( direction == AB && position > limitUnten) {
      motor->setSpeed(speed);
      motor->step(1, AB, DOUBLE);
      manuallyDriven++;
      position--;
    } else {
      motor->setSpeed(0);
      motor->release();
      manuallyDriven = 0;
      if(( direction == AUF && position >= limitOben) || ( direction == AB && position <= limitUnten) )
        return true;      
    }

    return false;
   }

  

  int _sensorPin;
  int timerId = -1;
  Catcher *_theCatcher;
  Pusher* _thePusher;
  
  boolean isLoaded = false;
  int manuallyDriven = 0;
  boolean shouldCatcherOpen = false;
  boolean shouldPusherKick = false;

public:
  boolean refDriveRequested = false;
  boolean timeOutEnabled = false;

};


AutoPilot pilot( pinTaster, &theCatcher, &topPusher);

void updatePilotFromTimer()
{
  pilot.updateFromTimer();
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
    Serial.print(" RP:");
    Serial.print(refPoint.state);    
    if ( digitalRead(pinManualMode) == LOW)
      Serial.print(" Mode:M");
    else
      Serial.print(" Mode:A");
    Serial.print(" AP:");
    Serial.print(pilot.state);
    
    Serial.print(" MV:");
    Serial.print(pilot.moveCounter);    

    Serial.print(" Pos:");
    Serial.println(position);
 
}

void setup() {

  Serial.begin(9600);

  AFMS.begin();
  servo1.attach(pinServoPusher);
  pinMode(pinButtonUp, INPUT_PULLUP);
  pinMode(pinButtonDown, INPUT_PULLUP);
  pinMode(pinButtonSet, INPUT_PULLUP);
  pinMode(pinManualMode, INPUT_PULLUP);
  pinMode(pinReference, INPUT_PULLUP);
  pinMode(pinTaster, INPUT_PULLUP);
  if( STATUSLOGGING )
    timer.every(1000, logStatus);
  theCatcher.setup();

}

  
void loop() {
  timer.update();
  
  if ( digitalRead( pinManualMode ) == LOW) {
    driveManual();
    theCatcher.drive( digitalRead( pinButtonSet ) == LOW);
    refPoint.reset();
    pilot.updateManual();
  }else{
    //Automatic
    if( refPoint.checkState() ){
      pilot.update();  
      if( digitalRead(pinButtonDown) == LOW)
        pilot.timeOutEnabled = false;
      if( digitalRead(pinButtonUp) == LOW)
        pilot.timeOutEnabled = true;
      if( digitalRead(pinButtonSet) == LOW)
        pilot.refDriveRequested = true;
    }else{
      pilot.updateManual();   
    }
  } 
}


