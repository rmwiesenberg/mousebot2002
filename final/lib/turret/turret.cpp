#include "turret.h"
#define TURRET_SPEED 78
#define TURRET_HOME 0
#define SERVO_HOME 100
#define FOUND_FLAME 500
#define NO_FIRE 1000

Servo tM, tS;
Encoder enc(2, 28);
int tSpeed = TURRET_SPEED;
boolean zeroed = false;
int posFlame = -900;

turret::turret(int pinMotor, int pinServo, int pinFlameSensor, int pinPhoto, int pinFan){
  _pM = pinMotor;
  _pS = pinServo;
  _pFS = pinFlameSensor;
  _pP = pinPhoto;
  _pF = pinFan;
}

//initiates the proper pins and servos for the turret
void turret::turretSetup(){
  tM.attach(_pM, 1000, 2000);
  tS.attach(_pS);
  pinMode(_pP, INPUT);
  pinMode(_pFS, INPUT);
  pinMode(_pF, OUTPUT);
  enc.write(TURRET_HOME);
}

//calculates the correct home position
//for the turret using a photoresistor
void turret::zero(){
  while(analogRead(_pP) < 700 && !zeroed){
    sweep(-90, 90);
  }
  zeroed = true;
  stop();
  tS.write(SERVO_HOME);
  enc.write(TURRET_HOME);
}

//varries the position of the turret between the given
// high and low position
void turret::sweep(int low, int high){
  if(enc.read() <= low || enc.read() >= high){
    if (enc.read() < TURRET_HOME) tSpeed = 180 - TURRET_SPEED;
    if (enc.read() > TURRET_HOME) tSpeed = TURRET_SPEED;
  }
  tM.write(tSpeed);
}

//stops the motion of the turret
void turret::stop(){
  tM.write(90);
}

//returns the turret to home position
boolean turret::home(){
  if (enc.read() < TURRET_HOME){
    tSpeed = 180 - TURRET_SPEED;
    tM.write(tSpeed);
  }
  if (enc.read() > TURRET_HOME){
    tSpeed = TURRET_SPEED;
    tM.write(tSpeed);
  }
  if (enc.read() == TURRET_HOME) {
    stop();
    return true;
  } else return false;
}

//send the turret to the input degree
boolean turret::go(int deg){
  if (enc.read() < deg){
    tSpeed = 180 - TURRET_SPEED;
    tM.write(tSpeed);
  }
  if (enc.read() > deg){
    tSpeed = TURRET_SPEED;
    tM.write(tSpeed);
  }
  if (enc.read() == deg) {
    stop();
    return true;
  } else return false;
}

//checks to see if the IR sensor has found a flame
boolean turret::foundFlame(){
  if(analogRead(_pFS) < FOUND_FLAME) {
    posFlame = enc.read();
    stop();
    return true;
  }
  else return false;
}

//retrieves the encoder position for the flame
int turret::getPosFlame(){
  return posFlame;
}

//turns on the fan to put out the flame if
//the IR sensor sees a flame
boolean turret::extinguish(){
  if(analogRead(_pFS) < NO_FIRE){
    digitalWrite(_pF, HIGH);
    return false;
  } else return true;
}

//getter for the current turret encoder position
long turret::getTurretEncPos(){
  return enc.read();
}
