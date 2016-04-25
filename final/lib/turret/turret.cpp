#include "turret.h"
#define TURRET_SPEED 78
#define TURRET_HOME 0
#define SERVO_HOME 100
#define FOUND_FLAME 500
#define NO_FIRE 1000

Servo tM, tS;
Encoder enc(2, 3);
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

void turret::turretSetup(){
  tM.attach(_pM, 1000, 2000);
  tS.attach(_pS);
  pinMode(_pP, INPUT);
  pinMode(_pFS, INPUT);
  pinMode(_pF, OUTPUT);
  enc.write(TURRET_HOME);
}

void turret::zero(){
  while(analogRead(_pP) < 800 && !zeroed){
    sweep(-100, 100);
  }
  zeroed = true;
  stop();
  tS.write(SERVO_HOME);
  enc.write(TURRET_HOME);
}

void turret::sweep(int low, int high){
  if(enc.read() <= low || enc.read() >= high){
    if (enc.read() < TURRET_HOME) tSpeed = 180 - TURRET_SPEED;
    if (enc.read() > TURRET_HOME) tSpeed = TURRET_SPEED;
  }
  tM.write(tSpeed);
}

void turret::stop(){
  tM.write(90);
}

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

boolean turret::foundFlame(){
  if(analogRead(_pFS) < FOUND_FLAME) {
    posFlame = enc.read();
    stop();
    return true;
  }
  else return false;
}

int turret::getPosFlame(){
  return posFlame;
}

boolean turret::extinguish(){
  if(analogRead(_pFS) < NO_FIRE){
    digitalWrite(_pF, HIGH);
    return false;
  } else return true;
}
