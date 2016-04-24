#include "turret.h"
#define TURRET_SPEED 79
#define TURRET_HOME 0
#define SERVO_HOME 100
#define FOUND_FLAME 800

Servo tM, tS;
Encoder enc(2, 3);
int tSpeed = TURRET_SPEED;
boolean zeroed = false;

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
  tM.write(tSpeed);
  if(enc.read() < low || enc.read() > high){
    if (enc.read() < TURRET_HOME) tSpeed = 180 - TURRET_SPEED;
    if (enc.read() > TURRET_HOME) tSpeed = TURRET_SPEED;
  }
  Serial.println(enc.read());
}

void turret::stop(){
  tM.write(90);
}

void turret::home(){
  if (enc.read() < TURRET_HOME){
    tSpeed = 180 - TURRET_SPEED;
    tM.write(tSpeed);
  }
  if (enc.read() > TURRET_HOME){
    tSpeed = TURRET_SPEED;
    tM.write(tSpeed);
  }
  if (enc.read() == TURRET_HOME) stop();
}

boolean turret::foundFlame(){
  if(analogRead(_pFS) < FOUND_FLAME) return true;
  else return false;
}
