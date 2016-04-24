#include "turret.h"
#define TURRET_SPEED 80

Servo tM, tS;
Encoder enc(2, 3);
int tSpeed = TURRET_SPEED;

turret::turret(int pinMotor, int pinServo, int pinFlame, int pinPhoto){
  _pM = pinMotor;
  _pS = pinServo;
  _pF = pinFlame;
  _pP = pinPhoto;
}

void turret::turretSetup(){
  tM.attach(_pM, 1000, 2000);
  tS.attach(_pS);
  pinMode(_pP, INPUT);
  enc.write(0);
}

void turret::zero(){
  while(analogRead(_pP) < 800){
    sweep(-100, 100);
  }
  stop();
  enc.write(0);
}

void turret::sweep(int low, int high){
  tM.write(tSpeed);
  if(enc.read() < low || enc.read() > high){
    if (enc.read() < 0) tSpeed = 180 - TURRET_SPEED;
    if (enc.read() > 0) tSpeed = TURRET_SPEED;
  }
  Serial.println(enc.read());
}

void turret::stop(){
  tM.write(90);
}
