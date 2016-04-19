#include "turret.h"
#define TURRET_SPEED 80

Servo tM, tS;
Encoder enc(2, 3);
int tSpeed = TURRET_SPEED;

turret::turret(int pinMotor, int pinServo){
  _pM = pinMotor;
  _pS = pinServo;
}

void turret::turretSetup(){
  tM.attach(_pM, 1000, 2000);
  tS.attach(_pS);
  enc.write(0);
}

void turret::sweep(){
  tM.write(tSpeed);
  if(enc.read() < -100 || enc.read() > 0){
    if (enc.read() < 0) tSpeed = 180 - TURRET_SPEED;
    if (enc.read() > 0) tSpeed = TURRET_SPEED;
  }
  Serial.println(enc.read());
}
