#include "turret.h"

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
  newPos =+ enc.read();
  tM.write(tSpeed);
  if (newPos < 15 && newPos >= 0){
    tM.write(60);
  } else if (newPos > -15 && newPos <= 0){
    tM.write(120);
  } else {
    if (tSpeed == 60) tSpeed = 120;
    else tSpeed = 120;
  }
}
