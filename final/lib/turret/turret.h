#ifndef turret_h
#define turret_h
#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"

Servo tM, tS;
int newPos, oldPos;
int tSpeed = 60;
Encoder enc(2, 3);

class turret {
  public:
    turret(int pinMotor, int pinServo);
    void turretSetup();
    void sweep();

  private:
    int _pM, _pS;
};

#endif
