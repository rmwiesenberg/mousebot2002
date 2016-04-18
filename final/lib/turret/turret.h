#ifndef turret_h
#define turret_h
#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"

Servo tM, tS;
int newPos, oldPos;
Encoder enc(2, 3);

class turret {
  public:
    turret(int pinMotor, int pinServo);

  private:
    int _pM, _pS;
    void turretSetup();
    void sweep();
};

#endif
