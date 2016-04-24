#ifndef turret_h
#define turret_h
#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"

class turret {
  public:
    turret(int pinMotor, int pinServo, int pinFlame, int pinPhoto);
    void turretSetup();
    void zero();
    void sweep(int low, int high);
    void stop();

  private:
    int _pM, _pS, _pF, _pP;
};

#endif
