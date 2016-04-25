#ifndef turret_h
#define turret_h
#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"

class turret {
  public:
    turret(int pinMotor, int pinServo, int pinSensor, int pinPhoto, int pinFan);
    void turretSetup();
    void zero();
    void sweep(int low, int high);
    void stop();
    void home();
    boolean foundFlame();
    int getPosFlame();
    boolean extinguish();

  private:
    int _pM, _pS, _pFS, _pP, _pF;
    boolean zeroed;
};

#endif
