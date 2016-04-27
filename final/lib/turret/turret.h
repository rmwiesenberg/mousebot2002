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
    void updown();
    void stop();
    boolean home();
    boolean go(int deg);
    boolean foundFlame();
    int getPosFlame();
    boolean extinguish();
    long getTurretEncPos();

  private:
    int _pM, _pS, _pFS, _pP, _pF;
    boolean zeroed;
};

#endif
