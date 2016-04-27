#include "turret.h"
#define TURRET_SPEED 78
#define TURRET_HOME 0
#define SERVO_HOME 100
#define SERVO_TOP 180
#define FOUND_FLAME 400
#define NO_FIRE 800
#define FLAME_WAIT 20
#define RECHECK 200

Servo tM, tS;
Encoder enc(2, 28);
int tSpeed = TURRET_SPEED;
boolean zeroed = false;
int posFlame = -900;
unsigned long nextUpDown;
unsigned long nextCheck;
int curPos = SERVO_HOME;

turret::turret(int pinMotor, int pinServo, int pinFlameSensor, int pinPhoto, int pinFan){
  _pM = pinMotor;
  _pS = pinServo;
  _pFS = pinFlameSensor;
  _pP = pinPhoto;
  _pF = pinFan;
}

//initiates the proper pins and servos for the turret
void turret::turretSetup(){
  tM.attach(_pM, 1000, 2000);
  tS.attach(_pS);
  pinMode(_pP, INPUT);
  pinMode(_pFS, INPUT);
  pinMode(_pF, OUTPUT);
  digitalWrite(_pF, HIGH);
  enc.write(TURRET_HOME);
}

//calculates the correct home position
//for the turret using a photoresistor
void turret::zero(){
  while(analogRead(_pP) < 750 && !zeroed){
    sweep(-90, 90);
  }
  zeroed = true;
  stop();
  tS.write(SERVO_HOME);
  enc.write(TURRET_HOME);
}

//varries the position of the turret between the given
// high and low position
void turret::sweep(int low, int high){
  if(enc.read() <= low || enc.read() >= high){
    if (enc.read() < low) tSpeed = 180 - TURRET_SPEED;
    if (enc.read() > high) tSpeed = TURRET_SPEED;
  }
  tM.write(tSpeed);
}

void turret::updown(){
  if(millis() > nextUpDown){
    if(curPos == SERVO_HOME) curPos = SERVO_TOP;
    if(curPos == SERVO_TOP) curPos = SERVO_HOME;
    nextUpDown = millis() + FLAME_WAIT;
  }
  tS.write(curPos);
}

//stops the motion of the turret
void turret::stop(){
  tM.write(90);
}

//returns the turret to home position
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

//send the turret to the input degree
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

//checks to see if the IR sensor has found a flame
boolean turret::foundFlame(){
  int fintent = analogRead(_pFS);
  if(fintent < FOUND_FLAME) {
    posFlame = enc.read();
    stop();

    return true;
  }
  else return false;
}

//retrieves the encoder position for the flame
int turret::getPosFlame(){
  return posFlame;
}

int turret::getFlame(){
  return analogRead(_pFS);
}

//turns on the fan to put out the flame if
//the IR sensor sees a flame
boolean turret::extinguish(){
  for(unsigned long check = millis() + RECHECK; millis() < check; millis()){
    sweep(posFlame-5, posFlame+5);
    updown();
    digitalWrite(_pF, LOW);
  }
  delay(10);

  if(analogRead(_pFS) < NO_FIRE) {
    stop();
    digitalWrite(_pF, HIGH);
    return true;
  }
  else extinguish();
}

//getter for the current turret encoder position
long turret::getTurretEncPos(){
  return enc.read();
}
