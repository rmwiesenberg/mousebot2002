#include <Arduino.h>
#include <Servo.h>
#include "final.h"

#define RIGHT_MOTOR_PIN 4
#define LEFT_MOTOR_PIN 5
#define TURRET_MOTOR_PIN 6
#define TURRET_SERVO_PIN 7

#define MOTOR_MAX_FRW 180
#define MOTOR_STOP 90
#define MOTOR_MAX_REV 0

Servo rightMotor, leftMotor, turretMotor, turretServo;

enum globalState{
  INIT, FIND_FLAME, TO_FLAME, EXTINGUISH, FIND_WALL, GO_HOME, STOP
};

enum wallState{
  RIGHT_FOLLOW, LEFT_FOLLOW, CORNER, WALL_END
};

globalState gState = FIND_FLAME;
wallState wState;

void setup() {
  rightMotor.attach(RIGHT_MOTOR_PIN, 1000, 2000);
  leftMotor.attach(LEFT_MOTOR_PIN, 1000, 2000);
  turretMotor.attach(TURRET_MOTOR_PIN, 1000, 2000);
  turretServo.attach(TURRET_SERVO_PIN);
}

void loop() {
  switch (gState) {
    case INIT:

    break;

    case FIND_FLAME:

    break;

    case TO_FLAME:
    break;

    case EXTINGUISH:
    break;

    case FIND_WALL:
    break;

    case GO_HOME:
    break;

    case STOP:
    break;
  }
}
