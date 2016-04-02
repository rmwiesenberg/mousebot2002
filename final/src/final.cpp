#include <Arduino.h>
#include <Servo.h>
#include "NewPing.cpp"
#include "ps2.cpp"
#include <LiquidCrystal.h>

// Start Definitions
// Pin Definitions
#define RIGHT_MOTOR_PIN 4
#define LEFT_MOTOR_PIN 5
#define TURRET_MOTOR_PIN 6
#define TURRET_SERVO_PIN 7

// Motor Declarations
Servo rightMotor, leftMotor, turretMotor, turretServo;

// Motor Speed Definitions
#define MOTOR_MAX_FRW 180
#define MOTOR_STOP 90
#define MOTOR_MAX_REV 0

// Ultrasonic Range Finder Pins
#define RIGHT_URF_TRIG 24
#define MID_URF_TRIG 23
#define LEFT_URF_TRIG 22
#define RIGHT_URF_ECHO 20
#define MID_URF_ECHO 19
#define LEFT_URF_ECHO 2

// URF Declarations
NewPing rightURF(RIGHT_URF_TRIG, RIGHT_URF_ECHO);
NewPing midURF(MID_URF_TRIG, MID_URF_ECHO);
NewPing leftURF(LEFT_URF_TRIG, LEFT_URF_ECHO);

// Variables for URFs
double rightDist, midDist, leftDist;


// States for General Robot Running
enum robotState{
  INIT,  // Initialization - Find Walls
  FIND_FLAME, // Wall Follow to FIND_FLAME
  TO_FLAME, // After Found Flame. Drive to it
  EXTINGUISH, // Put out Flame
  FIND_WALL, // Refind Wall
  GO_HOME, // Wall Follow Home
  STOP // Stop Robot
};

enum wallState{
  RIGHT_FOLLOW, // Follow wall if on left side
  LEFT_FOLLOW, // Follow wall if on right side
  CORNER, // Turn through corner
  WALL_END // Turn around wall end
};

robotState rState = INIT; //start global state as INIT
wallState wState; // Delcare variable for wall following state

// Function Declarations
void wallFollow(void);
void pingWall(void);
void closeWall(void);

void setup() {
  // Serial port for debugging
  Serial.begin(9600);
  Serial.println("Start");

  // Attach Motors
  rightMotor.attach(RIGHT_MOTOR_PIN, 1000, 2000);
  leftMotor.attach(LEFT_MOTOR_PIN, 1000, 2000);
  turretMotor.attach(TURRET_MOTOR_PIN, 1000, 2000);
  turretServo.attach(TURRET_SERVO_PIN);
}

void loop(void) {
  switch (rState) {
    case INIT:
      pingWall();
      closeWall();
      rState = FIND_FLAME;
    break;

    case FIND_FLAME:
      wallFollow();
    break;

    case TO_FLAME:
    break;

    case EXTINGUISH:
    break;

    case FIND_WALL:
      pingWall();
      closeWall();
      rState = GO_HOME;
    break;

    case GO_HOME:
      wallFollow();
    break;

    case STOP:
    break;
  }
}

void wallFollow(void){
  pingWall();

  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.print("  Mid: ");
  Serial.print(midDist);
  Serial.print("  Right: ");
  Serial.println(rightDist);

  delay(100);
}

// determine distances to each wall
void pingWall(void){
  rightDist = rightURF.ping_in();
  midDist = midURF.ping_in();
  leftDist = leftURF.ping_in();
}

void closeWall(){

}
