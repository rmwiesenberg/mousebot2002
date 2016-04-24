#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "NewPing.h"
#include "ps2.h"
#include "PID_v1.h"
#include "LSM303.h"
#include <Wire.h>
#include "turret.h"

// Start Definitions
// Pin Definitions
#define RIGHT_MOTOR_PIN 4
#define LEFT_MOTOR_PIN 5
#define TURRET_MOTOR_PIN 6
#define TURRET_SERVO_PIN 7

// Motor Declarations
Servo rightMotor, leftMotor;
int rightSpeed, leftSpeed;

// Motor Speed Definitions
#define MOTOR_MAX_FRW 180
#define MOTOR_STOP 90
#define MOTOR_MAX_REV 0

// Ultrasonic Range Finder Pins
#define RIGHT_URF_TRIG 27
#define MID_URF_TRIG 25
#define LEFT_URF_TRIG 23
#define RIGHT_URF_ECHO 26
#define MID_URF_ECHO 24
#define LEFT_URF_ECHO 22

// URF Declarations
NewPing rightURF(RIGHT_URF_TRIG, RIGHT_URF_ECHO);
NewPing midURF(MID_URF_TRIG, MID_URF_ECHO);
NewPing leftURF(LEFT_URF_TRIG, LEFT_URF_ECHO);

// Variables for URFs
double rightDist, midDist, leftDist;

// PID BASE THINGS
#define TARGET_DIST 8
#define MIN_FRONT_DIST 8
#define MAX_DIST 30
#define SAMPLE_TIME 10
#define MIN_SPEED 30
#define REG_SPEED 30

// Variables for PID
double pidInput, pidOutput, pidSetpoint;
const double Kp = 400, Ki = 0, Kd = 0;
const double pidError = .25;

// PID Declarations
PID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Turret
#define ENCODER1_PIN 2
#define ENCODER2_PIN 3
#define FLAME_PIN A10
#define PHOTO_PIN A11
#define FAN_PIN 30

turret extinguisher(TURRET_MOTOR_PIN, TURRET_SERVO_PIN, FLAME_PIN, PHOTO_PIN, FAN_PIN);

// LCD Declaration
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);

// UNO Stuff
#define UNO_PIN1 33

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
  FOLLOW, // Follow wall
  CORNER, // Turn through corner
  WALL_END // Turn around wall end
};

enum closeWall{
  DEBUG,
  RIGHT,
  LEFT,
  ZERO
};

robotState rState = INIT; //start global state as INIT
wallState wState; // Delcare variable for wall following state
closeWall cWall = DEBUG;

// Function Declarations
void regDrive(int speed);
void pingWall(void);
void lcdPrintWallDist(void);
void findWall(void);
void wallSwitch(void);
void wallFollow(void);
void turnCorner(void);
void turnEnd(void);
