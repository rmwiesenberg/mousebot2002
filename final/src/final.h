#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "NewPing.h"
#include "ps2.h"
#include "PID_v1.h"
#include "LSM303.h"
#include <Wire.h>

// Start Definitions
// Pin Definitions
#define RIGHT_MOTOR_PIN 4
#define LEFT_MOTOR_PIN 5
#define TURRET_MOTOR_PIN 6
#define TURRET_SERVO_PIN 7

// Motor Declarations
Servo rightMotor, leftMotor, turretMotor, turretServo;
int rightSpeed, leftSpeed;

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

// PID BASE THINGS
#define TARGET_DIST 8
#define SAMPLE_TIME 10
#define MIN_SPEED 30

// Variables for PID
double pidInput, pidOutput, pidSetpoint;
const double Kp = 2, Ki = .1, Kd = .1;
const double pidError = 1;

// PID Declarations
PID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// LCD Declaration
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);

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
