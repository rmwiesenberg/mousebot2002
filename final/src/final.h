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
#define MIN_FRONT_DIST 10
#define MAX_DIST 30
#define SAMPLE_TIME 10
#define MIN_SPEED 20
#define MAX_PID_SPEED 50
#define REG_SPEED 25
#define TURN_SPEED 25

// Variables for PID
double pidInput, pidOutput, pidSetpoint;
const double Kp = 40, Ki = 10, Kd = 10;
const double pidError = .25;

// PID Declarations
PID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

// Turret
#define ENCODER1_PIN 2
#define ENCODER2_PIN 28
#define FLAME_PIN A10
#define PHOTO_PIN A11
#define FAN_PIN 30

turret extinguisher(TURRET_MOTOR_PIN, TURRET_SERVO_PIN, FLAME_PIN, PHOTO_PIN, FAN_PIN);

// Mice
#define LEFT_MOUSE_CLOCK 30
#define LEFT_MOUSE_DATA 28
#define RIGHT_MOUSE_CLOCK 31
#define RIGHT_MOUSE_DATA 29
#define POSITION_ERROR 4
const double turnConst = .65;
const double driveConst = 5.5;
const double cdist = 5;

//PS2 leftMouse(LEFT_MOUSE_CLOCK, LEFT_MOUSE_DATA);
//PS2 rightMouse(RIGHT_MOUSE_CLOCK, RIGHT_MOUSE_DATA);

char lstat, lx, ly, rstat, rx, ry;

double ld, rd;
double tdist;
double trDist, tlDist;
double tx, ty;
double cx, cy;
double heading;
double deg;

//Drive Encoders
#define L_ENCODER1_PIN 18
#define L_ENCODER2_PIN 3
#define R_ENCODER1_PIN 20
#define R_ENCODER2_PIN 19
#define WHEEL_DIST 9
const double encMaxVal = 360;
const double circum = 11;
const double ratio = 1.6667; //36:60

Encoder rdenc(R_ENCODER1_PIN,R_ENCODER2_PIN);
Encoder ldenc(L_ENCODER1_PIN,L_ENCODER2_PIN);

long rEncVal, lEncVal;
long oldrEncVal, oldlEncVal;
double rDist, lDist;

// LCD Declaration
LiquidCrystal lcd(40, 41, 42, 43, 44, 45);

// UNO Stuff
#define UNO_PIN1 33

// DedDrive
unsigned long t1 = 24500;
unsigned long t2 = t1 + 3000;
unsigned long t3 = t2 + 10000;
unsigned long st2;
unsigned long t4 = 4500;
unsigned long t5 = t4 + 12000;
unsigned long t6 = t5 + 2500;
unsigned long t7 = t6 + 24500;

// States for General Robot Running
enum robotState{
  INIT,  // Initialization - Find Walls
  FIND_FLAME, // Wall Follow to FIND_FLAME
  TURN_FLAME,
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

enum motion{
  START,
  MOVING,
  END
};

motion turning = START;
motion driving = START;
robotState rState = INIT; //start global state as INIT
wallState wState; // Delcare variable for wall following state
closeWall cWall = DEBUG;

// Function Declarations
void regDrive(int speed);
void turnRight();
void turnLeft();
void pingWall(void);
void lcdPrintWallDist(void);
void lcdPrintTravelDist(void);
void lcdPrintEncVals(void);
void lcdCandle(void);
void findWall(void);
void wallSwitch(void);
void wallFollow(void);
void turnCorner(void);
void turnEnd(void);
boolean turnDeg(double degTurn);
float getDeg(void);
void driveDist(double dist);
void updatePos(void);
void distDriven(void);
void setCandle(void);
boolean home(void);
void driveThere();
void driveHome();
