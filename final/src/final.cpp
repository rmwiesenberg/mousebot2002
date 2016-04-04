#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include "NewPing.cpp"
#include "ps2.cpp"
#include "PID_v1.cpp"

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

// PID
#define TARGET_DIST 8
#define SAMPLE_TIME 10

double pidInput, pidOutput, pidSetpoint;
double Kp = 2, Ki = 1, Kd = 1;
const double pidError = .25;

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

robotState rState = INIT; //start global state as INIT
wallState wState; // Delcare variable for wall following state

// Function Declarations
void regDrive(int speed);
void pidDrive(int speed);
void pingWall(void);
void lcdPrintWallDist(void);
char closeWall(void);
void wallSwitch(void);
void wallFollow(char wall);

void setup() {
  // Serial port for debugging
  Serial.begin(9600);

  // Attach Motors
  rightMotor.attach(RIGHT_MOTOR_PIN, 1000, 2000);
  leftMotor.attach(LEFT_MOTOR_PIN, 1000, 2000);
  turretMotor.attach(TURRET_MOTOR_PIN, 1000, 2000);
  turretServo.attach(TURRET_SERVO_PIN);

  // PID setup
  pidSetpoint = TARGET_DIST; // sets dist from wall
  pid.SetOutputLimits(MOTOR_MAX_REV, MOTOR_STOP); // sets limits
  pid.SetSampleTime(SAMPLE_TIME);
  pid.SetMode(AUTOMATIC); // turns PID on

  // LCD Start
  lcd.begin(16,2);
}

void loop(void) {
  switch (rState) {
    case INIT:
    pingWall();
    rState = FIND_FLAME;
    wState = FOLLOW;
    break;

    case FIND_FLAME:
    pingWall();
    lcdPrintWallDist();
    wallSwitch();
    break;

    case TO_FLAME:
    break;

    case EXTINGUISH:
    break;

    case FIND_WALL:
    pingWall();
    rState = GO_HOME;
    break;

    case GO_HOME:
    wallSwitch();
    break;

    case STOP:
    break;
  }
}

// Drive Robot
void regDrive(int speed){
  leftMotor.write(MOTOR_MAX_FRW-speed);
  rightMotor.write(speed);
}

// Drive Robot for PID
void pidDrive(int speed){
  leftMotor.write(MOTOR_STOP - speed);
  rightMotor.write(MOTOR_STOP - speed);
}

// Switch between wall following states
void wallSwitch(){
  switch (wState) {
    case FOLLOW:
    wallFollow(closeWall());
    break;
    case CORNER:
    break;
    case WALL_END:
    break;
  }
}

// determine distances to each wall
void pingWall(void){
  rightDist = rightURF.ping_in();
  midDist = midURF.ping_in();
  leftDist = leftURF.ping_in();
}

// print wall distances to the LCD
void lcdPrintWallDist(){
  lcd.setCursor(0,0);
  lcd.print("L: ");
  lcd.print(leftDist);

  lcd.setCursor(7,0);
  lcd.print(" R: ");
  lcd.print(rightDist);

  lcd.setCursor(0,1);
  lcd.print("M: ");
  lcd.print(midDist);

  lcd.setCursor(7,1);
  lcd.print(" W: ");
  lcd.print(closeWall());
}

// determines closest wall - left or right
// used for figuring out which wall to follow
char closeWall(){
  char result;

  if ((rightDist > 0) && (rightDist < leftDist)) result = 'R';
  else if ((leftDist > 0) && (leftDist < rightDist)) result = 'L';
  else if ((rightDist == 0) && (leftDist == 0)) result = 'x';

  return result;
}

// uses PID to follow wall
void wallFollow(char wall){
  pid.Compute();
  if (wall == 'L'){
    if (leftDist <= (TARGET_DIST + pidError) && leftDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_MAX_FRW);
    } else {
      pidInput = leftDist;
      pidDrive((int) pidOutput);
    }
  } else if (wall == 'R'){
    if (rightDist <= (TARGET_DIST + pidError) && rightDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_MAX_FRW);
    } else {
      pidInput = rightDist;
      pidDrive(-((int) pidOutput));
    }
  }
}
