/***************  RBE2002 Final Code  ***************/
/**************    Team 4: Mousebot   ***************/

/**************      Richard Cole     ***************/
/**************    Elijah Eldredge    ***************/
/**************    Ryan Wiesenberg    ***************/

#include "final.h" // constants or other includes in other file for neatness

void setup() {
  // Serial port for debugging
  Serial.begin(9600);

  // LCD Start
  lcd.begin(16,2);

  // Attach Motors
  rightMotor.attach(RIGHT_MOTOR_PIN, 1000, 2000);
  leftMotor.attach(LEFT_MOTOR_PIN, 1000, 2000);
  turretMotor.attach(TURRET_MOTOR_PIN, 1000, 2000);
  turretServo.attach(TURRET_SERVO_PIN);

  // PID setup
  pidSetpoint = TARGET_DIST; // sets dist from wall
  pid.SetOutputLimits((-MOTOR_STOP + MIN_SPEED), (MOTOR_STOP - MIN_SPEED)); // sets limits
  pid.SetSampleTime(SAMPLE_TIME);
  pid.SetMode(AUTOMATIC); // turns PID on
}

void loop(void) {
  switch (rState) {
    case INIT: // wall distance initializations
    // pingWall();
    rState = FIND_FLAME;
    wState = FOLLOW;
    break;

    case FIND_FLAME: // wall following to find flame
    pingWall();
    lcdPrintWallDist();
    wallSwitch();
    break;

    case TO_FLAME: // moving toward flame
    break;

    case EXTINGUISH: // extinguishing flame
    break;

    case FIND_WALL: // re-find and drive to wall
    pingWall();
    rState = GO_HOME;
    break;

    case GO_HOME: // wall following home
    wallSwitch();
    break;

    case STOP: // made it home
    regDrive(MOTOR_STOP);
    break;
  }
}

// Drive Robot
// called if robot just needs to drive straight
void regDrive(int speed){
  leftMotor.write(MOTOR_MAX_FRW-speed);
  rightMotor.write(speed);
}


// Switch between wall following states
void wallSwitch(){
  switch (wState) {
    case FOLLOW: // following wall
    wallFollow(closeWall());
    break;
    case CORNER: // turning 90 degrees at a corner
    break;
    case WALL_END: // turning around the end of a wall
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
  // print left
  lcd.setCursor(0,0);
  lcd.print("L: ");
  lcd.print(leftDist);

  // print right
  lcd.setCursor(7,0);
  lcd.print(" R: ");
  lcd.print(rightDist);

  // print mid
  lcd.setCursor(0,1);
  lcd.print("M: ");
  lcd.print(midDist);

  // print closest wall
  lcd.setCursor(7,1);
  lcd.print(" W: ");
  lcd.print(closeWall());
}

// determines closest wall - left or right
// used for figuring out which wall to follow
// 'R' - right wall; 'L' - left wall; 'x' - debugging
char closeWall(){
  char result;

  if ((rightDist > 0) && (rightDist < leftDist)) result = 'R';
  else if ((leftDist > 0) && (leftDist < rightDist)) result = 'L';
  else if ((rightDist == 0) && (leftDist == 0)) result = 'x';

  return result;
}

// uses PID to follow wall
void wallFollow(char wall){
  // case for left wall being closer to robot
  if (wall == 'L'){
    pidInput = leftDist;
    pid.Compute(); // pid detects if it needs to run again

    // determine if dist is out of range of error
    if (leftDist <= (TARGET_DIST + pidError) && leftDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_MAX_FRW);
    } else {
      if (leftDist < TARGET_DIST){ // is wall is closer
        leftMotor.write(MOTOR_STOP + MIN_SPEED + (int) pidOutput);
        rightMotor.write(MOTOR_STOP - MIN_SPEED);
      } else if (leftDist > TARGET_DIST){ // if wall is further
        leftMotor.write(MOTOR_STOP + MIN_SPEED);
        rightMotor.write(MOTOR_STOP - MIN_SPEED + (int) pidOutput);
      }
    }

    // case for right wall being closer to robot
  } else if (wall == 'R'){
    pidInput = rightDist;
    pid.Compute(); // pid detects if it needs to run again

    // determine if dist is out of range of error
    if (rightDist <= (TARGET_DIST + pidError) && rightDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_MAX_FRW);
    } else {
        if (rightDist < TARGET_DIST){ // is wall is closer
        leftMotor.write(MOTOR_STOP + MIN_SPEED);
        rightMotor.write(MOTOR_STOP - MIN_SPEED - (int) pidOutput);
      } else if (rightDist > TARGET_DIST){ // if wall is further
        leftMotor.write(MOTOR_STOP + MIN_SPEED - (int) pidOutput);
        rightMotor.write(MOTOR_STOP - MIN_SPEED);
      }
    }
  }
}
