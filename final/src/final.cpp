/***************  RBE2002 Final Code  ***************/
/**************    Team 4: Mousebot   ***************/

/**************      Richard Cole     ***************/
/**************    Elijah Eldredge    ***************/
/**************    Ryan Wiesenberg    ***************/

#include "final.h" // constants or other includes in other file for neatness

void setup() {
  // Uno setup
  pinMode(UNO_PIN1, OUTPUT);
  digitalWrite(UNO_PIN1, LOW);

  // Serial port for debugging
  Serial.begin(9600);

  // LCD Start
  lcd.begin(16,2);

  // Attach Motors
  rightMotor.attach(RIGHT_MOTOR_PIN, 1000, 2000);
  leftMotor.attach(LEFT_MOTOR_PIN, 1000, 2000);

  // PID setup
  pidSetpoint = TARGET_DIST; // sets dist from wall
  pid.SetOutputLimits(MOTOR_MAX_REV, (MOTOR_STOP - MIN_SPEED)); // sets limits for PID (essentially -60 to 60)
  pid.SetSampleTime(SAMPLE_TIME);
  pid.SetMode(AUTOMATIC); // turns PID on

  // turret setup
  extinguisher.turretSetup();
}

void loop(void) {
  switch (rState) {
    case INIT: // wall distance initializations
    pingWall();
    findWall();
    extinguisher.zero();
    rState = FIND_FLAME;
    wState = FOLLOW;
    break;

    case FIND_FLAME: // wall following to find flame
    pingWall();
    findWall();
    lcdPrintWallDist();

    if(extinguisher.foundFlame() == true) digitalWrite(UNO_PIN1, HIGH);
    else digitalWrite(UNO_PIN1, LOW);

    //wallSwitch();
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
    wallFollow();
    /*
    if(midDist < MIN_FRONT_DIST) wState = CORNER;
    if(cWall == LEFT && leftDist > MAX_DIST) wState = WALL_END;
    if(cWall == RIGHT && rightDist > MAX_DIST) wState = WALL_END;
    */
    break;

    case CORNER: // turning 90 degrees at a corner
    turnCorner();
    break;

    case WALL_END: // turning around the end of a wall
    turnEnd();
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
   switch (cWall) { // print closest wall
    case RIGHT:
    lcd.print("R");
    break;
    case LEFT:
    lcd.print("L");
    break;
    case ZERO:
    lcd.print("Z");
    break;
    case DEBUG:
    lcd.print("D");
    break;
  }
}

// determines closest wall - left or right
// used for figuring out which wall to follow
// RIGHT - right wall; LEFT - left wall; ZERO - if it can't see any wall
void findWall(void){
  if ((rightDist > 0) && (rightDist < leftDist)) cWall = RIGHT;
  else if ((leftDist > 0) && (leftDist < rightDist)) cWall = LEFT;
  else if ((rightDist == 0) && (leftDist == 0)) cWall = ZERO;
}

// uses PID to follow wall
void wallFollow(void){
  // case for left wall being closer to robot
  if (cWall == LEFT){
    pidInput = leftDist;
    pid.Compute(); // pid detects if it needs to run again

    // determine if dist is out of range of error
    if (leftDist <= (TARGET_DIST + pidError) && leftDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_STOP + REG_SPEED);
    } else {
      if (leftDist < TARGET_DIST){ // is wall is closer
        leftMotor.write(MOTOR_STOP - MIN_SPEED - (int) pidOutput);
        rightMotor.write(MOTOR_STOP + MIN_SPEED);
      } else if (leftDist > TARGET_DIST){ // if wall is further
        leftMotor.write(MOTOR_STOP - MIN_SPEED);
        rightMotor.write(MOTOR_STOP + MIN_SPEED + (int) pidOutput);
      }
    }

    // case for right wall being closer to robot
  } else if (cWall == RIGHT){
    pidInput = rightDist;
    pid.Compute(); // pid detects if it needs to run again

    // determine if dist is out of range of error
    if (rightDist <= (TARGET_DIST + pidError) && rightDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_STOP + REG_SPEED);
    } else {
        if (rightDist < TARGET_DIST){ // is wall is closer
        leftMotor.write(MOTOR_STOP - MIN_SPEED);
        rightMotor.write(MOTOR_STOP + MIN_SPEED + (int) pidOutput);
      } else if (rightDist > TARGET_DIST){ // if wall is further
        leftMotor.write(MOTOR_STOP - MIN_SPEED - (int) pidOutput);
        rightMotor.write(MOTOR_STOP + MIN_SPEED);
      }
    }

    // case if no walls are seen
  } else if (cWall == ZERO) {
    regDrive(MOTOR_STOP);
  }
}

void turnCorner(){

}

void turnEnd(){

}
