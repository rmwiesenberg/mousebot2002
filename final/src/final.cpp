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
  Serial.print(30);


  // LCD Start
  lcd.begin(16,2);

  // Attach Motors
  rightMotor.attach(RIGHT_MOTOR_PIN, 1000, 2000);
  leftMotor.attach(LEFT_MOTOR_PIN, 1000, 2000);

  // PID setup
  pidSetpoint = TARGET_DIST; // sets dist from wall
  pid.SetOutputLimits(MOTOR_MAX_REV, (MAX_PID_SPEED - MIN_SPEED)); // sets limits for PID (essentially -60 to 60)
  pid.SetSampleTime(SAMPLE_TIME);
  pid.SetMode(AUTOMATIC); // turns PID on

  // turret setup
  extinguisher.turretSetup();

  // mouse setup
  //leftMouse.mouse_init();
  //rightMouse.mouse_init();
}

void loop(void) {
 updatePos();
 lcdPrintTravelDist();
  switch (rState) {
    case INIT: // wall distance initializations
    pingWall();
    findWall();
    //lcd.setCursor(0,0);
    //lcd.print("come @ me");
    extinguisher.zero();
    //lcd.setCursor(0,0);
    //lcd.print("bitch");
    rState = FIND_FLAME;
    wState = FOLLOW;
    break;

    case FIND_FLAME: // wall following to find flame
    //lcd.setCursor(0,0);
    //lcd.print("fuck you");
    pingWall();
    //lcd.setCursor(0,0);
    //lcd.print("bleh");
    findWall();
    //lcd.setCursor(0,0);
    //lcd.print("fite me");
    wallSwitch();
    if(extinguisher.foundFlame() == true) rState = TURN_FLAME;
    break;

    case TURN_FLAME:
    regDrive(MOTOR_STOP);
    digitalWrite(UNO_PIN1, HIGH);
    // turnDeg(extinguisher.getPosFlame());
    extinguisher.go(extinguisher.getPosFlame());
    rState = TO_FLAME;
    break;

    case TO_FLAME: // moving toward flame
    digitalWrite(UNO_PIN1, HIGH);
    setCandle();
    rState = EXTINGUISH;
    break;

    case EXTINGUISH: // extinguishing flame
    digitalWrite(UNO_PIN1, HIGH);
    if(extinguisher.extinguish()) {
      rState = FIND_WALL;
      digitalWrite(UNO_PIN1, LOW);
    }
    break;

    case FIND_WALL: // re-find and drive to wall
    if(turnDeg(180)) rState = GO_HOME;
    break;

    case GO_HOME: // wall following home
    pingWall();
    findWall();
    wallSwitch();
    if(home()) rState = STOP;
    break;

    case STOP: // made it home
    while(1){
      regDrive(MOTOR_STOP);
      lcdCandle();
    }
    break;
  }
}

// Drive Robot
// called if robot just needs to drive straight
void regDrive(int speed){
  leftMotor.write(MOTOR_MAX_FRW-speed);
  rightMotor.write(speed);
}

// called if robot needs to turn right
void turnRight(){
  leftMotor.write(MOTOR_STOP + TURN_SPEED);
  rightMotor.write(MOTOR_STOP + TURN_SPEED);
}

// called if robot needs to turn left
void turnLeft(){
  leftMotor.write(MOTOR_STOP - TURN_SPEED);
  rightMotor.write(MOTOR_STOP - TURN_SPEED);
}

// Switch between wall following states
void wallSwitch(){
  switch (wState) {
    case FOLLOW: // following wall
    wallFollow();

    if(midDist < MIN_FRONT_DIST) wState = CORNER;
    // if(cWall == LEFT && leftDist > MAX_DIST) wState = WALL_END;
    // if(cWall == RIGHT && rightDist > MAX_DIST) wState = WALL_END;

    break;

    case CORNER: // turning 90 degrees at a corner
    extinguisher.stop();
    turnCorner();
    break;

    case WALL_END: // turning around the end of a wall
    extinguisher.stop();
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

//prints the important distance information and closest wall
void lcdPrintTravelDist(){
  // print heading
  lcd.setCursor(0,0);
  lcd.print("H: ");
  lcd.print(heading);

  // print right
  lcd.setCursor(7,0);
  lcd.print(" X: ");
  lcd.print(tx);

  // print mid
  lcd.setCursor(0,1);
  lcd.print("Y: ");
  lcd.print(ty);

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

// print encoder values to the LCD
void lcdPrintEncVals(){
  // print turret enc
  lcd.setCursor(0,0);
  lcd.print("D: ");
  lcd.print(deg);

  // print left drive enc
  lcd.setCursor(7,0);
  lcd.print(" L: ");
  lcd.print(ld);

  // print right drive enc
  lcd.setCursor(0,1);
  lcd.print("R: ");
  lcd.print(rd);

  //print heading
  lcd.setCursor(7,1);
  lcd.print("H: ");
  lcd.print(heading);
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
  lcd.print("F: ");
  lcd.print(extinguisher.getPosFlame());

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

void lcdCandle(){
  // print left
  lcd.setCursor(0,0);
  lcd.print("X: ");
  lcd.print(cx);

  // print mid
  lcd.setCursor(0,1);
  lcd.print("Y: ");
  lcd.print(cy);
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
  switch (cWall) {
    case LEFT:
    extinguisher.sweep(-100, 0);
    pidInput = leftDist;
    pid.Compute(); // pid detects if it needs to run again

    // determine if dist is out of range of error
    if (leftDist <= (TARGET_DIST + pidError) && leftDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_STOP - REG_SPEED);
    } else {
      if (leftDist < TARGET_DIST){ // is wall is closer
        leftMotor.write(MOTOR_STOP + MIN_SPEED + (int) pidOutput);
        rightMotor.write(MOTOR_STOP - MIN_SPEED);
      } else if (leftDist > TARGET_DIST){ // if wall is further
        leftMotor.write(MOTOR_STOP + MIN_SPEED);
        rightMotor.write(MOTOR_STOP - MIN_SPEED - (int) pidOutput);
      }
    }
    break;

    // case for right wall being closer to robot
    case RIGHT:
    extinguisher.sweep(0, 100);
    pidInput = rightDist;
    pid.Compute(); // pid detects if it needs to run again

    // determine if dist is out of range of error
    if (rightDist <= (TARGET_DIST + pidError) && rightDist >= (TARGET_DIST - pidError)){
      regDrive(MOTOR_STOP - REG_SPEED);
    } else {
      if (rightDist < TARGET_DIST){ // is wall is closer
        leftMotor.write(MOTOR_STOP + MIN_SPEED);
        rightMotor.write(MOTOR_STOP - MIN_SPEED - (int) pidOutput);
      } else if (rightDist > TARGET_DIST){ // if wall is further
        leftMotor.write(MOTOR_STOP + MIN_SPEED + (int) pidOutput);
        rightMotor.write(MOTOR_STOP - MIN_SPEED);
      }
    }
    break;

    // case if no walls are seen
    case ZERO:
    regDrive(MOTOR_STOP);
    break;

    case DEBUG:
    regDrive(MOTOR_STOP);
    break;
  }
}

//turns the robot 90 degrees to avoid a corner
void turnCorner(){
  switch (cWall) {//turns the robot right or left
    case LEFT:
    regDrive(MOTOR_STOP);
    if(turnDeg(-90)) wState = FOLLOW;
    break;

    case RIGHT:
    regDrive(MOTOR_STOP);
    if(turnDeg(90)) wState = FOLLOW;
    break;

    case DEBUG:
    regDrive(MOTOR_STOP);
    break;

    case ZERO:
    regDrive(MOTOR_STOP);
    break;
  }
}

//turns the robot around the end of a wall
void turnEnd(){
}

//turns the robot the given number of degrees
//either right or left
boolean turnDeg(double degTurn){
  switch (turning) {
    case START:
      regDrive(MOTOR_STOP);
      deg = heading + degTurn;
      turning = MOVING;
      return false;
    break;

    case MOVING:
      if(deg < 0 && deg <= heading) {
        turnRight();
        updatePos();
      } else if (deg > 0 && deg >= heading) {
        turnLeft();
        updatePos();
      } else {
        turning = END;
        regDrive(MOTOR_STOP);
      }
      return false;
    break;

    case END:
      turning = START;
      return true;
    break;
  }
}

// drives straight for the given distance
void driveDist(double dist){
  long initrEncVal = rdenc.read();
  long initlEncVal = ldenc.read();

  double trdist = 0;
  double tldist = 0;

  while(dist > trdist && dist > tldist){
    regDrive(MOTOR_STOP + REG_SPEED);
    trdist = (rdenc.read() - initrEncVal) * (circum / encMaxVal);
    tldist = (ldenc.read() - initlEncVal) * (circum / encMaxVal);
  }

  regDrive(MOTOR_STOP);
  updatePos();

}

// float getDeg(){
//   leftMouse.mouse_pos(lstat, lx, ly);
//   rightMouse.mouse_pos(rstat, rx, ly);
//
//   double x1 = (double) lx;
//   double x2 = (double) rx;
//
//   deg = deg + (((abs(x1) + abs(x2)) / 2.0) * turnConst);
//   fuMice();
//   return deg;
// }

//calculated the distance driven by the wheels
void distDriven(){
  rEncVal = rdenc.read();
  lEncVal = ldenc.read();

  rd = (double) (rEncVal-oldrEncVal);
  ld = (double) (lEncVal-oldlEncVal);

  rDist = (rd * ratio) * (circum / encMaxVal);
  lDist = (ld * ratio) * (circum / encMaxVal);

  oldrEncVal = rEncVal;
  oldlEncVal = lEncVal;
}

//updates the calculated position
//of the robot with the position data from
//the last loop
void updatePos(){
  distDriven();

  heading = heading + ((rDist-lDist) * turnConst);

  tx = tx + ((((ld + rd) / 2) * ratio) * (circum / encMaxVal) * cos(heading));
  ty = ty + ((((ld + rd) / 2) * ratio) * (circum / encMaxVal) * sin(heading));
}

void setCandle(){
  cx = tx + cdist * cos(heading + extinguisher.getPosFlame());
  cy = ty + cdist * sin(heading + extinguisher.getPosFlame());
}

boolean home(){
  return ((tx < 4) && (tx > -4) && (ty < 4) && (ty > -4));
}
