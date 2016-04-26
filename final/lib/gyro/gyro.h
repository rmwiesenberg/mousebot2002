#ifndef gyro_h
#define gyro_h
#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"
#include <Wire.h>
#include <L3G.h>


class gyro {
public:
  void gyroSetup();
  void runGyro();

private:
  //This gain factor can be effected upto +/- %2 based on mechanical stress to the component after mounting.
  // if you rotate the gyro 180 degress and it only show 170 this could be the issue.
  float gyro_x; //gyro x val
  float gyro_y; //gyro x val
  float gyro_z; //gyro x val
  float gyro_xold; //gyro cummulative x value
  float gyro_yold; //gyro cummulative y value
  float gyro_zold; //gyro cummulative z value
  float gerrx; // Gyro x error
  float gerry; // Gyro y error
  float gerrz; // Gyro z error

};

#endif

// Definitions for the gyro
float G_Dt = 0.005;  // Integration time (DCM algorithm)  We will run the integration loop at 200Hz if possible
long timer = 0; // timer for the gyro
long timer1 = 0; //timer for printig
float G_gain = .00875; // gyros gain factor for 250deg/sec

#define TURRET_SPEED 79
#define TURRET_HOME 0
#define SERVO_HOME 100
#define FOUND_FLAME 500
// Simply keeps track of the gyro reading from an minimu9 using the microseconds clock.
// This code has been tested on 3 seperate seperate sets of hardware in serveral locations.
// the code results in less than 1 degree per minute drift when stationary.
//Testing included 3 consecutive trials of 3 minutes on each set of hardware in 3 places.
// the maxiumum error observeded with a steady voltage was 1.3 degress per minute on a single axis over the 27 test.
// Average error was .275 degrees per minute averaged across all 3 axis.
L3G gy;
