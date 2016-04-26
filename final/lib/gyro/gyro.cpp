#include "gyro.h"

void gyro::gyroSetup() {
  Serial.begin(115200);//fastest transmission
  Wire.begin(); // i2c begin
  if (!gy.init()) // gyro init
  {
    Serial.println("Failed to autodetect gyro type! not connected");
    while (1);
  }
  delay(500);
  timer = micros(); // init timer for first reading
  gy.enableDefault(); // gyro init. default 250/deg/s
  delay(1000);// allow time for gyro to settle
  Serial.println("starting zero, stay still for 10 seconds");
  for (int i = 1; i <= 2000; i++) { // takes 2000 samples of the gyro
    gy.read(); // read gyro I2C call
    gerrx += gy.g.x; // add all the readings
    gerry += gy.g.y;
    gerrz += gy.g.z;
    delay(5);
  }

  gerrx = gerrx / 2000; // average readings to obtain an error offset
  gerry = gerry / 2000;
  gerrz = gerrz / 2000;

  Serial.println(gerrx); // print error vals
  Serial.println(gerry);
  Serial.println(gerrz);
}

void gyro::runGyro() {

  if ((micros() - timer) >= 5000) { // wait for 5000 microseconds
    gy.read(); // read gyro

    gyro_x = (gy.g.x - gerrx) * G_gain; // offset by error then multiply by gyro gain factor
    gyro_y = (gy.g.y - gerry) * G_gain;
    gyro_z = (gy.g.z - gerrz) * G_gain;

    gyro_x = gyro_x * G_Dt; // Multiply the angular rate by the time interval
    gyro_y = gyro_y * G_Dt;
    gyro_z = gyro_z * G_Dt;

    gyro_x += gyro_xold; // add the displacment(rotation) to the cumulative displacment
    gyro_y += gyro_yold;
    gyro_z += gyro_zold;

    gyro_xold = gyro_x ; // Set the old gyro angle to the current gyro angle
    gyro_yold = gyro_y ;
    gyro_zold = gyro_z ;
    // flag = 0;
    timer = micros(); //reset timer

  }


  if ((millis() - timer1) >= 1000) // print the gyro values once per second
  {
    timer1 = millis();

    Serial.print("G ");
    Serial.print("X: ");
    Serial.print(gyro_x);
    Serial.print(" Y: ");
    Serial.print(gyro_y);
    Serial.print(" Z: ");
    Serial.println(gyro_z);
  }

}
