/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
#include <Wire.h>   
#include "MPU9250.h"

MPU9250 mpu;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 

float   temperature;    // Stores the real internal chip temperature in degrees Celsius

uint32_t count = 0, sumCount = 0; // used to control display output rate
//float pitch, yaw, roll;
float sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t delt_t = 0; // used to control display output rate

void setup()
{
  Serial.begin(38400);
  mpu.init();

/*
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
   delay(4000);
   
   float dest1[3], dest2[3];
   mpu.magcalMPU9250(dest1, dest2);
   Serial.println("Soft irons:");
   Serial.println(dest1[0]);
   Serial.println(dest1[1]);
   Serial.println(dest1[2]);
   Serial.println("Hard irons:");
   Serial.println(dest2[0]);
   Serial.println(dest2[1]);
   Serial.println(dest2[2]);
   Serial.println("Mag Calibration done!");
*/
}

void loop()
{  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmis match in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  
  delay(10);
  sumCount += mpu.mahonyQuaternionUpdate();

  float now = micros();
  float deltat = ((now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = now;

  sum += deltat; // sum for averaging filter update rate

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  
  if (delt_t > 500) 
  { // update LCD once per half-second independent of read rate

    if(SerialDebug) 
    {
      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth. 
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      float yaw   = mpu.getYaw();   
      float pitch = mpu.getPitch();
      float roll  = mpu.getRoll();

      Serial.print("Yaw, Pitch, Roll: ");
      Serial.print(yaw, 2);
      Serial.print(", ");
      Serial.print(pitch, 2);
      Serial.print(", ");
      Serial.println(roll, 2);

      Serial.print("rate = "); 
      Serial.print((float)sumCount/sum, 2); 
      Serial.println(" Hz");

      mpu.getTempData(&temperature);  // Read the adc values
      // Print temperature in degrees Centigrade      
      Serial.print("Temperature is ");  
      Serial.print(temperature, 1);  
      Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    //display.setCursor(0, 40); display.print("rt: "); display.print((float) sumCount / sum, 2); display.print(" Hz"); 
    //display.display();

    count = millis(); 
    sumCount = 0;
    sum = 0;    
  }
}



