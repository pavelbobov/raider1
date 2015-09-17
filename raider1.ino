// I2C device class (I2Cdev) demonstration Arduino sketch for AK8975 class
// 6/11/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// This example uses the AK8975 as mounted on the InvenSense MPU-6050 Evaluation
// Board, and as such also depends (minimally) on the MPU6050 library from the
// I2Cdevlib collection. It initializes the MPU6050 and immediately enables its
// "I2C Bypass" mode, which allows the sketch to communicate with the AK8975
// that is attached to the MPU's AUX SDA/SCL lines. The AK8975 is configured on
// this board to use the 0x0E address.
//
// Note that this small demo does not make use of any of the MPU's amazing
// motion processing capabilities (the DMP); it only provides raw sensor access
// to the compass as mounted on that particular evaluation board.
//
// For more info on the MPU-6050 and some more impressive demos, check out the
// device page on the I2Cdevlib website:
//     http://www.i2cdevlib.com/devices/mpu6050
//
// Changelog:
//     2012-06-11 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev, AK8975, and MPU6050 must be installed as libraries, or else the
// .cpp/.h files for all classes must be in the include path of your project
#include "I2Cdev.h"
#include "AK8975.h"
#include "MPU6050.h"
// Test code for Adafruit GPS modules using MTK3329/MTK3339 driver
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Ultimate GPS module
// using MTK33x9 chipset
//    ------> http://www.adafruit.com/products/746
// Pick one up today at the Adafruit electronics shop 
// and help support open source hardware & software! -ada

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include "geo.h"

//Analog pins
#define RUDDER_POT A0
#define SAIL_POT   A1
#define WIND_VANE  A2 
#define I2C_SDA    A4
#define I2C_SCL    A5

//Digital pins
#define RUDDER_PWM       3  //Motor Shield Channel A
#define GPS_RX           4
#define GPS_TX           5
#define SAIL_PWM         11 //Motor Shield Channel B
#define RUDDER_DIRECTION 12 //Motor Shield Channel A
#define SAIL_DIRECTION   13 //Motor Shield Channel B

#define LED_PIN          10       

// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change 
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(GPS_TX, GPS_RX);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;

Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy


// class default I2C address is 0x0C
// specific I2C addresses may be passed as a parameter here
// Addr pins low/low = 0x0C
// Addr pins low/high = 0x0D
// Addr pins high/low = 0x0E (default for InvenSense MPU6050 evaluation board)
// Addr pins high/high = 0x0F
AK8975 mag(0x0C);
MPU6050 accelgyro(0x68); // address = 0x68, the default, on MPU6050 EVB

int16_t mx, my, mz;
float heading;

bool blinkState = false;

Point location;
Point destination(20.784778, -155.996052);

void setup()  
{
    
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  //Serial.begin(38400);

  // initialize devices
  Serial.println("Initializing I2C devices...");
  
  // initialize MPU first so we can connect the AUX lines
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
  mag.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
  
  delay(1000);
  
 // float dest1[3], dest2[3];
 // magcalMPU9250(dest1, dest2);
 // Serial.println("Soft irons:");
 // Serial.println(dest1[0]);
 // Serial.println(dest1[1]);
 // Serial.println(dest1[2]);
 // Serial.println("Hard irons:");
 // Serial.println(dest2[0]);
 // Serial.println(dest2[1]);
 // Serial.println(dest2[2]);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop()                     // run over and over again
{
  // read raw heading measurements from device
  mag.getHeading(&mx, &my, &mz);

  // display tab-separated magnetometer x/y/z values
  Serial.print("mag:\t");
  mx += 62;
  my -= 101;
  mz -= 112;

  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.print(mz); Serial.print("\t\t");

  heading = atan2((double)my, (double)mx) * 180.0/3.14159265 + 180;
  while (heading < 0) heading += 360;
  while (heading > 360) heading -= 360;
  Serial.print(heading);
  Serial.println(" degrees");

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  
  delay(100); // run at ~10 Hz

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);

      Serial.print("True Course: ");
      Serial.println(GPS.angle, 4);
      location.set(GPS.latitudeDegrees, GPS.longitudeDegrees);

      Serial.print("Bearing to destination: ");
      Serial.println(location.bearing(&destination), 4);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}

/*
void magcalMPU9250(float * dest1, float * dest2) 
 {
   uint16_t ii = 0, sample_count = 0;
   int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
   int16_t mag_max[3] = {0xF000, 0xF000, 0xF000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};
  
   Serial.println("Mag Calibration: Wave device in a figure eight until done!");
   delay(4000);
  
   sample_count = 128;
   for(ii = 0; ii < sample_count; ii++) {
     //MPU9250readMagData(mag_temp);  // Read the mag data   
     mag.getHeading(mag_temp, mag_temp + 1, mag_temp + 2);
     for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
     }
     delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
   }
  
  // Get hard iron correction
   dest1[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
   dest1[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
   dest1[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
  
   //dest1[0] = (float) mag_bias[0]*MPU9250mRes*MPU9250magCalibration[0];  // save mag biases in G for main program
   //dest1[1] = (float) mag_bias[1]*MPU9250mRes*MPU9250magCalibration[1];   
   //dest1[2] = (float) mag_bias[2]*MPU9250mRes*MPU9250magCalibration[2];  
  
  // Get soft iron correction estimate
   mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
   mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
   mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
  
   float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
   avg_rad /= 3.0;
  
   dest2[0] = avg_rad/((float)mag_scale[0]);
   dest2[1] = avg_rad/((float)mag_scale[1]);
   dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
 }
*/
