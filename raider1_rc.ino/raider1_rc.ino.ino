/*
 * Raider Of The Pacific
 http://www.paider1.com

 */

//Analog pins
#define RUDDER_CURRENT  A0 
#define WINCH_CURRENT   A1 
#define RUDDER_POT      A2
#define WINCH_POT       A3
#define I2C_SDA         A4
#define I2C_SCL         A5

//Digital pins
#define RUDDER_PWM       3  //Motor Shield Channel A
#define GPS_RX           4
#define GPS_TX           5
#define WIND_VANE_TXD    6
#define RUDDER_RC        7
#define BREAK_B          8
#define BREAK_A          9
#define WINCH_RC         10
#define WINCH_PWM        11 //Motor Shield Channel B
#define RUDDER_DIRECTION 12 //Motor Shield Channel A
#define WINCH_DIRECTION  13 //Motor Shield Channel B

void setup() {
  //start serial connection
  Serial.begin(9600);
  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(3, OUTPUT);
  
  pinMode(7, INPUT);
  pinMode(10, INPUT);
}

void loop() {
  //read the pushbutton value into a variable
  int ch1 = pulseIn(7,25000) ;//- 1500;
  int ch2 = pulseIn(10,25000);// - 1500;
  //print out the value of the pushbutton
  Serial.print(ch1);
  Serial.print(" ");
  Serial.println(ch2);

  if (ch1 == 0) {
    analogWrite(RUDDER_PWM, 0);
    analogWrite(WINCH_PWM, 0);
  } else {
    if (ch1 > 1500) {
      analogWrite(RUDDER_PWM, (ch1 - 1500)/2);
      digitalWrite(RUDDER_DIRECTION, HIGH);
    } else {
      analogWrite(RUDDER_PWM, (1500 - ch1)/2);
      digitalWrite(RUDDER_DIRECTION, LOW);
    }
  
    if (ch2 > 1500) {
      analogWrite(WINCH_PWM, (ch2 - 1500)/2);
      digitalWrite(WINCH_DIRECTION, HIGH);
    } else {
      analogWrite(WINCH_PWM, (1500 - ch2)/2);
      digitalWrite(WINCH_DIRECTION, LOW);
    }
  }

  delay(100);
}



