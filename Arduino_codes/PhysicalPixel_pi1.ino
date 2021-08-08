/*
  Physical Pixel

  An example of using the Arduino board to receive data from the computer. In
  this case, the Arduino boards turns on an LED when it receives the character
  'H', and turns off the LED when it receives the character 'L'.

  The data can be sent from the Arduino Serial Monitor, or another program like
  Processing (see code below), Flash (via a serial-net proxy), PD, or Max/MSP.

  The circuit:
  - LED connected from digital pin 13 to ground through 220 ohm resistor

  created 2006
  by David A. Mellis
  modified 30 Aug 2011
  by Tom Igoe and Scott Fitzgerald

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/PhysicalPixel
*/
#define Forward 1
#define Reverse 0

const int ledPin = 13; // the pin that the LED is attached to
int incomingByte;      // a variable to read incoming serial data into
int R_PWM = 10;
int L_PWM = 9;
int fw_right_speed = 145;
int fw_left_speed  = 140;
int rv_right_speed = 50;
int rv_left_speed = 50;
int R_direction = 5;
int L_direction = 6;

void setup() {
  // initialize serial communication:
  Serial.begin(115200);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
}

void forward();
void stop_c();
void left();
void right();

void loop() {
  // see if there's incoming serial data:
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    if (incomingByte == 'w') {
      forward();
      digitalWrite(ledPin, HIGH);
    }
    
    if (incomingByte == 'd') {
      right();
    } 
       
    if (incomingByte == 'a') {
      left();
    }
    
    if (incomingByte == 's') {
      reverse();
            digitalWrite(ledPin, LOW);
    }

        if (incomingByte == 'p') {
        stop_c();
            digitalWrite(ledPin, LOW);
    }
    
  }
}

void forward() 
{
        analogWrite(R_PWM, fw_right_speed);
        digitalWrite(R_direction, Forward);
        analogWrite(L_PWM, fw_left_speed);
        digitalWrite(L_direction, Forward);     
}

void right() 
{
        analogWrite(R_PWM, 32);
        digitalWrite(R_direction, Reverse);
        analogWrite(L_PWM, 210);
        digitalWrite(L_direction, Forward); 
}


void left() 
{   
        
        analogWrite(R_PWM, 210);
        digitalWrite(R_direction, Forward);
        analogWrite(L_PWM, 32);
        digitalWrite(L_direction, Reverse);
}

void reverse() 
{
        analogWrite(R_PWM, rv_left_speed);
        digitalWrite(R_direction, Reverse);
        analogWrite(L_PWM, rv_left_speed);
        digitalWrite(L_direction, Reverse);     
}

void stop_c() 
{
        analogWrite(R_PWM, 255);
        digitalWrite(R_direction, Forward);
        analogWrite(L_PWM, 255);
        digitalWrite(L_direction, Forward);    
}
