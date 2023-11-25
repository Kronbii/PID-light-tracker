#include "LightPID.h"
#include <Servo.h>

Servo yawServo;
Servo pitchServo;
const int yawPin = 10;
const int pitchPin = 6;

//////////////////////////////Sensor values/////////////////////////////////
double botr, botl, topr, topl;
double setpoint, feedback;
double output;
double curError;
////////////////////////////////////////////////////////////////////////////

Yaw yaw(0.3, 000, 0.0, -90, 90);
Pitch pitch(0.08, 0.09, 0, -45, 45);

void setup() {
  Serial.begin(9600);

  pinMode(yawPin, OUTPUT);
  pinMode(pitchPin, OUTPUT);

  setpoint = 0;
  feedback = 0;
  curError = 0;

  yawServo.attach(yawPin);
  pitchServo.attach(pitchPin);

  yawServo.write(90);
  pitchServo.write(90);
}

void loop() {
  //pitch control
  readSensors();
  
  setpoint = (botr + botl)/2.00;
  feedback = (topr + topl)/2.00;
  
  curError = setpoint - feedback;

  output = pitch.calculatePID(curError);

  
  Serial.print("output = ");
  Serial.println(output);
  
  
  pitchServo.write(90 - output);
}

void readSensors(){
  topl = analogRead(A0);
  topr = analogRead(A1);
  botl = analogRead(A2);
  botr = analogRead(A3);

 /*
  botr = botr * 5.00/1023;
  botl = botl * 5.00/1023;
  topr = topr * 5.00/1023;
  topl = topl * 5.00/1023;
*/

 /*
  Serial.println(botr);
   Serial.println(botl);
    Serial.println(topr);
     Serial.println(topl);
      Serial.println("");
      delay(1000);
*/
}

/////FILLER FUNCTION///////
void yawCode(){
  readSensors();

  //update error values for yaw
    setpoint = (botr + topr)/2.00;
    feedback = (botl + topl)/2.00;
    
  //calculating error
  curError = setpoint - feedback;

  Serial.println(curError);
  delay(1000);
  
  output = yaw.calculatePID(curError);

  Serial.print("output = ");
  Serial.println(output);
  Serial.println("");
  delay(1000);
 
  yawServo.write(90 + output);
}
