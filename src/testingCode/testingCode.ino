#include "LightPID.h"
#include <Servo.h>

Servo yawServo;
Servo pitchServo;
const int yawPin = 9;
const int pitchPin = 10;

//////////////////////////////Sensor values/////////////////////////////////
double s1,s2,s3,s4;
double setpoint, feedback;
double output;
double curError;
////////////////////////////////////////////////////////////////////////////

Yaw yaw(0.03, 000, 0.0, 90, -90);
Pitch pitch(0.03, 000, 0, 25, -25);


void readSensors();

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
  readSensors();

  //update error values for yaw
    setpoint = (s1 + s3)/2.00;
    feedback = (s2 + s4)/2.00;
    
  //calculating error
  curError = setpoint - feedback;

  /*
  Serial.println(curError);
  delay(1000);
  */
  
  output = yaw.calculatePID(curError);
/*
  Serial.print("output = ");
  Serial.println(output);
  Serial.println("");
  delay(1000);
  */
  yawServo.write(90 + output);
 
  //pitch control
  readSensors();
  
  setpoint = (s1 + s2)/2.00;
  feedback = (s3 + s4)/2.00;
  
  curError = setpoint - feedback;

  output = pitch.calculatePID(curError);


  Serial.print("output = ");
  Serial.println(output);
  delay(1000);
  
  pitchServo.write(90 + output);
}

void readSensors(){
  s1 = analogRead(A0);
  s2 = analogRead(A1);
  s3 = analogRead(A2);
  s4 = analogRead(A3);

  /*
  s1 = s1 * 5.00/1023;
  s2 = s2 * 5.00/1023;
  s3 = s3 * 5.00/1023;
  s4 = s4 * 5.00/1023;
/*
  Serial.println(s1);
   Serial.println(s2);
    Serial.println(s3);
     Serial.println(s4);
      Serial.println("");
      delay(1000);
      */

}
