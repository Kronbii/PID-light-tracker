#include "LightPID.h"
#include <Servo.h>

Servo yawServo;
Servo pitchServo;

//////////////////////////////Sensor values/////////////////////////////////
double s1,s2,s3,s4;
double setpoint, feedback;
double output;
double curError;
////////////////////////////////////////////////////////////////////////////

Yaw yaw(30, 000, 0.0, 9);
Pitch pitch(100, 000, 0, 10);


void readSensors();

void setup() {
  Serial.begin(9600);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  setpoint = 0;
  feedback = 0;
  curError = 0;

  yawServo.attach(9);
  pitchServo.attach(10);

  yawServo.write(0);
  pitchServo.write(90);
}

void loop() {
  readSensors();

  //update error values for yaw
  if ((s1 + s3) > (s2 + s4)){
    setpoint = (s1 + s3)/2.00;
    feedback = (s2 + s4)/2.00;
  }
  else {
    setpoint = (s2 + s4)/2.00;
    feedback = (s1 + s3)/2.00;
  }

  //calculating error
  curError = setpoint - feedback;

  output = yaw.calculatePID(curError);
  yawServo.write(90 + output);
 
  //pitch control
  readSensors();
  
  if((s1 + s2) > (s3 + s4))
  {
  setpoint = (s3 + s4)/2.00;
  feedback = (s1 + s2)/2.00;
  }
  else{
    setpoint = (s1 + s2)/2.00;
    feedback = (s3 + s4)/2.00;
  }
  
  curError = setpoint - feedback;

  output = pitch.calculatePID(curError);
  
  if (output > 25) output = 25;
  if (output < -25) output = -25;

  pitchServo.write(90 + output);
}

void readSensors(){
  s2 = analogRead(A0);
  s4 = analogRead(A1);
  s1 = analogRead(A2);
  s3 = analogRead(A3);
  
  s1 = s1 * 5.00/1023;
  s2 = s2 * 5.00/1023;
  s3 = s3 * 5.00/1023;
  s4 = s4 * 5.00/1023;

  Serial.println(s1);
   Serial.println(s2);
    Serial.println(s3);
     Serial.println(s4);
      Serial.println("");
      delay(1000);
}
