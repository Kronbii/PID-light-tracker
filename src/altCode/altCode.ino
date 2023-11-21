#include "LightPID.h"
#include <Servo.h>

void readSensors();

Servo yawServo;
Servo pitchServo;

//////////////////////////////Sensor values/////////////////////////////////
double s1,s2,s3,s4;
double setpoint, feedback;
double output;
double curError;
////////////////////////////////////////////////////////////////////////////

Yaw yaw(100, 0, 0, 9);
Pitch pitch(1000, 0, 0, 5);


void readSensors();

void setup() {
  Serial.begin(9600);
  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);

/*
  yaw.controlServo(0);
  pitch.controlServo(0);
*/
  setpoint = 0;
  feedback = 0;
  curError = 0;

  yawServo.attach(9);
  pitchServo.attach(6);
  yawServo.write(0);
  delay(1000);

}

void loop() {
  readSensors();

  //update error values for yaw
  /*
  if (s1 > s2){
    setpoint = (s1 + s3)/2.00;
    feedback = (s2 + s4)/2.00;
  }
  else if(s1 < s2){
    setpoint = (s2 + s4)/2.00;
    feedback = (s1 + s3)/2.00;
  }
  */
/*
  if (s3 > s4){
    setpoint += s3;
    feedback += s4;
  }
  else if(s3 < s4){
    setpoint += s4;
    feedback += s3;
  }
*/
  //yaw.prevError = yaw.error;
  //yaw.error = setpoint - feedback;
  setpoint = (s1 + s3)/2.00;
  feedback = (s2 + s4)/2.00;
  curError = setpoint - feedback;

  output = yaw.calculatePID(curError);
  //yaw.controlServo(output);
  output = int(output);
  yawServo.write(90+output);

  Serial.print("output = ");
  Serial.println(output);
  delay(1000);


  readSensors();

///////FIXXXXX THISSSSS
  setpoint = s3 + s4;
  feedback = s1 + s2;

  //pitch.prevError = pitch.error;
  curError = setpoint - feedback;

  output = pitch.calculatePID(curError);
  //pitch.controlServo(output);
  pitchServo.write(output);

}

void readSensors(){
  s1 = analogRead(A0);
  s2 = analogRead(A4);
  s3 = analogRead(A2);
  s4 = analogRead(A3);

 s1 = s1 * 5.00/1023;
 s2 = s2 * 5.00/1023;
 s3 = s3 * 5.00/1023;
 s4 = s4 * 5.00/1023;

/*
  Serial.print("s1 = ");
  Serial.println(s1);
  Serial.print("s2 = ");
  Serial.println(s2);
  Serial.print("s3 = ");
  Serial.println(s3);
  Serial.print("s4 = ");
  Serial.println(s4);
  Serial.println("");
  delay(1000);
  */
}