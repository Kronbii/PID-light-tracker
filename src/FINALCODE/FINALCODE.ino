//////////////////////////////Libraries Used/////////////////////////////////
  #include "LightPID.h"
  #include <Servo.h>
////////////////////////////////////////////////////////////////////////////

//////////////////////////////Servo Variables/////////////////////////////////
  Servo panServo;
  Servo pitchServo;
  const int panPin = 9;
  const int pitchPin = 6;
/////////////////////////////////////////////////////////////////////////////

//////////////////////////////Sensor Values/////////////////////////////////
  //double botr, botl, topr, topl;
  double setpoint, feedback;
  double output;
  double curError;
  int initAngle =0;
  int tiltAngle =0;
  bool orientation;// 1 3am ysalle 0 ma 3m ysalle
////////////////////////////////////////////////////////////////////////////

//////////////////////////////PID Objects/////////////////////////////////
  Pan pan(0.1, 0.00018, 0.0007, -90, 90); //0.1, 0.00018, 0.0007
  Pitch pitch(0.03 , 0.00018, 0.00035, -70, 55); //0.07, 0.00018, 0.002
////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);

  pinMode(panPin, OUTPUT);
  pinMode(pitchPin, OUTPUT);

  setpoint = 0;
  feedback = 0;
  curError = 0;

  panServo.attach(panPin);
  pitchServo.attach(pitchPin);

  panServo.write(90);
  pitchServo.write(90);

}

void loop() {
  //pitch control
  readSensors();
  
  setpoint = (botr + botl)/2.00;
  feedback = (topr + topl)/2.00;
  
  curError = setpoint - feedback;

  output = pitch.calculatePID(curError);

  if (output > 0) orientation = 1;
  else if (output <0) orientation = 0;
  
  pitchServo.write(90 - output);

  setpoint = (botr + topr)/2;
  feedback = (botl + topl)/2;

  curError = setpoint - feedback;

  output = pan.calculatePID(curError);

  if (orientation == 1) output = -output;
  panServo.write(90 - output);
}
