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
  double botr, botl, topr, topl;
  double setpoint, feedback;
  double output;
  double curError;
  int initAngle =0;
  int tiltAngle =0;
  bool orientation;// 0 for [0, 180] / 1 for [180, 360]
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

  //Initial servo positions
  panServo.write(90);
  pitchServo.write(90);

}

void loop() {
  //////////////////////////////Control of the tilt mechanism/////////////////////////////////
  //pitch control
  readSensors();
  
  //Calculating the systems input and feedback
  setpoint = (botr + botl)/2.00;
  feedback = (topr + topl)/2.00;
  
  //Calculating error 
  curError = setpoint - feedback;

  //Using the calculate method to get the output of the PID
  output = pitch.calculatePID(curError);

  //Determening the hemisphere in which the tilted system is currently located
  if (output > 0) orientation = 1;
  else if (output <0) orientation = 0;
  
  pitchServo.write(90 - output);

  //////////////////////////////Control of the pan mechanism/////////////////////////////////
  setpoint = (botr + topr)/2.00;
  feedback = (botl + topl)/2.00;

  curError = setpoint - feedback;

  output = pan.calculatePID(curError);

  //If the system is the the [180, 360] hemisphere, the output is inverted
  if (orientation == 1) output = -output;
  panServo.write(90 - output);
}

void readSensors(){
  topr = analogRead(A0);
  topl = analogRead(A5); //change to A1
  botr = analogRead(A2);
  botl = analogRead(A3);
}

