//////////////////////////////Libraries Used/////////////////////////////////
  #include "LightPID.h"
  #include <Servo.h>
////////////////////////////////////////////////////////////////////////////

//////////////////////////////Servo Variables/////////////////////////////////
  Servo yawServo;
  Servo pitchServo;
  const int yawPin = 10;
  const int pitchPin = 6;
////////////////////////////////////////////////////////////////////////////

//////////////////////////////Sensor Values/////////////////////////////////
  double botr, botl, topr, topl;
  double setpoint, feedback;
  double output;
  double curError;
////////////////////////////////////////////////////////////////////////////

//////////////////////////////PID Objects/////////////////////////////////
  Yaw yaw(0.3, 0, 0, -90, 90);
  Pitch pitch(0.5 , 0.01, 0, -45, 45);
////////////////////////////////////////////////////////////////////////////

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

  /*
  Serial.print("output = ");
  Serial.println(output);
  */
  
  pitchServo.write(90 - output);
}

void readSensors(){
  topl = analogRead(A0);
  topr = analogRead(A5); //change to A1
  botl = analogRead(A2);
  botr = analogRead(A3);

  /*
  Serial.println(botr);
   Serial.println(botl);
    Serial.println(topr);
     Serial.println(topl);
      Serial.println("");
      delay(500);
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
  
  output = yaw.calculatePID(curError);
 
  yawServo.write(90 + output);
}
