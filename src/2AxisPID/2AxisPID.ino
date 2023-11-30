//////////////////////////////Libraries Used/////////////////////////////////
  #include "LightPID.h"
  #include <Servo.h>
////////////////////////////////////////////////////////////////////////////

//////////////////////////////Servo Variables/////////////////////////////////
  Servo yawServo;
  Servo pitchServo;
  const int yawPin = 9;
  const int pitchPin = 6;
/////////////////////////////////////////////////////////////////////////////

//////////////////////////////Sensor Values/////////////////////////////////
  double botr, botl, topr, topl;
  double setpoint, feedback;
  double output;
  double curError;
  int initAngle =0;
  int tiltAngle =0;
////////////////////////////////////////////////////////////////////////////
float setpoint1;
float error1;
//////////////////////////////PID Objects/////////////////////////////////
  Yaw yaw(0.07, 00.00018, 0.002, -90, 90);
  Pitch pitch(0.07 , 0.00018, 0.002, -70, 55);
////////////////////////////////////////////////////////////////////////////
int per;
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

  setpoint = (botr + topr)/2;
  feedback = (botl + topl)/2;

  curError = setpoint - feedback;

  output = yaw.calculatePID(curError);
  yawServo.write(output);
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

/////FILLER FUNCTIONS///////
  /*
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

void yawScan(int& initAngle){
  //initial reading of sensors
  readSensors();
  
  //variable to store reading during runtime
  double temp;

  //variable to store initial reading
  double init = botr + topr + topl + botl;

  for (int i=0; i<180; i++){
    //write servo angle
    yaw.write(i);

    //read sensors to find the intensity of light
    readSensors();

    //calculate intensit of light
    temp = botr + topr + topl + botl;

    //update the maximum reading of sensors
    if (temp > init) initAngle = i;

    //control speed of servo
    delay(15);
  }
}

void pitchScan(int& initAngle){
  //initial reading of sensors
  readSensors();
  
  //variable to store reading during runtime
  double temp;

  //variable to store initial reading
  double init = botr + topr + topl + botl;
  
  for (int i=0; i<180; i++){
    //write servo angle
    pitch.write(i);

    //read sensors to find the intensity of light
    readSensors();

    //calculate intensit of light
    temp = botr + topr + topl + botl;

    //update the maximum reading of sensors
    if (temp > init) initAngle = i;

    //control speed of servo
    delay(15);
  }
}
*/