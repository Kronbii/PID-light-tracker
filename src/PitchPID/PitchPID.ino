//////////////////////////////Libraries Used////////////////////////////////
  #include "LightPID.h"
  #include <Servo.h>
////////////////////////////////////////////////////////////////////////////

//////////////////////////////Servo Variables///////////////////////////////
  Servo pitchServo;
  const int pitchPin = 6;
////////////////////////////////////////////////////////////////////////////

//////////////////////////////Sensor Values/////////////////////////////////
  double botr, botl, topr, topl;
  double setpoint, feedback;
  double output;
  double curError;
  int initAngle =0;
  int tiltAngle =0;
////////////////////////////////////////////////////////////////////////////

//////////////////////////////PID Objects///////////////////////////////////
  //Takes as input (Kp, Kd, Ki, minimum output, maximum output)
  Pitch pitch(0.07 , 0.00018, 0.002, -70, 55);

  int originTime;
  int curTime;
  int rstTime = 20000;
////////////////////////////////////////////////////////////////////////////

void setup() {
  //Set servo pin as output
  pinMode(pitchPin, OUTPUT);

  //Initialize values of error and input needed for PID
  setpoint = 0;
  feedback = 0;
  curError = 0;

  //Attach servo pin
  pitchServo.attach(pitchPin);

  //Set the servo initially vertical (90 degrees)
  pitchServo.write(90);

  //Takes the reference of the output angle and the resolution of scan as input
  pitchScan(initAngle, 10);

  //Origin time
  originTime = millis();
}

void loop() {
  //Read values of sensors and store them
  readSensors();
  
  //Calculating the value of setpoint and feedback based on the positions of the sensor positions
  setpoint = (botr + botl)/2.00;
  feedback = (topr + topl)/2.00;
  
  //Calculate the value of the PID error
  curError = setpoint - feedback;

  //Get the value of the output using the PID calculate method
  output = pitch.calculatePID(curError);

  //Move the servo to the caluclated PID angle
  pitchServo.write(90 - output);

  //reset integration error
  if ((millis() - originTime) > rstTime){
    pitch.resetIntegralError();
  }
}

//Function to read values of sensors
void readSensors(){
  topl = analogRead(A0);
  topr = analogRead(A5); //change to A1
  botl = analogRead(A2);
  botr = analogRead(A3);
}

//Function to initally scan the area to detect the initial source of light
void pitchScan(int& initAngle, int res){
  //initial reading of sensors
  readSensors();

  //variable to store initial reading
  double init = botr + topr + topl + botl;
  
  //variable to store reading during runtime
  double temp;

  for (int i=0; i<180; i = i + res){
    //write servo angle
    pitchServo.write(i);

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