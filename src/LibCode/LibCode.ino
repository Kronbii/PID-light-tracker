#include <Servo.h>
#include <PID_v1.h>

#define yawPin 6
#define pitchPin 5

Servo yaw;
Servo pitch;
/*
#define s1 A0
#define s2 A1
#define s3 A2
#define s4 A3
*/

//Define Variables we'll be connecting to
double setpoint, output, feedback;
double s1, s2, s3 ,s4;

//Specify the links and initial tuning parameters
double Kp=10, Ki=0, Kd=0.000;
PID myPID(&feedback, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(9600);
  //initialize the variables we're linked to
  //Input = analogRead(PIN_INPUT);
  //Setpoint = 100;

  //turn the PID on
  yaw.attach(yawPin);
  pitch.attach(pitchPin);
  myPID.SetMode(AUTOMATIC);
}

//S3////////////////////////////S4//
////////////////////////////////////
////////////////////////////////////
////////////////////////////////////
////////////////////////////////////
//S1////////////////////////////S2//

void loop()
{
  readSensors();

  if (s1 < s2){
    setpoint = (s2 + s4)/2.00;
    feedback = (s1 + s3)/2.00;
  }
  else{
    setpoint = (s1 + s3)/2.00;
    feedback = (s2 + s4)/2.00;
  }

  myPID.Compute();
  output = map(output,0,1023,500,2000);
  output = int(output);
  //yaw.writeMicroseconds(output);

  readSensors();
  if (s1 < s3){
    setpoint = (s3 + s4)/2.00;
    feedback = (s1 + s2)/2.00;
  }
  else{
    setpoint = (s1 + s2)/2.00;
    feedback = (s3 + s4)/2.00;
  }

  myPID.Compute();
  output = map(output,0,1023,500,2000);
  output = int(output);
  //pitch.writeMicroseconds(output);
}

void readSensors(){
  s1 = analogRead(A0);
  s2 = analogRead(A1);
  s3 = analogRead(A2);
  s4 = analogRead(A3);
  Serial.print("s1 = ");
  Serial.println(s1);
  Serial.print("s2 = ");
  Serial.println(s2);
  Serial.print("s3 = ");
  Serial.println(s3);
  Serial.print("s4 = ");
  Serial.println(s4);
  Serial.println("");
  delay(5000);
}


