#include <Servo.h>
#include <PID_v1.h>

#define yawPin 6
#define pitchPin 5

Servo yaw;
Servo pitch;

//Define Variables we'll be connecting to
double setpoint, output, feedback;
double s1, s2, s3 ,s4;

//Specify the links and initial tuning parameters
double Kp=10000, Ki=0, Kd=0.000;
PID myPID(&feedback, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  Serial.begin(19200);
  //initialize the variables we're linked to
  feedback = 0;
  setpoint = 0;

  //turn the PID on
  yaw.attach(yawPin);
  pitch.attach(pitchPin);
  myPID.SetMode(AUTOMATIC);
}


void loop()
{
  readSensors();
  setpoint = (botr + botl)/2.00;
  feedback = (topr + topl)/2.00;
  myPID.Compute();
  output = output * 180/1023;
  pitch.write(output);
}

void readSensors(){
  s1 = analogRead(A0);
  s2 = analogRead(A1);
  s3 = analogRead(A2);
  s4 = analogRead(A3);

 s1 = s1 * 5.00/1023;
 s2 = s2 * 5.00/1023;
 s3 = s3 * 5.00/1023;
 s4 = s4 * 5.00/1023;

}


