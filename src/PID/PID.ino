#include <Servo.h>

//define servo motors
Servo yawServo;
Servo pitchServo;
int yawPin; //add a value
int pitchPin; //add a value

//Variables to store the PID gains
float kp, kd, ki;
float kp1, kd1, ki1;
float kp2, kd2, ki2;

//Variables to store PID values
float propValue1, integValue1, derivValue1;
float propValue2, integValue2, derivValue2;

//PID input and feedback
float setpoint, feedback;

//sensor readings
float s1, s2, s3, s4;

//output value of PID
double output;

//error values
double error, derivError, integError, prevError;

//function definitions
double PID_control(double error);
void readSensors();
void PID_control();

void setup() {
  // Initialize gain values
  kp = 1;
  kd= 1;
  ki = 1;

  kp1 = kp;
  kd1 = kd;
  ki1 = ki;

  kp2 = kp;
  kd2 = kd;
  ki2 = ki;

  //Attach servos
  yawServo.attach(yawPin);
  pitchServo.attach(pitchPin);

  //Set servo motors to midpoint
  yawServo.writeMicroseconds(1500);
  pitchServo.writeMicroseconds(1500);

}

void loop() {
  //read data from sensors using read function
  readSensors();
  //get output for PID using PID function

  //convert output to a servo position
  //update servo position
}

double PID_control(double error, double prevError){
  derivError = error - prevError;
  integError = error + integError;
  return output = kp*error + kd*derivError + ki*integError;
}

void readSensors(){
  //read values from analog input pins
  s1 = analogRead(A1);
  s2 = analogRead(A2);
  s3 = analogRead(A3);
  s4 = analogRead(A4);

  //find the TF of the LDR depending on the lowest and maximum values of light
  //put the LDR in a voltage division circuit to convert resistave output to voltage

}

void PID_control(double output){
  //1500 is midpoint, 1000 is fully ccw, 2000 is fully cw
  //control servo motors (write.Microseconds)
  yawServo.writeMicroseconds(1500 + output);
  yawServo.writeMicroseconds(1500 + output);
  
}


