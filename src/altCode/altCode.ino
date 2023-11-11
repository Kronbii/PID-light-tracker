#include <Servo.h>

class Yaw {
  public:
  double kp, kd, ki;
  double integError, derivError, error, prevError;
  Servo servo;
  int servoPin;

  // constructor
  Yaw(double kp, double kd, double ki, int servoPin){
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    integError = 0;
    derivError = 0;
    error = 0;
    prevError = 0;
    this->servoPin = servoPin;
    servo.attach(servoPin);
  }

  double calculatePID(double error){
    double output;
    this->error = error;
    derivError = this->error + prevError;
    integError += this->error;
    prevError = this->error;
    return output = kp*this->error + kd*derivError + ki*integError;
  }

  void controlServo(double output){
    output = int(output);
    servo.writeMicroseconds(1500 + output);
  }
};

void readSensors();

class Pitch : public Yaw{
  public:
  Pitch(double kp, double kd, double ki, int servoPin) : Yaw(kp, kd, ki, servoPin) {}
};
//////////////////////////////Sensor values/////////////////////////////////
double s1,s2,s3,s4;
double setpoint, feedback;
double output;
////////////////////////////////////////////////////////////////////////////

Yaw yaw(1, 1, 1, 9);
Pitch pitch(1, 1, 1, 11);


void readSensors();

void setup() {
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);

  yaw.controlServo(0);
  pitch.controlServo(0);

  setpoint = 0;
  feedback = 0;
}

void loop() {
  readSensors();

  //update error values for yaw
  if (s1 > s2){
    setpoint = s1;
    feedback = s2;
  }
  else if(s1 < s2){
    setpoint = s2;
    feedback = s1;
  }

  if (s3 > s4){
    setpoint += s3;
    feedback += s4;
  }
  else if(s3 < s4){
    setpoint += s4;
    feedback += s3;
  }

  yaw.prevError = yaw.error;
  yaw.error = setpoint - feedback;

  output = yaw.calculatePID(yaw.error);
  yaw.controlServo(output);

  readSensors();

  setpoint = s3 + s4;
  feedback = s1 + s2;
  pitch.prevError = pitch.error;
  pitch.error = setpoint - feedback;

  output = pitch.calculatePID(pitch.error);
  pitch.controlServo(output);

}

void readSensors(){
  s1 = analogRead(A0);
  s2 = analogRead(A1);
  s3 = analogRead(A2);
  s4 = analogRead(A3);
}