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

  void controlServo(){
    output = calculatePID();
    output = int(output);
    servo.writeMicroseconds(1500 + output);
  }
};

class Pitch : public Yaw{
};


//////////////////////////////Sensor values/////////////////////////////////
double s1,s2,s3,s4;
////////////////////////////////////////////////////////////////////////////

void readSensors();

void setup() {

  Yaw yaw(1, 1, 1, 9);
  Pitch pitch(1, 1, 1, 11);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void readSensors(){
  s1 = analogRead(A0);
  s2 = analogRead(A1);
  s3 = analogRead(A2);
  s4 = analogRead(A3);
}