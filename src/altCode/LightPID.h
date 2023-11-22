#include <Servo.h>
//////////////COUDL FIND INTENSITY AND DIVIDE IT BY SENSOR READINGS TO GET BETTER CALIBRATION//////////////////
class Yaw {
  public:
  double kp, kd, ki;
  double integError, derivError, error, prevError, min, max;

  // constructor
  Yaw(double kp, double kd, double ki, double min, double max){
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;
    integError = 0;
    derivError = 0;
    error = 0;
    prevError = 0;
    this->min = min;
    this->max = max;
  }

  double calculatePID(double error){
    double output;
    prevError = this->error;
    this->error = error;
    derivError = this->error - prevError;
    integError += this->error;
    output = kp*this->error + kd*derivError + ki*integError;
    if (output > max) output = max;
    if (output < min) output = min;
    return output;
  }
};


class Pitch : public Yaw{
  public:
  Pitch(double kp, double kd, double ki) : Yaw(kp, kd, ki) {}
};
