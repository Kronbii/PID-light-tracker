class Pan {
  public:
  double kp, kd, ki;
  double integError, derivError, error, prevError, min, max;

  // constructor
  Pan(double kp, double kd, double ki, double min, double max){
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

  void setError(double error){
    this->error = error;
  }

  double calculatePID(){
    double output;
    derivError = this->error - prevError;
    integError += this->error;
    output = kp*this->error + kd*derivError + ki*integError;
    prevError = this->error;
    if (output > max) output = max;
    else if (output < min) output = min;
    return output;
  }
};

class Pitch : public Pan{
  public:
  Pitch(double kp, double kd, double ki, double min, double max) : Pan(kp, kd, ki, min, max) {}
};
