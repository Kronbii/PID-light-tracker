class Pitch {
  public:
  double kp, kd, ki;
  double integError, derivError, error, prevError, min, max;

  //constructor
  Pitch(double kp, double kd, double ki, double min, double max){
    //Setting the values of the PID constants
    this->kp = kp;
    this->kd = kd;
    this->ki = ki;

    //Initializing the values of the PID errors
    integError = 0;
    derivError = 0;
    error = 0;
    prevError = 0;

    //Setting the values of the minimum and maximum output
    this->min = min;
    this->max = max;
  }

  double calculatePID(double error){
    double output;
    prevError = this->error;
    this->error = error;
    derivError = error - prevError;
    integError += error;
    output = kp*error + kd*derivError + ki*integError;
    if (output > max) output = max;
    else if (output < min) output = min;
    return output;
  }
};