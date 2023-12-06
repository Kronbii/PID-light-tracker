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
    else if (output < min) output = min;
    return output;
  }
};

void readSensors(){
  topr = analogRead(A0);
  topl = analogRead(A5); //change to A1
  botr = analogRead(A2);
  botl = analogRead(A3);
}

void do_stuff_idk(){
  //pitch control
  readSensors();
  
  setpoint = (botr + botl)/2.00;
  feedback = (topr + topl)/2.00;
  
  curError = setpoint - feedback;

  output = pitch.calculatePID(curError);
  
  pitchServo.write(90 - output);

  setpoint = (botr + topr)/2;
  feedback = (botl + topl)/2;

  curError = setpoint - feedback;

  output = yaw.calculatePID(curError);
  yawServo.write(90- output);
}


class Pitch : public Yaw{
  public:
  Pitch(double kp, double kd, double ki, double min, double max) : Yaw(kp, kd, ki, min, max) {}
};


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
