//Variables to store the PID gains
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

double PID_control(double error);
void readSensors();

void setup() {
  // Initialize gain values

  /*
  kp = ;
  kd = ;
  ki = ;
  */

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
  output = kp*error + kd*derivError + ki*integError;
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


