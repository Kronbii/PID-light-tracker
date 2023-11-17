#include <Servo.h>

Servo panServo;
Servo tiltServo;

int ldrPin1 = A0;
int ldrPin2 = A1;
int ldrPin3 = A2;
int ldrPin4 = A3;

int panSetpoint = 90;
int tiltSetpoint = 90;

double Kp = 0.5;
double Ki = 0.001;
double Kd = 0.3;

double previousErrorPan = 0;
double integralPan = 0;
double previousErrorTilt = 0;
double integralTilt = 0;

void setup() {
  panServo.attach(5);
  tiltServo.attach(6);

  pinMode(ldrPin1, INPUT);
  pinMode(ldrPin2, INPUT);
  pinMode(ldrPin3, INPUT);
  pinMode(ldrPin4, INPUT);

  Serial.begin(9600);
}

void loop() {
  int ldrValue1 = analogRead(ldrPin1);
  int ldrValue2 = analogRead(ldrPin2);
  int ldrValue3 = analogRead(ldrPin3);
  int ldrValue4 = analogRead(ldrPin4);

  int errorPan = calculateError(ldrValue1, ldrValue2);
  int errorTilt = calculateError(ldrValue3, ldrValue4);

  integralPan += errorPan;
  integralTilt += errorTilt;

  double outputPan = Kp * errorPan + Ki * integralPan + Kd * (errorPan - previousErrorPan);
  double outputTilt = Kp * errorTilt + Ki * integralTilt + Kd * (errorTilt - previousErrorTilt);

  panSetpoint += outputPan;
  tiltSetpoint += outputTilt;

  panSetpoint = constrain(panSetpoint, 0, 180);
  tiltSetpoint = constrain(tiltSetpoint, 0, 180);

  panServo.write(panSetpoint);
  tiltServo.write(tiltSetpoint);

  previousErrorPan = errorPan;
  previousErrorTilt = errorTilt;

  // Output data in CSV format
  /*
  Serial.print(ldrValue1);
  Serial.print(",");
  Serial.print(ldrValue2);
  Serial.print(",");
  Serial.print(ldrValue3);
  Serial.print(",");
  Serial.print(ldrValue4);
  Serial.print(",");
  Serial.print(errorPan);
  Serial.print(",");
  */
  Serial.print(outputPan);
  Serial.println("");
  /*
  Serial.print(panSetpoint);
  Serial.print(",");
  Serial.print(errorTilt);
  Serial.print(",");
  /*
  Serial.print(outputTilt);
  Serial.print(",");
  Serial.println(tiltSetpoint);
  */

  delay(100);
}

int calculateError(int ldrValue1, int ldrValue2) {
  return ldrValue1 - ldrValue2;
}