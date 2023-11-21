#include <Servo.h>

Servo panServo;
Servo tiltServo;

void setup() {
  Serial.begin(9600);
  panServo.attach(6);   // Attach pan servo to pin 9
  tiltServo.attach(10); // Attach tilt servo to pin 10
}

void loop() {
  int sensorValues[4];
  float voltages[4];
  float lightIntensities[4];

  for (int i = 2; i < 6; i++) {
    sensorValues[i - 2] = analogRead(i);
    voltages[i - 2] = sensorValues[i - 2] * (5.0 / 1023.0);
    lightIntensities[i - 2] = -20.235 * voltages[i - 2] + 101.175;

    Serial.print("LDR ");
    Serial.print(i - 1);
    Serial.print(" - V: ");
    Serial.print(sensorValues[i - 2]);
    Serial.print(" - I: ");
    Serial.print(lightIntensities[i - 2], 2);
    Serial.print("%");
    Serial.println("");
  }
      Serial.println(" ");
   delay(2000);

  // Calculate the difference between adjacent light intensities
  float intensityDiff1 = lightIntensities[1] - lightIntensities[0];
  float intensityDiff2 = lightIntensities[3] - lightIntensities[2];

  // Adjust servo positions based on the intensity differences
  int panAngle = map(intensityDiff1, -50, 50, -90, 90);
  int tiltAngle = map(intensityDiff2, -50, 50, -90, 90);
  panServo.write(90 + panAngle);   // Adjust center position to 90 degrees
  tiltServo.write(90 + tiltAngle); // Adjust center position to 90 degrees

  delay(1000);
}