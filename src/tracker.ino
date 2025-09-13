/**
 * @file tracker.ino
 * @brief Main Arduino sketch for PID-controlled light tracking robot
 * @author Rami Kronbi
 * @date 2023
 * 
 * This sketch implements a dual-axis light tracking system using PID control.
 * The robot uses four LDR sensors arranged in a cross pattern to detect light
 * direction and adjusts two servo motors (pan and pitch) to track the light source.
 * 
 * Configuration:
 * - All system parameters, PID gains, and pin assignments are defined in config.h
 * - Modify config.h to tune PID performance or change hardware configuration
 * - Multiple preset configurations available in config.h
 * 
 * Hardware Requirements:
 * - Arduino Uno/Nano
 * - 2x Servo Motors (MG995 or similar)
 * - 4x LDR (Light Dependent Resistor) sensors
 * - 4x 10kΩ pull-down resistors for LDRs
 * - Power supply for servos (6V recommended)
 * 
 * Pin Configuration (defined in config.h):
 * - A0: Top-Right LDR sensor
 * - A5: Top-Left LDR sensor  
 * - A2: Bottom-Right LDR sensor
 * - A3: Bottom-Left LDR sensor
 * - Pin 9: Pan servo (horizontal rotation)
 * - Pin 6: Pitch servo (vertical tilt)
 */

// ===== LIBRARIES =====
#include "config.h"
#include "tracker.h"
#include <Servo.h>

// ===== HARDWARE CONFIGURATION =====
Servo panServo;                  
Servo pitchServo;               

// ===== SENSOR VARIABLES =====
double bottom_right_sensor = 0;  
double bottom_left_sensor = 0;   
double top_right_sensor = 0;     
double top_left_sensor = 0;   

// ===== CONTROL VARIABLES =====
double setpoint = 0;            
double feedback = 0;            
double pid_output = 0;          
double current_error = 0;        
bool system_orientation = false;

// ===== PID CONTROLLER INSTANCES =====
Pan pan_controller(PAN_KP, PAN_KI, PAN_KD, PAN_OUTPUT_MIN, PAN_OUTPUT_MAX);    
Pitch pitch_controller(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_OUTPUT_MIN, PITCH_OUTPUT_MAX); 


void setup() {
  // Initialize serial communication for debugging
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("=== PID Light Tracker Initialization ===");
  
  // Configure servo control pins as outputs
  pinMode(PAN_SERVO_PIN, OUTPUT);
  pinMode(PITCH_SERVO_PIN, OUTPUT);
  
  // Initialize control system variables
  setpoint = 0.0;
  feedback = 0.0;
  current_error = 0.0;
  system_orientation = false;
  
  // Attach servo motors to their respective pins
  panServo.attach(PAN_SERVO_PIN);
  pitchServo.attach(PITCH_SERVO_PIN);
  
  // Set servos to center position for initialization
  panServo.write(SERVO_CENTER_POSITION);
  pitchServo.write(SERVO_CENTER_POSITION);
  
  // Allow time for servos to reach initial position
  delay(1000);
  
  Serial.println("Initialization complete. Starting light tracking...");
  Serial.println("Sensor Layout:");
  Serial.println("  [TL] [TR]");
  Serial.println("  [BL] [BR]");
  Serial.println();
}


void loop() {
  // ===== PITCH CONTROL (VERTICAL TRACKING) =====
  readSensors();
  
  // Calculate pitch error: compare bottom sensors (setpoint) vs top sensors (feedback)
  // If bottom sensors receive more light, the system should tilt down (positive error)
  // If top sensors receive more light, the system should tilt up (negative error)
  setpoint = (bottom_right_sensor + bottom_left_sensor) / 2.0;
  feedback = (top_right_sensor + top_left_sensor) / 2.0;
  
  // Calculate error and update PID controller
  current_error = setpoint - feedback;
  pitch_controller.setError(current_error);
  
  // Get PID output for pitch control
  pid_output = pitch_controller.calculatePID();
  
  // Determine system orientation based on pitch output
  if (pid_output > 0) {
    system_orientation = true;   // System tilted forward [180°-360° hemisphere]
  } else if (pid_output < 0) {
    system_orientation = false;  // System tilted backward [0°-180° hemisphere]
  }
  
  // Apply pitch correction to servo
  // Servo position = center ± PID output
  pitchServo.write(SERVO_CENTER_POSITION - (int)pid_output);
  
  // ===== PAN CONTROL (HORIZONTAL TRACKING) =====
  readSensors();
  
  setpoint = (bottom_right_sensor + top_right_sensor) / 2.0;
  feedback = (bottom_left_sensor + top_left_sensor) / 2.0;
  
  // Calculate error and update PID controller
  current_error = setpoint - feedback;
  pan_controller.setError(current_error);
  
  // Get PID output for pan control
  pid_output = pan_controller.calculatePID();
  
  if (system_orientation == true) {
    pid_output = -pid_output;
  }
  
  // Apply pan correction to servo
  panServo.write(SERVO_CENTER_POSITION - (int)pid_output);
  
  // Small delay to prevent overwhelming the servos
  delay(LOOP_DELAY_MS);
}

void readSensors() {
  // Read all four LDR sensors
  top_right_sensor = analogRead(TOP_RIGHT_LDR);    // A0 - Top-Right sensor
  top_left_sensor = analogRead(TOP_LEFT_LDR);      // A5 - Top-Left sensor  
  bottom_right_sensor = analogRead(BOTTOM_RIGHT_LDR); // A2 - Bottom-Right sensor
  bottom_left_sensor = analogRead(BOTTOM_LEFT_LDR);   // A3 - Bottom-Left sensor
}

