/**
 * @file config.h
 * @brief Configuration file for PID Light Tracker parameters
 * @author Rami Kronbi
 * @date 2023
 * 
 * This file contains all configurable parameters for the light tracking system.
 * Modify these values to tune the system performance for your specific hardware setup.
 */

#pragma once

// ===== HARDWARE CONFIGURATION =====

// Servo Motor Pin Assignments
#define PAN_SERVO_PIN    9     
#define PITCH_SERVO_PIN  6    

// LDR Sensor Pin Assignments  
#define TOP_RIGHT_LDR    A0   
#define TOP_LEFT_LDR     A5   
#define BOTTOM_RIGHT_LDR A2   
#define BOTTOM_LEFT_LDR  A3

// ===== SYSTEM PARAMETERS =====

// Servo Configuration
#define SERVO_CENTER_POSITION 90    
#define SERVO_MIN_PULSE      1000   
#define SERVO_MAX_PULSE      2000   

// Communication Settings
#define SERIAL_BAUD_RATE     9600   
#define DEBUG_OUTPUT         false  

// Control Loop Timing
#define LOOP_DELAY_MS        10     
#define SENSOR_READ_DELAY    2 

// ===== PID CONTROLLER PARAMETERS =====

// Pan Controller (Horizontal Movement) - Tuned Parameters
#define PAN_KP              0.075   
#define PAN_KI              0.00045   
#define PAN_KD              0.00035 
#define PAN_OUTPUT_MIN      -90     
#define PAN_OUTPUT_MAX      90      

// Pitch Controller (Vertical Movement) - Tuned Parameters
#define PITCH_KP            0.03    
#define PITCH_KI            0.00018 
#define PITCH_KD            0.00035   
#define PITCH_OUTPUT_MIN    -70     
#define PITCH_OUTPUT_MAX    55      

// ===== SENSOR CONFIGURATION =====

// LDR Sensor Parameters
#define SENSOR_SMOOTHING_FACTOR  0.1
#define SENSOR_DEAD_ZONE        10  
#define SENSOR_CALIBRATION_OFFSET 0 

// Light Detection Thresholds
#define MIN_LIGHT_THRESHOLD     50     
#define MAX_SENSOR_DIFFERENCE   500    

// ===== ADVANCED CONFIGURATION =====

// PID Controller Advanced Settings
#define INTEGRAL_WINDUP_LIMIT   1000  
#define DERIVATIVE_FILTER_ALPHA 0.8    
#define ERROR_DEADBAND          5    

// System Behavior
#define TRACKING_ENABLED        true  
#define ORIENTATION_CORRECTION  true   
#define STARTUP_DELAY_MS        1000  

// Safety Limits
#define MAX_SERVO_SPEED         50     
#define SERVO_PROTECTION_DELAY  20    

// ===== FEATURE TOGGLES =====

#define ENABLE_SENSOR_FILTERING    true   
#define ENABLE_ADAPTIVE_GAINS      false 
#define ENABLE_PREDICTIVE_CONTROL  false  
#define ENABLE_MULTI_TARGET        false  

// ===== DEBUGGING AND MONITORING =====

#define DEBUG_SENSOR_VALUES        false  
#define DEBUG_PID_OUTPUTS          false  
#define DEBUG_SERVO_POSITIONS      false  
#define DEBUG_ERROR_VALUES         false 
#define DEBUG_TIMING               false  

// Debug output format
#define DEBUG_PRINT_INTERVAL       100    
#define DEBUG_PRECISION            2     
