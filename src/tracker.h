/**
 * @file tracker.h
 * @brief PID Controller Classes for Light Tracking Robot
 * @author Rami Kronbi
 * @date 2023
 * 
 * This header file contains the PID controller classes used for controlling
 * the pan and pitch servos in the light tracking robot system.
 */

#pragma once

/**
 * @class PIDController
 * @brief Base PID Controller class for servo motor control
 * 
 * This class implements a standard PID (Proportional-Integral-Derivative) controller
 * with output limiting and integral reset functionality. It's designed to control
 * servo motors for precise positioning in the light tracking system.
 */
class PIDController {
private:
    double kp_;                // Proportional gain
    double ki_;                // Integral gain  
    double kd_;                // Derivative gain
    double integral_error_;    // Accumulated integral error
    double derivative_error_;  // Current derivative error
    double current_error_;     // Current error value
    double previous_error_;    // Previous error for derivative calculation
    double output_min_;        // Minimum output limit
    double output_max_;        // Maximum output limit

public:
    /**
     * @brief Constructor for PID Controller
     * @param kp Proportional gain coefficient
     * @param ki Integral gain coefficient  
     * @param kd Derivative gain coefficient
     * @param output_min Minimum output limit (degrees)
     * @param output_max Maximum output limit (degrees)
     */
    PIDController(double kp, double ki, double kd, double output_min, double output_max) 
        : kp_(kp), ki_(ki), kd_(kd), output_min_(output_min), output_max_(output_max) {
        integral_error_ = 0.0;
        derivative_error_ = 0.0;
        current_error_ = 0.0;
        previous_error_ = 0.0;
    }

    /**
     * @brief Set the current error value for PID calculation
     * @param error Current error between setpoint and feedback
     */
    void setError(double error) {
        current_error_ = error;
    }

    /**
     * @brief Reset the integral error accumulator
     * Used to prevent integral windup in certain conditions
     */
    void resetIntegral() {
        integral_error_ = 0.0;
    }

    /**
     * @brief Calculate PID output based on current error
     * @return Calculated PID output value (clamped to min/max limits)
     */
    double calculatePID() {
        // Calculate derivative error
        derivative_error_ = current_error_ - previous_error_;
        
        // Accumulate integral error
        integral_error_ += current_error_;
        
        // Calculate PID output
        double output = (kp_ * current_error_) + 
                       (ki_ * integral_error_) + 
                       (kd_ * derivative_error_);
        
        // Store current error for next derivative calculation
        previous_error_ = current_error_;
        
        // Clamp output to specified limits
        if (output > output_max_) {
            output = output_max_;
        } else if (output < output_min_) {
            output = output_min_;
        }
        
        return output;
    }

    /**
     * @brief Get current PID parameters for debugging
     * @param kp Reference to store proportional gain
     * @param ki Reference to store integral gain
     * @param kd Reference to store derivative gain
     */
    void getParameters(double& kp, double& ki, double& kd) const {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }
};

/**
 * @class Pan
 * @brief PID Controller specialized for pan (horizontal) servo control
 * 
 * Inherits from PIDController and is specifically tuned for controlling
 * the horizontal rotation servo motor in the light tracking system.
 */
class Pan : public PIDController {
public:
    /**
     * @brief Constructor for Pan controller
     * @param kp Proportional gain (recommended: ~0.075)
     * @param ki Integral gain (recommended: ~0.00045)
     * @param kd Derivative gain (recommended: ~0.00035)
     * @param output_min Minimum pan angle in degrees (recommended: -90)
     * @param output_max Maximum pan angle in degrees (recommended: +90)
     */
    Pan(double kp, double ki, double kd, double output_min, double output_max) 
        : PIDController(kp, ki, kd, output_min, output_max) {}
};

/**
 * @class Pitch
 * @brief PID Controller specialized for pitch (vertical) servo control
 * 
 * Inherits from PIDController and is specifically tuned for controlling
 * the vertical tilt servo motor in the light tracking system.
 */
class Pitch : public PIDController {
public:
    /**
     * @brief Constructor for Pitch controller
     * @param kp Proportional gain (recommended: ~0.03)
     * @param ki Integral gain (recommended: ~0.00018) 
     * @param kd Derivative gain (recommended: ~0.00035)
     * @param output_min Minimum pitch angle in degrees (recommended: -70)
     * @param output_max Maximum pitch angle in degrees (recommended: +55)
     */
    Pitch(double kp, double ki, double kd, double output_min, double output_max) 
        : PIDController(kp, ki, kd, output_min, output_max) {}
};
