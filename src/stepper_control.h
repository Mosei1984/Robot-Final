#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"

namespace StepperSystem {
    // Initialize stepper motors
    void init();
    
    // Update stepper motors (should be called in main loop)
    void update();
    
    // Test stepper motor functionality
    void testSteppers();
    
    // Joint selection for manual control
    int getSelectedJoint();
    void setSelectedJoint(int joint);
    
    // Home a specific joint
    bool homeJoint(int jointIndex);
    
    // Enable/disable motors
    void enableAllMotors();
    void disableAllMotors();
    
    // Access the stepper motors
    extern AccelStepper* steppers[6];
}

#endif // STEPPER_CONTROL_H
