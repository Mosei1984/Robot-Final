#ifndef STEPPER_SYSTEM_H
#define STEPPER_SYSTEM_H

#include <Arduino.h>
#include <AccelStepper.h>

// Constants
#define NUM_JOINTS 6

// Enhanced stepper structure with additional configuration
struct EnhancedStepper {
    AccelStepper* stepper;
    float stepsPerDegree;
    float homingSpeed;
    float maxSpeed;
    float acceleration;
    int limitSwitchPin;
    int enablePin;
};

namespace StepperSystem {
    // External variables
    extern AccelStepper* steppers[NUM_JOINTS];
    extern EnhancedStepper enhancedSteppers[NUM_JOINTS];
    
    // Functions
    void init();
    void update();
    void runSteppers();
    
    // Joint selection functions
    int getSelectedJoint();
    void setSelectedJoint(int joint);
    
    // Motor control functions
    void enableAllMotors();
    void disableAllMotors();
}

#endif // STEPPER_SYSTEM_H