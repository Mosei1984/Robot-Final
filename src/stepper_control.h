#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <AccelStepper.h>

namespace StepperSystem {
    // Define a structure to hold extended stepper information
    struct EnhancedStepper {
        float stepsPerDegree;
    };
    
    // Declare external variables
    extern AccelStepper* steppers[6];
    extern EnhancedStepper enhancedSteppers[6];
    
    // Function declarations
    void init();
    void update();
    void enableSteppers(bool enable);
    void emergencyStop();
    void disableAllMotors();
    int getSelectedJoint();
    void setSelectedJoint(int joint);
}

#endif // STEPPER_CONTROL_H
