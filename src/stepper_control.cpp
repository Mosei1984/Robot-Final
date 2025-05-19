#include "stepper_control.h"
#include "config.h"
#include "Debug.h"

namespace StepperSystem {
    // Define the variables
    AccelStepper* steppers[6];
    EnhancedStepper enhancedSteppers[6];
    
    // Add these implementations
    int selectedJoint = 0;
    
    void disableAllMotors() {
        for(int i = 0; i < 6; i++) {
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);  // Disable (HIGH = disabled)
            steppers[i]->disableOutputs();
        }
        Debug::println(F("All stepper motors disabled"));
    }
    
    int getSelectedJoint() {
        return selectedJoint;
    }
    
    void setSelectedJoint(int joint) {
        if (joint >= 0 && joint < 6) {
            selectedJoint = joint;
        }
    }
    
    // Initialize steppers from config
    void init() {
        Debug::println(F("Initializing stepper motors..."));
        
        // Initialize steppers based on the pin configuration
        for(int i = 0; i < 6; i++) {
            steppers[i] = new AccelStepper(
                AccelStepper::DRIVER,
                _pinConfig.stepperPins[i][0],  // Step pin
                _pinConfig.stepperPins[i][1]   // Direction pin
            );
            
            // Configure stepper parameters from configuration
            steppers[i]->setMaxSpeed(_stepperConfig[i].maxSpeed);
            steppers[i]->setAcceleration(_stepperConfig[i].acceleration);
            
            // Initialize enhanced stepper data
            enhancedSteppers[i].stepsPerDegree = _stepperConfig[i].stepsPerDegree;
            
            // Enable stepper motors (active low)
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);  // Initially disabled
            
            Debug::print(F("Stepper "));
            Debug::print(i);
            Debug::print(F(" initialized with "));
            Debug::print(enhancedSteppers[i].stepsPerDegree);
            Debug::println(F(" steps per degree"));
        }
    }
    
    // Update all steppers
    void update() {
        for(int i = 0; i < 6; i++) {
            steppers[i]->run();
        }
    }
    
    // Enable or disable all steppers
    void enableSteppers(bool enable) {
        for(int i = 0; i < 6; i++) {
            digitalWrite(_pinConfig.stepperPins[i][2], enable ? LOW : HIGH);
        }
    }
    
    // Emergency stop all steppers
    void emergencyStop() {
        for(int i = 0; i < 6; i++) {
            steppers[i]->stop();
            steppers[i]->setSpeed(0);
            steppers[i]->disableOutputs();
        }
    }
}