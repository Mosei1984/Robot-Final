#ifndef JOYSTICK_SYSTEM_H
#define JOYSTICK_SYSTEM_H

#include <Arduino.h>
#include "joystick.h"        // Include your existing Joystick class
#include "joystick_types.h"  // Include existing JoystickValues structure

// Only include the namespace part, not the class definitions
namespace JoystickSystem {
    void init();
    void update();
    
    Joystick* getLeftJoystick();
    Joystick* getRightJoystick();
    
    // Calibration functions
    void startCalibration();
    void processCalibration();
    bool isCalibrating();
    
    // Process joystick inputs for different modes
    void processJointModeJoysticks();
    void processKinematicModeJoysticks();
    
    // Get current joystick values in normalized format
    // Use the existing JoystickValues structure from joystick_types.h
    JoystickValues getJoystickValues();
}

#endif // JOYSTICK_SYSTEM_H
