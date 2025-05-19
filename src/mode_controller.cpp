#include "mode_controller.h"
#include "display_system.h"
#include "joystick_system.h"
#include "robot_system.h"
#include <Wire.h>

// Initialize static members
ModeController::Mode ModeController::currentMode = MODE_JOINT;
bool ModeController::buttonWasPressed = false;
unsigned long ModeController::buttonPressTime = 0;
bool ModeController::debugState = false;

void ModeController::init() {
    // Configure mode switch and debug pins
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(DEBUG_MODE_PIN, OUTPUT);
    digitalWrite(DEBUG_MODE_PIN, LOW);
    
    // Initialize display via DisplaySystem
    DisplaySystem::init();
    
    // Initialize ADXL345 accelerometer for future stability control
    initADXL();
    
    // Initialize watchdog
    setupWatchdog();
    
    // Display welcome animation
    showAnimation();
}

void ModeController::update() {
    // Read joystick values
    _joystickConfig = JoystickConfig::readValues();
    
    // Check for mode change (using right button)
    if (joystickValues.rightButton && !buttonWasPressed) {
        // Button pressed - record time
        buttonPressTime = millis();
        buttonWasPressed = true;
    } else if (joystickValues.rightButton && buttonWasPressed) {
        // Button is still pressed - check if it's been held long enough
        if ((millis() - buttonPressTime) > BUTTON_PRESS_TIME) {
            // Switch mode
            switchMode();
            
            // Reset button state to avoid multiple mode changes
            buttonWasPressed = false;
        }
    } else if (!joystickValues.rightButton && buttonWasPressed) {
        // Button released without mode change
        buttonWasPressed = false;
    }
    
    // Toggle debug pin to assist with oscilloscope debugging
    toggleDebugPin();
    
    // Update system state based on current mode
    updateSystemState();
}

void ModeController::switchMode() {
    // Cycle through available modes
    currentMode = static_cast<Mode>((currentMode + 1) % 3);
    
    // Show the new mode on the display
    DisplaySystem::showMessage(
        "Mode changed",
        currentMode == MODE_JOINT ? "Joint Control" :
        (currentMode == MODE_POSITION_ONLY ? "Position Control" : "Full Control"),
        1000
    );
    
    Serial.print(F("Mode changed to: "));
    Serial.println(currentMode);
}

Mode ModeController::getCurrentMode() {
    return currentMode;
}

void ModeController::showAnimation() {
    // Simple animation to show on startup
    DisplaySystem::showMessage("6DOF Robot", "Ready", 1500);
}

void ModeController::initADXL() {
    // Initialize I2C
    Wire.begin();
    
    // Future implementation of accelerometer for stability control
}

void ModeController::setupWatchdog() {
    // Future implementation of watchdog timer
}

void ModeController::toggleDebugPin() {
    // Toggle debug pin state for oscilloscope monitoring
    debugState = !debugState;
    digitalWrite(DEBUG_MODE_PIN, debugState);
}

void ModeController::updateSystemState() {
    // Update the robot system state based on the current controller mode
    if (RobotSystem::getCurrentState() == NORMAL_OPERATION) {
        // Display differs based on mode
        switch (currentMode) {
            case MODE_JOINT:
                // Use joint mode display
                DisplaySystem::displayJointMode(
                
                    RobotSystem::getKinematics(),
                    StepperSystem::getSelectedJoint()
                );
                break;
                
            case MODE_POSITION_ONLY:
                // Use position mode display
                DisplaySystem::displayPositionMode(RobotSystem::getKinematics());
                break;
                
            case MODE_POSITION_ORIENTATION:
                // Use kinematic mode display
                DisplaySystem::displayKinematicMode(RobotSystem::getKinematics());
                break;
        }
    }
}
