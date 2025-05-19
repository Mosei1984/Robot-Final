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
    
    // Initialize display via DisplaySystem
    DisplaySystem::init();
    
    // Initialize ADXL345 accelerometer for future stability control
    initADXL();
    
    
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
void ModeController::updateSystemState() {
    // Aktualisiere den Roboter-Systemzustand basierend auf dem aktuellen Controller-Modus
    if (RobotSystem::getCurrentState() == NORMAL_OPERATION) {
        // Display je nach Modus unterschiedlich aktualisieren
        switch (currentMode) {
            case MODE_JOINT:
                // Joint-Modus-Display verwenden
                DisplaySystem::displayJointMode(
                    RobotSystem::getKinematics(),
                    StepperSystem::getSelectedJoint()
                );
                
                // Joysticks für Joint-Steuerung verarbeiten
                JoystickSystem::processJointModeJoysticks();
                break;
                
            case MODE_POSITION_ONLY:
                // Positions-Modus-Display verwenden
                DisplaySystem::displayPositionMode(RobotSystem::getKinematics());
                
                // Joysticks für Position-Steuerung verarbeiten
                JoystickSystem::processKinematicModeJoysticks();
                break;
                
            case MODE_FULL_POSE:
                // Vollständiges Pose-Display verwenden
                DisplaySystem::displayFullPoseMode(RobotSystem::getKinematics());
                
                // Joysticks für vollständige Pose-Steuerung verarbeiten
                JoystickSystem::processKinematicModeJoysticks();
                break;
                
            case MODE_TEACHING:
                // Teaching-Modus-Display verwenden
                DisplaySystem::displayTeachingMode();
                break;
                
            case MODE_HOMING:
                // Homing-Modus-Display verwenden
                if (RobotSystem::isHomingStarted()) {
                    DisplaySystem::displayHomingProgress(RobotSystem::getHomingJointIndex());
                } else {
                    DisplaySystem::displayHomingMenu();
                    JoystickSystem::processMenuJoysticks();
                }
                break;
                
            case MODE_SETTINGS:
                // Einstellungsmenü-Display verwenden
                DisplaySystem::displaySettingsMenu();
                JoystickSystem::processMenuJoysticks();
                break;
                
            case MODE_DEBUG:
                // Debug-Display verwenden
                DisplaySystem::displayDebugInfo();
                break;
        }
    }
    // Andere Systemzustände (HOMING, CALIBRATION, usw.) werden außerhalb verarbeitet
}
