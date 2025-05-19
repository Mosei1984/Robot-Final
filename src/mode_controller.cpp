#include "mode_controller.h"
#include "Debug.h"
#include "display_system.h"
#include "joystick_system.h"
#include "stepper_control.h"
#include "robot_system.h"

namespace ModeController {
    // Define the variables
    Mode currentMode = MODE_NORMAL;
    bool buttonWasPressed = false;
    unsigned long buttonPressTime = 0;
    int debugState = 0;
    JoystickValues joystickValues = {0};
    
    // Initialization function
    void init() {
        // Initialize mode switch pin
        pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
        
        // Set initial mode
        currentMode = MODE_NORMAL;
        
        // Initialize ADXL if needed
        initADXL();
        
        // Show startup animation
        showAnimation();
    }
    
    // Update function called regularly
    void update() {
        // Update joystick values from the joystick system
        joystickValues = JoystickSystem::getJoystickValues();
        
        // Check if mode switch button is pressed
        if (digitalRead(MODE_SWITCH_PIN) == LOW) {
            static unsigned long lastPressTime = 0;
            if (millis() - lastPressTime > BUTTON_PRESS_TIME) {
                switchMode();
                lastPressTime = millis();
            }
        }
        
        // Update system state based on current mode
        updateSystemState();
    }
    
    // Switch to next mode in sequence
    void switchMode() {
        // Cycle through available modes
        int nextMode = (static_cast<int>(currentMode) + 1) % 6; // Assuming 6 modes
        currentMode = static_cast<Mode>(nextMode);
        
        Debug::print(F("Switched to mode: "));
        Debug::println(static_cast<int>(currentMode));
    }
    
    // Get current mode
    Mode getCurrentMode() {
        return currentMode;
    }
    
    // Show animation on display
    void showAnimation() {
        // Show transition animation on display
        // Implementation depends on your display system
    }
    
    // Initialize ADXL accelerometer if used
    void initADXL() {
        // Initialize accelerometer code here
        // Implementation depends on your hardware
    }
    
    // Set specific mode
    void setMode(Mode mode) {
        currentMode = mode;
    }
    
    // Update the system state based on current mode
    void updateSystemState() {
        
        
        // Display differently based on mode
        switch (currentMode) {
            case MODE_NORMAL:
                // For RobotKinematics conversion issues, use only RobotSystem functions
                // Don't try to directly use or convert RobotKinematics pointers
                // Example:
                // RobotSystem::displayRobotInfo();
                break;
                
            case MODE_TEACHING:
                // Teaching mode code
                break;
                
            case MODE_PLAYBACK:
                // Playback mode code
                break;
                
            case MODE_DEBUG:
                // Debug mode code - fix the displayHomingMenu call
                DisplaySystem::displayHomingMenu(
                    RobotSystem::getHomingMenuSelection(), 
                    RobotSystem::getHomingMenuOptionCount()
                );
                break;
                
            case MODE_CONFIG:
                // Config mode code
                break;
                
            case MODE_DEBUG_ADVANCED:
                // Use correct function name
                DisplaySystem::displayDebugScreen();
                break;
                
            default:
                break;
        }
    }
}
