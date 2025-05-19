#ifndef MODE_CONTROLLER_H
#define MODE_CONTROLLER_H

#include <Arduino.h>
#include "joystick_types.h" // Use the shared type definition

namespace ModeController {
    // Define the Mode enum
    enum Mode {
        MODE_NORMAL = 0,
        MODE_TEACHING = 1,
        MODE_PLAYBACK = 2,
        MODE_DEBUG = 3,
        MODE_CONFIG = 4,
        MODE_DEBUG_ADVANCED = 5
    };
    
    // Constants
    const unsigned long BUTTON_PRESS_TIME = 500;  // ms
    #define MODE_SWITCH_PIN 33  // Define this as a macro
    
    // Function declarations
    void init();
    void update();
    void switchMode();
    void showAnimation();
    void initADXL();
    void setMode(Mode mode);
    Mode getCurrentMode();
    void updateSystemState();
    
    // External variables
    extern Mode currentMode;
    extern bool buttonWasPressed;
    extern unsigned long buttonPressTime;
    extern int debugState;
    extern JoystickValues joystickValues;
}

#endif // MODE_CONTROLLER_H
