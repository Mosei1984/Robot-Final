#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <AccelStepper.h>

// ======================== System States ========================
enum SystemState {
    IDLE = 0,
    HOMING,
    NORMAL_OPERATION,
    RECORD,
    PLAY,
    GCODE,
    CALIBRATION,
    CONFIG
};

// Current system state
extern SystemState currentState;

// Function to change state
void changeState(SystemState newState);

// ======================== Display Modes ========================
enum DisplayMode {
    DISPLAY_SPLASH,
    DISPLAY_JOINTS,
    DISPLAY_GCODE,
    DISPLAY_RECORD,
    DISPLAY_MENU,
    DISPLAY_HOMING,
    DISPLAY_ERROR,
    DISPLAY_CONFIG
};

extern DisplayMode currentDisplayMode;
void setDisplayMode(DisplayMode mode);

// ======================== EEPROM Storage ========================
#define EEPROM_MAGIC_VALUE 0x42
#define EEPROM_ADDR_MAGIC 0
#define EEPROM_ADDR_CALIBRATION 10
#define EEPROM_ADDR_HOMING 100
#define ROBOT_HOME_MAGIC 0x42ABCDEF

// ======================== Configuration Management ========================
namespace ConfigSystem {
    void init();
    bool saveConfig(const char* filename);
    bool loadConfig(const char* filename);
    void loadDefaultPinConfig();
    void loadDefaultJoystickConfig();
    void loadDefaultStepperConfig();
    void displayConfigMenu();
    void processConfigMenu();
}

// ======================== Hardware Configuration Structures ========================
struct PinConfig {
    // Joystick pins
    int leftXPin;
    int leftYPin;
    int leftBtnPin;
    int rightXPin;
    int rightYPin;
    int rightBtnPin;
    
    // Stepper pins [joint][function] where function is STEP, DIR, ENABLE, LIMIT
    int stepperPins[6][4];
    
    // LED pins
    int errorLedPin;
    
    // I2C pins
    int oledSdaPin;
    int oledSclPin;
};

struct JoystickConfig {
    int deadband;
    float sensitivity;
};

struct StepperConfig {
    float stepsPerDegree;
    float maxSpeed;
    float acceleration;
    float homingSpeed;
    float minPosition;
    float maxPosition;
};

// External declarations for global configuration variables
extern PinConfig _pinConfig;
extern JoystickConfig _joystickConfig;
extern StepperConfig _stepperConfig[6];
extern AccelStepper* _steppers[6]; // Add this missing external declaration

#endif // CONFIG_H
