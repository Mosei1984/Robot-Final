#include "joystick_system.h"
#include "Debug.h"
#include "config.h"
#include "robot_system.h"

// Extern-Referenzen
extern JoystickConfig _joystickConfig;

namespace JoystickSystem {
    // Private Variablen
    static Joystick* leftJoystick = nullptr;
    static Joystick* rightJoystick = nullptr;
    static JoystickValues currentJoystickValues;
    static bool leftButtonPressed = false;
    static bool rightButtonPressed = false;
    
    // Kalmanfilter für geglättete Bewegungen
    static KalmanFilter positionFilterX(0.01, 0.1);
    static KalmanFilter positionFilterY(0.01, 0.1);
    static KalmanFilter positionFilterZ(0.01, 0.1);
    static KalmanFilter rotationFilter(0.01, 0.1);
    
    void init() {
        Debug::println(F("Initializing joystick system..."));
        
        // Joystick-Objekte erstellen
        leftJoystick = new Joystick(_joystickConfig.leftXPin, _joystickConfig.leftYPin, _joystickConfig.leftBtnPin);
        rightJoystick = new Joystick(_joystickConfig.rightXPin, _joystickConfig.rightYPin, _joystickConfig.rightBtnPin);
        
        // Joysticks initialisieren
        leftJoystick->begin();
        rightJoystick->begin();
        
        // Grundkalibrierung durchführen
        calibrateJoysticks();
        
        Debug::println(F("Joystick system initialized"));
    }
    
    void update() {
        // Joystick-Werte aktualisieren
        readValues();
        
        // Button-Verarbeitung erfolgt in der aufrufenden Funktion (ModeController)
    }
    
    JoystickValues readValues() {
        if (leftJoystick && rightJoystick) {
            leftJoystick->read();
            rightJoystick->read();
            
            JoystickValues values;
            values.leftX = leftJoystick->getNormalizedX();
            values.leftY = leftJoystick->getNormalizedY();
            values.rightX = rightJoystick->getNormalizedX();
            values.rightY = rightJoystick->getNormalizedY();
            values.leftButton = leftJoystick->isPressed();
            values.rightButton = rightJoystick->isPressed();
            
            currentJoystickValues = values;
            return values;
        }
        
        // Falls Joysticks nicht initialisiert wurden
        JoystickValues emptyValues = {0};
        return emptyValues;
    }
    
    Joystick* getLeftJoystick() {
        return leftJoystick;
    }
    
    Joystick* getRightJoystick() {
        return rightJoystick;
    }
    
    void calibrateJoysticks() {
        Debug::println(F("Basic joystick calibration..."));
        
        if (leftJoystick) leftJoystick->calibrate();
        if (rightJoystick) rightJoystick->calibrate();
    }
    
    void startFullCalibration() {
        Debug::println(F("Starting full joystick calibration..."));
        
        if (leftJoystick) leftJoystick->startCalibration();
        if (rightJoystick) rightJoystick->startCalibration();
        
        Debug::println(F("Calibration complete"));
    }
    
    void processButtonInput() {
        if (!leftJoystick || !rightJoystick) return;
        
        bool leftPressed = leftJoystick->isPressed();
        bool rightPressed = rightJoystick->isPressed();
        
        // Button-Status aktualisieren und Flanken erkennen
        if (leftPressed && !leftButtonPressed) {
            leftButtonPressed = true;
            // Hier können Aktionen für die steigende Flanke des linken Buttons eingebaut werden
        } else if (!leftPressed && leftButtonPressed) {
            leftButtonPressed = false;
        }
        
        if (rightPressed && !rightButtonPressed) {
            rightButtonPressed = true;
            // Hier können Aktionen für die steigende Flanke des rechten Buttons eingebaut werden
        } else if (!rightPressed && rightButtonPressed) {
            rightButtonPressed = false;
        }
    }
    
    JoystickValues getJoystickValues() {
        return currentJoystickValues;
    }
    
    bool isLeftButtonPressed() {
        return leftButtonPressed;
    }
    
    bool isRightButtonPressed() {
        return rightButtonPressed;
    }
}
