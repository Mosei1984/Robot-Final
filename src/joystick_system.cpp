#include "joystick_system.h"
#include "Debug.h"
#include "config.h"
#include "robot_system.h"
#include "stepper_control.h"

// Joystick-Zustände für die Erkennung von Richtungsänderungen
enum JoystickState {
    JOYSTICK_CENTERED,
    JOYSTICK_LEFT,
    JOYSTICK_RIGHT,
    JOYSTICK_UP,
    JOYSTICK_DOWN
};

// Externes Konfigurationsobjekt
extern JoystickConfig _joystickConfig;

namespace JoystickSystem {
    // Private Variablen
    static Joystick* leftJoystick = nullptr;
    static Joystick* rightJoystick = nullptr;
    static JoystickValues currentJoystickValues;
    static bool leftButtonPressed = false;
    static bool rightButtonPressed = false;
    
    // Joystick-Zustandsvariablen
    static JoystickState joystickXState = JOYSTICK_CENTERED;
    static JoystickState joystickYState = JOYSTICK_CENTERED;
    static unsigned long lastJoystickMove = 0;
    
    // Kalmanfilter für geglättete Bewegungen
    static KalmanFilter positionFilterX(0.01, 0.1);
    static KalmanFilter positionFilterY(0.01, 0.1);
    static KalmanFilter positionFilterZ(0.01, 0.1);
    static KalmanFilter rotationFilter(0.01, 0.1);
    
        void init() {
        Debug::println(F("Initializing joystick system..."));
        
        // Joystick-Objekte erstellen
        leftJoystick = new Joystick(_pinConfig.leftXPin, _pinConfig.leftYPin, _pinConfig.leftBtnPin);
        rightJoystick = new Joystick(_pinConfig.rightXPin, _pinConfig.rightYPin, _pinConfig.rightBtnPin);
        
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
    
    void processJointModeJoysticks() {
        if (!leftJoystick || !rightJoystick) return;
        
        // Debug-Ausgabe der Joystick-Werte
        Debug::print(F("Joystick values: Left X: "));
        Debug::print(leftJoystick->getX());
        Debug::print(F(", Left Y: "));
        Debug::println(leftJoystick->getY());
        
        // Gelenkauswahl mit rechtem Joystick X
        float rightX = (rightJoystick->getX() - 500.0f) / 500.0f;  // auf [-1..+1]
        
        if (rightX > 0.7f && joystickXState == JOYSTICK_CENTERED) {
            joystickXState = JOYSTICK_RIGHT;
            int selectedJoint = StepperSystem::getSelectedJoint();
            if (selectedJoint < 5) {
                StepperSystem::setSelectedJoint(selectedJoint + 1);
                Debug::print(F("Joint increased to: "));
                Debug::println(selectedJoint + 1);
            }
        }
        else if (rightX < -0.7f && joystickXState == JOYSTICK_CENTERED) {
            joystickXState = JOYSTICK_LEFT;
            int selectedJoint = StepperSystem::getSelectedJoint();
            if (selectedJoint > 0) {
                StepperSystem::setSelectedJoint(selectedJoint - 1);
                Debug::print(F("Joint decreased to: "));
                Debug::println(selectedJoint - 1);
            }
        }
        else if (fabs(rightX) < 0.4f) {
            joystickXState = JOYSTICK_CENTERED;
        }
        
        // Gelenkbewegung mit linkem Joystick X (mit Deadband)
        int selectedJoint = StepperSystem::getSelectedJoint();
        float rawX = leftJoystick->getXWithDeadband(_joystickConfig.deadband);
        
        // Normalisieren auf [-1..+1]
        float norm = (rawX - 500.0f) / 500.0f;
        
        Debug::print(F("Joint mode: rawX w/ deadzone = "));
        Debug::print(rawX);
        Debug::print(F(" → norm = "));
        Debug::println(norm);
        
        // Geschwindigkeit setzen
        if (rawX == 0.0f) {
            // Im Deadzone-Bereich = Stillstand
            StepperSystem::steppers[selectedJoint]->setSpeed(0);
        } else {
            // Volle Empfindlichkeit: ±maxSpeed bei |norm|==1
            extern StepperConfig _stepperConfig[6];
            float spd = -norm * _stepperConfig[selectedJoint].maxSpeed;
            
            Debug::print(F(" -> Setting speed for Joint "));
            Debug::print(selectedJoint);
            Debug::print(F(": "));
            Debug::println(spd);
            
            StepperSystem::steppers[selectedJoint]->setSpeed(spd);
        }
        
        // Motor kontinuierlich laufen lassen
        StepperSystem::steppers[selectedJoint]->runSpeed();
    }
    
    void processKinematicModeJoysticks() {
        if (!leftJoystick || !rightJoystick) return;
        
        Debug::println(F("=== processKinematicModeJoysticks() start ==="));
        
        // Joystick-Rohwerte
        float leftX = leftJoystick->getNormalizedX();
        float leftY = leftJoystick->getNormalizedY();
        float rightX = rightJoystick->getNormalizedX();
        float rightY = rightJoystick->getNormalizedY();
        
        // Filter-Reset bei Stillstand
        static bool wasMoving = false;
        bool isMoving = (fabs(leftX) > 0.01f) || (fabs(leftY) > 0.01f) ||
                        (fabs(rightX) > 0.01f) || (fabs(rightY) > 0.01f);
        
        if (!isMoving && wasMoving) {
            // Joystick wurde losgelassen → Filter zurücksetzen
            positionFilterX.reset();
            positionFilterY.reset();
            positionFilterZ.reset();
            rotationFilter.reset();
            Debug::println(F("Joystick released, filters reset!"));
        }
        wasMoving = isMoving;
        
        Debug::print(F("  Joystick: Lx=")); Debug::print(leftX, 2);
        Debug::print(F("  Ly=")); Debug::print(leftY, 2);
        Debug::print(F("  Rx=")); Debug::print(rightX, 2);
        Debug::print(F("  Ry=")); Debug::println(rightY, 2);
        
        // Geschwindigkeit dynamisch setzen
        float speedFactor = max(max(fabs(leftX), fabs(leftY)),
                               max(fabs(rightX), fabs(rightY)));
        float dynSpd = speedFactor * 100.0f;
        float dynAcc = dynSpd * 2.0f;
        
        for (int i = 0; i < 6; ++i) {
            StepperSystem::steppers[i]->setMaxSpeed(dynSpd);
            StepperSystem::steppers[i]->setAcceleration(dynAcc);
        }
        
        Debug::print(F("  dynSpd=")); Debug::print(dynSpd, 1);
        Debug::print(F("  dynAcc=")); Debug::println(dynAcc, 1);
        
        // Delta-Berechnung via Filter
        CartesianPose current = RobotSystem::getKinematics()->getCurrentPose();
        float xChg = positionFilterX.update(leftX * 1.0f);
        float yChg = positionFilterY.update(-leftY * 1.0f);
        float zChg = positionFilterZ.update(-rightY * 1.0f);
        float yawChg = rotationFilter.update(rightX * 0.03f);
        
        Debug::print(F("  Deltas: xChg=")); Debug::print(xChg, 3);
        Debug::print(F("  yChg=")); Debug::print(yChg, 3);
        Debug::print(F("  zChg=")); Debug::print(zChg, 3);
        Debug::print(F("  yawChg=")); Debug::println(yawChg, 4);
        
        // Nur bewegen, wenn signifikante Änderung
        if (fabs(xChg) > 0.01f || fabs(yChg) > 0.01f ||
            fabs(zChg) > 0.01f || fabs(yawChg) > 0.001f) {
            
            // Zielpose berechnen
            CartesianPose target = current;
            target.x += xChg;
            target.y += yChg;
            target.z += zChg;
            target.yaw += yawChg;
            
            // Prüfen, ob innerhalb des Arbeitsraums
            if (RobotSystem::isWithinWorkspace(target)) {
                // IK aufrufen und Bewegung durchführen
                RobotSystem::moveToPose(target, false);
            } else {
                Debug::println(F("Target position outside workspace!"));
            }
        }
        
        Debug::println(F("=== processKinematicModeJoysticks() end ==="));
    }
    
    void processMenuJoysticks() {
        if (!leftJoystick || !rightJoystick) return;
        
        // Menüsteuerung mit Y-Achse des rechten Joysticks
        float rightY = rightJoystick->getNormalizedY();
        
        if (millis() - lastJoystickMove > 200) {
            if (rightY > 0.7f && joystickYState == JOYSTICK_CENTERED) {
                joystickYState = JOYSTICK_UP;
                // Menüauswahl nach oben bewegen
                // Die eigentliche Implementierung hängt vom aktiven Menü ab
                // und wird im mode_controller.cpp behandelt
                lastJoystickMove = millis();
            } 
            else if (rightY < -0.7f && joystickYState == JOYSTICK_CENTERED) {
                joystickYState = JOYSTICK_DOWN;
                // Menüauswahl nach unten bewegen
                lastJoystickMove = millis();
            }
            else if (fabs(rightY) < 0.4f) {
                joystickYState = JOYSTICK_CENTERED;
            }
        }
        
        // Der linke Button wird verwendet, um Menüpunkte auszuwählen
        // Dies wird in der update()-Methode des mode_controller.cpp gehandhabt
    }
}

