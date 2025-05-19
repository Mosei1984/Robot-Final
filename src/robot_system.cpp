#include "robot_system.h"
#include "Debug.h"
#include "kalmanfilter.h"
#include <EEPROM.h>
#include "joystick_system.h"
#include "stepper_control.h"
#include "display_system.h"

// Globale Variablen aus main.cpp, die wir nutzen müssen
extern AccelStepper* _steppers[6];
extern PinConfig _pinConfig;
extern JoystickConfig _joystickConfig;
extern StepperConfig _stepperConfig[6];

namespace RobotSystem {
    // Private Zustandsvariablen
    static SystemState currentState = STATE_STARTUP;
    static bool homingStarted = false;
    static int homingJointIndex = 0;
    static int selectedJoint = 0;
    static int homingMenuSelection = 0;
    static bool calibrationLock = false;
    static unsigned long stateChangeTime = 0;
    static unsigned long lastButtonCheckTime = 0;
    static unsigned long lastMenuMove = 0;
    static unsigned long lastClearHomePress = 0;
    static int clearHomePressCount = 0;
    
    // RobotKinematics und RobotConfig
    static RobotConfig robotConfig;
    static RobotKinematics* robotKin = nullptr;
    
    // Joystick-Zustände für Richtungsänderungen
    enum JoystickStateInternal {
        JOYSTICK_CENTERED_INTERNAL,
        JOYSTICK_LEFT_INTERNAL,
        JOYSTICK_RIGHT_INTERNAL,
        JOYSTICK_UP_INTERNAL,
        JOYSTICK_DOWN_INTERNAL,
        
    };
    // Systemzustände
enum SystemState {
    STATE_STARTUP,
    STATE_JOINT_MODE,
    STATE_KINEMATIC_MODE,
    STATE_HOMING_MODE,
    STATE_CALIBRATION_MODE,
    NORMAL_OPERATION,
    ERROR_STATE,
    EMERGENCY_STOP
};
    static JoystickStateInternal joystickXState = JOYSTICK_CENTERED_INTERNAL;
    
    // Kalmanfilter für sanfte Bewegungen
    static KalmanFilter positionFilterX(0.01, 0.1);
    static KalmanFilter positionFilterY(0.01, 0.1);
    static KalmanFilter positionFilterZ(0.01, 0.1);
    static KalmanFilter rotationFilter(0.01, 0.1);
    
    // Datenstruktur für das Speichern der Home-Position
    struct RobotHomeData {
        uint32_t magic;
        float jointAngles[6];
    };
    
    void init() {
        Debug::println(F("Initializing robot system..."));
        
        // Roboter-Konfiguration initialisieren
        initRobotConfig();
        
        // Roboterkinematik initialisieren
        robotKin = new RobotKinematics(robotConfig);
        
        // Initialisiere Gelenkwinkel auf Null
        JointAngles init = {{0, 0, 0, 0, 0, 0}};
        robotKin->setCurrentJointAngles(init);
        
        // Anfangszustandsvariablen setzen
        stateChangeTime = millis();
        
        // Filter initialisieren
        resetKalmanFilters();
        
        Debug::println(F("Robot system initialized"));
    }
    
    void initRobotConfig() {
        // Gelenkwinkelgrenzen (in Grad)
        for (int i = 0; i < 6; i++) {
            robotConfig.jointMin[i] = -180.0 * DEG_TO_RAD;
            robotConfig.jointMax[i] = 180.0 * DEG_TO_RAD;
        }
        
        // DH-Parameter - einfacher 6DOF-Arm
        // Format: {a, alpha, d, theta}
        robotConfig.dhParams[0] = {0.0, M_PI/2, 50.0, 0.0};     // Basis
        robotConfig.dhParams[1] = {80.0, 0.0, 0.0, M_PI/2};     // Schulter
        robotConfig.dhParams[2] = {80.0, 0.0, 0.0, 0.0};        // Ellbogen
        robotConfig.dhParams[3] = {0.0, M_PI/2, 80.0, 0.0};     // Handgelenk Pitch
        robotConfig.dhParams[4] = {0.0, -M_PI/2, 0.0, 0.0};     // Handgelenk Roll
        robotConfig.dhParams[5] = {0.0, 0.0, 40.0, 0.0};        // Greifer
        
        // Werkzeug-Offset
        robotConfig.toolOffsetX = 0.0;
        robotConfig.toolOffsetY = 0.0;
        robotConfig.toolOffsetZ = 30.0;
        
        Debug::println(F("Robot configuration initialized"));
    }
    
    void update() {
        // Diese Funktion wird vom Hauptloop aufgerufen
        // Die Verarbeitung des aktuellen Zustands erfolgt in processCurrentState()
    }
    
    void processCurrentState() {
        // Prozessiert den aktuellen Roboterzustand
        switch (currentState) {
            case STATE_JOINT_MODE:
                processJointControl();
                break;
                
            case STATE_KINEMATIC_MODE:
                processKinematicControl();
                break;
                
            case STATE_HOMING_MODE:
                processHomingMode();
                break;
                
            case STATE_CALIBRATION_MODE:
                // Kalibrierung hier verarbeiten
                break;
                
            default:
                break;
        }
    }
    
    void processButtonInput() {
        Joystick* leftJoystick = JoystickSystem::getLeftJoystick();
        Joystick* rightJoystick = JoystickSystem::getRightJoystick();
        
        if (!leftJoystick || !rightJoystick) return;
        
        bool leftPressed = leftJoystick->isPressed();
        bool rightPressed = rightJoystick->isPressed();
        
        // Entprellen
        if (millis() - lastButtonCheckTime < 200) return;
        
        // Rechter Button: Moduswechsel
        static bool rightButtonPressed = false;
        if (rightPressed && !rightButtonPressed) {
            if (millis() - stateChangeTime > 500) {
                rightButtonPressed = true;
                lastButtonCheckTime = millis();
                
                // Modi durchschalten
                if (currentState == STATE_JOINT_MODE) {
                    setState(STATE_KINEMATIC_MODE);
                } else if (currentState == STATE_KINEMATIC_MODE) {
                    exitKinematicMode();
                    setState(STATE_HOMING_MODE);
                    homingStarted = false;
                    homingJointIndex = 0;
                } else if (currentState == STATE_HOMING_MODE) {
                    setState(STATE_CALIBRATION_MODE);
                } else if (currentState == STATE_CALIBRATION_MODE) {
                    setState(STATE_JOINT_MODE);
                }
                
                Debug::print(F("State changed to: "));
                Debug::println(currentState);
            }
        } 
        else if (!rightPressed) {
                        rightButtonPressed = false;
        }
        
        // Linker Button: Aktion im Modus
        if (leftPressed && !calibrationLock) {
            if (millis() - stateChangeTime > 500) {
                if (currentState == STATE_HOMING_MODE && !homingStarted) {
                    homingStarted = true;
                    Debug::println(F("Homing started"));
                } else if (currentState == STATE_CALIBRATION_MODE) {
                    JoystickSystem::startFullCalibration();
                }
                
                stateChangeTime = millis();
                lastButtonCheckTime = millis();
            }
        } 
        else if (!leftPressed) {
            calibrationLock = false;
        }
    }
    
    void processJointControl() {
        Joystick* leftJoystick = JoystickSystem::getLeftJoystick();
        Joystick* rightJoystick = JoystickSystem::getRightJoystick();
        
        if (!leftJoystick || !rightJoystick) return;
        
        // Debug-Ausgabe
        Debug::print(F("Joystick values: Left X: "));
        Debug::print(leftJoystick->getX());
        Debug::print(F(", Left Y: "));
        Debug::println(leftJoystick->getY());
        
        // Gelenkauswahl mit rechtem Joystick X
        float rightX = (rightJoystick->getX() - 500.0f) / 500.0f;  // auf [-1..+1]
        
        if (rightX > 0.7f && joystickXState == JOYSTICK_CENTERED_INTERNAL) {
            joystickXState = JOYSTICK_RIGHT_INTERNAL;
            if (selectedJoint < 5) {
                selectedJoint++;
                Debug::print(F("Joint increased to: "));
                Debug::println(selectedJoint + 1);
            }
        }
        else if (rightX < -0.7f && joystickXState == JOYSTICK_CENTERED_INTERNAL) {
            joystickXState = JOYSTICK_LEFT_INTERNAL;
            if (selectedJoint > 0) {
                selectedJoint--;
                Debug::print(F("Joint decreased to: "));
                Debug::println(selectedJoint + 1);
            }
        }
        else if (fabs(rightX) < 0.4f) {
            joystickXState = JOYSTICK_CENTERED_INTERNAL;
        }
        
        // Gelenkbewegung mit linkem Joystick X (mit Deadband)
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
            _steppers[selectedJoint]->setSpeed(0);
        } else {
            // Volle Empfindlichkeit: ±maxSpeed bei |norm|==1
            float spd = -norm * _stepperConfig[selectedJoint].maxSpeed;
            
            Debug::print(F(" -> Setting speed for Joint "));
            Debug::print(selectedJoint);
            Debug::print(F(": "));
            Debug::println(spd);
            
            _steppers[selectedJoint]->setSpeed(spd);
        }
        
        // Motor kontinuierlich laufen lassen
        _steppers[selectedJoint]->runSpeed();
    }
    
    void processKinematicControl() {
        Joystick* leftJoystick = JoystickSystem::getLeftJoystick();
        Joystick* rightJoystick = JoystickSystem::getRightJoystick();
        
        if (!leftJoystick || !rightJoystick) return;
        
        Debug::println(F("=== processKinematicControl() start ==="));
        
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
            _steppers[i]->setMaxSpeed(dynSpd);
            _steppers[i]->setAcceleration(dynAcc);
        }
        
        Debug::print(F("  dynSpd=")); Debug::print(dynSpd, 1);
        Debug::print(F("  dynAcc=")); Debug::println(dynAcc, 1);
        
        // Delta-Berechnung via Filter
        CartesianPose current = robotKin->getCurrentPose();
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
            
            // Zielpose berechnen und begrenzen
            CartesianPose target = current;
            target.x += xChg;
            target.y += yChg;
            target.z += zChg;
            target.yaw += yawChg;
            
            // Konstanten für Begrenzungen aus Konfiguration
            const float a1 = robotConfig.dhParams[1].a;
            const float a2 = robotConfig.dhParams[2].a;
            const float maxRad = a1 + a2;
            const float minZ = robotConfig.dhParams[0].d;
            const float maxZ = minZ + maxRad;
            
            // Begrenzung anwenden
            target.x = constrain(target.x, -maxRad, +maxRad);
            target.y = constrain(target.y, -maxRad, +maxRad);
            target.z = constrain(target.z, minZ, maxZ);
            
            Debug::print(F("  Kinematic target: X=")); Debug::print(target.x, 2);
            Debug::print(F("  Y=")); Debug::print(target.y, 2);
            Debug::print(F("  Z=")); Debug::println(target.z, 2);
            Debug::print(F("               Yaw=")); Debug::println(target.yaw, 3);
            
            // IK aufrufen
            JointAngles newA;
            if (robotKin->inverseKinematics(target, newA)) {
                for (int i = 0; i < 6; ++i) {
                    float deg = newA.angles[i] * 180.0 / M_PI;
                    long steps = deg * _stepperConfig[i].stepsPerDegree;
                    _steppers[i]->moveTo(steps);
                }
                robotKin->setCurrentJointAngles(newA);
            } else {
                Debug::println(F("  IK failed, skipping movement"));
                
                // Stepper-Targets auf aktuelle Position setzen
                for (int i = 0; i < 6; ++i) {
                    _steppers[i]->moveTo(_steppers[i]->currentPosition());
                }
            }
        }
        
        Debug::println(F("=== processKinematicControl() end ==="));
    }
    
    void processHomingMode() {
        // Menü anzeigen, wenn kein Homing läuft
        if (!homingStarted) {
            processHomingMenu();
            return;
        }
        
        // Wenn Homing läuft, Fortschritt anzeigen und homeJoint() benutzen
        DisplaySystem::displayHomingMode();
        
        if (homingJointIndex < 6) {
            if (homeJoint(homingJointIndex)) {
                homingJointIndex++;
                Debug::print(F("Joint "));
                Debug::print(homingJointIndex);
                Debug::println(F(" homed"));
                delay(500);
            }
        } else {
            // Nach Homing zurück ins Menü
            Debug::println(F("Homing completed"));
            homingStarted = false;
            homingJointIndex = 0;
            homingMenuSelection = 0;
            
            // Kinematik synchronisieren
            JointAngles homeAngles;
            for (int i = 0; i < 6; i++) {
                float posDegrees = _steppers[i]->currentPosition() / _stepperConfig[i].stepsPerDegree;
                homeAngles.angles[i] = posDegrees * M_PI / 180.0;
            }
            robotKin->setCurrentJointAngles(homeAngles);
        }
    }
    
    void processHomingMenu() {
        Joystick* leftJoystick = JoystickSystem::getLeftJoystick();
        Joystick* rightJoystick = JoystickSystem::getRightJoystick();
        
        if (!leftJoystick || !rightJoystick) return;
        
        // Moduswechsel per rechtem Button (entprellt)
        static bool rightButtonPressed = false;
        if (rightJoystick->isPressed() && !rightButtonPressed) {
            if (millis() - stateChangeTime > 500) {
                rightButtonPressed = true;
                stateChangeTime = millis();
                
                // Modi durchschalten: Homing -> Calibration -> Joint -> Kinematic -> Homing ...
                if (currentState == STATE_HOMING_MODE) {
                    setState(STATE_CALIBRATION_MODE);
                } else if (currentState == STATE_CALIBRATION_MODE) {
                    setState(STATE_JOINT_MODE);
                } else if (currentState == STATE_JOINT_MODE) {
                    setState(STATE_KINEMATIC_MODE);
                } else if (currentState == STATE_KINEMATIC_MODE) {
                    exitKinematicMode();
                    setState(STATE_HOMING_MODE);
                    homingStarted = false;
                    homingJointIndex = 0;
                }
                
                Debug::print(F("Mode changed to: "));
                Debug::println(currentState);
                return;
            }
        } else if (!rightJoystick->isPressed()) {
            rightButtonPressed = false;
        }
        
        DisplaySystem::displayHomingMenu();
        
        // Menüauswahl mit rechtem Joystick Y
        float rightY = rightJoystick->getNormalizedY();
        if (millis() - lastMenuMove > 200) {
            if (rightY > 0.7f && homingMenuSelection > 0) {
                homingMenuSelection--;
                lastMenuMove = millis();
            } else if (rightY < -0.7f && homingMenuSelection < HOMING_MENU_COUNT-1) {
                homingMenuSelection++;
                lastMenuMove = millis();
            }
        }
        
        // Linker Button: Bestätigen (nur auf steigende Flanke)
        static bool lastLeftPressed = false;
        bool leftPressed = leftJoystick->isPressed();
        
        if (leftPressed && !lastLeftPressed) {
            switch (homingMenuSelection) {
                case HOMING_MENU_START_HOMING:
                    homingStarted = true;
                    homingJointIndex = 0;
                    break;
                    
                case HOMING_MENU_TO_CENTER: {
                    Debug::println(F("Menu: Move to center selected"));
                    
                    // Zielpunkt im Arbeitsraum definieren
                    float a1 = robotConfig.dhParams[1].a;
                    float a2 = robotConfig.dhParams[2].a;
                    float minZ = robotConfig.dhParams[0].d;
                    
                    CartesianPose centerPose;
                    centerPose.x = 0;
                    centerPose.y = 0;
                    centerPose.z = minZ + (a1 + a2) / 2.0f;
                    centerPose.yaw = 0;
                    centerPose.pitch = 0;
                    centerPose.roll = 0;
                    
                    moveToPose(centerPose);
                    break;
                }
                
                case HOMING_MENU_SAVE_HOME: {
                    JointAngles current = robotKin->getCurrentJointAngles();
                    saveRobotHome(current);
                                        DisplaySystem::showMessage("Home saved!", nullptr, 1200);
                    delay(1200);
                    break;
                }
                
                case HOMING_MENU_LOAD_HOME: {
                    JointAngles homeAngles;
                    if (loadRobotHome(homeAngles)) {
                        // KEINE Bewegung! Nur Kinematik und Stepper-Positionen setzen
                        robotKin->setCurrentJointAngles(homeAngles);
                        for (int i = 0; i < 6; ++i) {
                            _steppers[i]->setCurrentPosition(
                                homeAngles.angles[i] * 180.0 / M_PI * _stepperConfig[i].stepsPerDegree
                            );
                        }
                        Debug::println(F("Home loaded! Kinematics synchronized, no movement"));
                        DisplaySystem::showMessage("Home loaded!", "No movement", 1200);
                        delay(1200);
                    } else {
                        DisplaySystem::showMessage("No home", "saved!", 1200);
                        delay(1200);
                    }
                    break;
                }
                
                case HOMING_MENU_CLEAR_HOME: {
                    if (clearHomePressCount == 0 || millis() - lastClearHomePress > 1500) {
                        clearHomePressCount = 1;
                        lastClearHomePress = millis();
                        DisplaySystem::showMessage("Press again", "to clear home!");
                    } else if (clearHomePressCount == 1) {
                        clearRobotHome();
                        DisplaySystem::showMessage("Home cleared!", nullptr, 1200);
                        delay(1200);
                        clearHomePressCount = 0;
                    }
                    break;
                }
            }
        }
        lastLeftPressed = leftPressed;
    }
    
    bool homeJoint(int jointIndex) {
        static bool homingJointStarted = false;
        static bool coarseHomingDone = false;
        static int lastSwitchState = HIGH;
        
        // Erste Initialisierung für dieses Gelenk
        if (!homingJointStarted) {
            homingJointStarted = true;
            coarseHomingDone = false;
            
            Debug::print(F("Starting homing for joint "));
            Debug::println(jointIndex + 1);
            
            // Motor aktivieren
            digitalWrite(_pinConfig.stepperPins[jointIndex][2], LOW); // Enable aktivieren
            
            // Schnelle Homing-Geschwindigkeit für Grobsuche setzen
            _steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed);
            
            // Aktuelle Position merken
            _steppers[jointIndex]->setCurrentPosition(0);
            
            // Initialen Zustand des Endschalters lesen
            lastSwitchState = digitalRead(_pinConfig.stepperPins[jointIndex][3]);
            
            Debug::print(F("Limit switch pin: "));
            Debug::print(_pinConfig.stepperPins[jointIndex][3]);
            Debug::print(F(", initial state: "));
            Debug::println(lastSwitchState == HIGH ? "HIGH" : "LOW");
        }
        
        // Endschalter überprüfen
        int limitSwitchPin = _pinConfig.stepperPins[jointIndex][3];
        int currentSwitchState = digitalRead(limitSwitchPin);
        
        // Phase 1: Grobe Suche nach dem Endschalter
        if (!coarseHomingDone) {
            if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
                Debug::print(F("Limit switch for joint "));
                Debug::print(jointIndex + 1);
                Debug::println(F(" reached during coarse homing"));
                
                // Motor anhalten
                _steppers[jointIndex]->setSpeed(0);
                _steppers[jointIndex]->stop();
                
                delay(100);
                
                // Vom Endschalter zurückfahren (positive Richtung, weg vom Endschalter)
                _steppers[jointIndex]->move(100); // Etwa 5 Grad zurück bei 20 steps/degree
                _steppers[jointIndex]->setSpeed(_stepperConfig[jointIndex].homingSpeed * 0.5);
                
                while (_steppers[jointIndex]->distanceToGo() != 0) {
                    _steppers[jointIndex]->runSpeed();
                }
                
                delay(100);
                
                Debug::print(F("Retreat completed, starting precise homing for joint "));
                Debug::println(jointIndex + 1);
                
                // Langsames Homing einleiten (1/4 der normalen Geschwindigkeit)
                _steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed * 0.25);
                
                coarseHomingDone = true;
                lastSwitchState = HIGH; // Zurücksetzen für die Feinsuche
            }
            
            lastSwitchState = currentSwitchState;
            _steppers[jointIndex]->runSpeed();
        }
        // Phase 2: Präzise langsame Suche
        else {
            if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
                Debug::print(F("Limit switch for joint "));
                Debug::print(jointIndex + 1);
                Debug::print(F(" reached during precise homing (Pin "));
                Debug::print(limitSwitchPin);
                Debug::println(F(")"));
                
                _steppers[jointIndex]->setSpeed(0);
                _steppers[jointIndex]->stop();
                
                _steppers[jointIndex]->setCurrentPosition(0);
                
                digitalWrite(_pinConfig.stepperPins[jointIndex][2], HIGH);
                
                JointAngles currentAngles = robotKin->getCurrentJointAngles();
                currentAngles.angles[jointIndex] = 0.0f;
                robotKin->setCurrentJointAngles(currentAngles);
                
                homingJointStarted = false;
                coarseHomingDone = false;
                return true; // Homing für dieses Gelenk abgeschlossen
            }
            
            lastSwitchState = currentSwitchState;
            _steppers[jointIndex]->runSpeed();
        }
        
        return false; // Homing für dieses Gelenk läuft noch
    }
    
    void saveRobotHome(const JointAngles& angles) {
        RobotHomeData data;
        data.magic = ROBOT_HOME_MAGIC;
        for (int i = 0; i < 6; ++i) data.jointAngles[i] = angles.angles[i];
        EEPROM.put(100, data);
    }
    
    bool loadRobotHome(JointAngles& angles) {
        RobotHomeData data;
        EEPROM.get(100, data);
        if (data.magic != ROBOT_HOME_MAGIC) return false;
        for (int i = 0; i < 6; ++i) angles.angles[i] = data.jointAngles[i];
        return true;
    }
    
    void clearRobotHome() {
        RobotHomeData data = {0};
        EEPROM.put(100, data);
    }
    
    void synchronizeKinematicsWithSteppers() {
        JointAngles currentAngles = robotKin->getCurrentJointAngles();
        bool updated = false;
        
        for (int i = 0; i < 6; i++) {
            // Aktuelle Stepper-Position in Grad umrechnen
            long currentSteps = _steppers[i]->currentPosition();
            float currentDegrees = currentSteps / _stepperConfig[i].stepsPerDegree;
            float currentRadians = currentDegrees * M_PI / 180.0;
            
            // Wenn es einen Unterschied gibt, aktualisiere die Kinematik
            if (fabs(currentRadians - currentAngles.angles[i]) > 0.01) {
                currentAngles.angles[i] = currentRadians;
                updated = true;
                
                Debug::print(F("Synchronizing joint "));
                Debug::print(i + 1);
                Debug::print(F(": "));
                Debug::print(currentDegrees);
                Debug::print(F("° ("));
                Debug::print(currentSteps);
                Debug::println(F(" steps)"));
            }
        }
        
        if (updated) {
            robotKin->setCurrentJointAngles(currentAngles);
        }
    }
    
    void exitKinematicMode() {
        for (int i = 0; i < 6; ++i) {
            _steppers[i]->moveTo(_steppers[i]->currentPosition());
        }
        synchronizeKinematicsWithSteppers();
    }
    
    void moveToPose(const CartesianPose& targetPose, bool waitForCompletion) {
        Debug::println(F("Moving to target Cartesian pose..."));
        
        JointAngles targetAngles;
        if (!robotKin->inverseKinematics(targetPose, targetAngles)) {
            Debug::println(F("IK error! Target not reachable."));
            DisplaySystem::showMessage("IK error!");
            return;
        }
        
        // Zielpositionen berechnen
        long targetSteps[6];
        long startSteps[6];
        for (int i = 0; i < 6; ++i) {
            float deg = targetAngles.angles[i] * 180.0 / M_PI;
            targetSteps[i] = deg * _stepperConfig[i].stepsPerDegree;
            startSteps[i] = _steppers[i]->currentPosition();
            _steppers[i]->moveTo(targetSteps[i]);
        }
        
        if (waitForCompletion) {
            // Bewegung mit Fortschrittsbalken
            bool allDone = false;
            while (!allDone) {
                allDone = true;
                long maxDist = 0, maxDistToGo = 0;
                for (int i = 0; i < 6; ++i) {
                    long dist = abs(targetSteps[i] - startSteps[i]);
                    long distToGo = abs(_steppers[i]->distanceToGo());
                    if (dist > maxDist) maxDist = dist;
                    if (distToGo > maxDistToGo) maxDistToGo = distToGo;
                    if (_steppers[i]->distanceToGo() != 0) {
                        _steppers[i]->run();
                        allDone = false;
                    }
                }
                
                // Fortschritt berechnen (0.0 ... 1.0)
                float progress = 1.0f;
                if (maxDist > 0) {
                    progress = 1.0f - (float)maxDistToGo / (float)maxDist;
                    if (progress < 0) progress = 0;
                    if (progress > 1) progress = 1;
                }
                
                // Fortschrittsanzeige aktualisieren
                DisplaySystem::displayHomingCenterProgress(progress);
                
                delay(10);
            }
            
            for (int i = 0; i < 6; ++i) {
                _steppers[i]->setCurrentPosition(_steppers[i]->targetPosition());
            }
            robotKin->setCurrentJointAngles(targetAngles);
            
            // FK zur Kontrolle
            CartesianPose pose = robotKin->getCurrentPose();
            Debug::print(F("FK End pose: X=")); Debug::print(pose.x, 2);
            Debug::print(F(" Y=")); Debug::print(pose.y, 2);
            Debug::print(F(" Z=")); Debug::print(pose.z, 2);
            Debug::print(F(" | Yaw=")); Debug::print(pose.yaw*180.0/M_PI, 2);
            Debug::print(F(" Pitch=")); Debug::print(pose.pitch*180.0/M_PI, 2);
            Debug::print(F(" Roll=")); Debug::println(pose.roll*180.0/M_PI, 2);
        }
    }
    
    void moveToJointAngles(const JointAngles& targetAngles, bool waitForCompletion) {
        Debug::println(F("Moving to target joint angles..."));
        
        // Zielpositionen berechnen
        long targetSteps[6];
        long startSteps[6];
        for (int i = 0; i < 6; ++i) {
            float deg = targetAngles.angles[i] * 180.0 / M_PI;
            targetSteps[i] = deg * _stepperConfig[i].stepsPerDegree;
            startSteps[i] = _steppers[i]->currentPosition();
            _steppers[i]->moveTo(targetSteps[i]);
        }
        
        if (waitForCompletion) {
            // Ähnlich wie bei moveToPose, mit Fortschrittsbalken
            bool allDone = false;
            while (!allDone) {
                allDone = true;
                for (int i = 0; i < 6; ++i) {
                    if (_steppers[i]->distanceToGo() != 0) {
                        _steppers[i]->run();
                        allDone = false;
                    }
                }
                delay(10);
            }
            
            for (int i = 0; i < 6; ++i) {
                _steppers[i]->setCurrentPosition(_steppers[i]->targetPosition());
            }
            robotKin->setCurrentJointAngles(targetAngles);
        }
    }
    
    void resetKalmanFilters() {
        positionFilterX.reset();
        positionFilterY.reset();
        positionFilterZ.reset();
        rotationFilter.reset();
    }
    
    // Getter und Setter
    SystemState getCurrentState() {
        return currentState;
    }
    
    void setState(SystemState newState) {
        currentState = newState;
        stateChangeTime = millis();
    }
    
    void setStateChangeTime(unsigned long time) {
        stateChangeTime = time;
    }
    
    int getSelectedJoint() {
        return selectedJoint;
    }
    
    void setSelectedJoint(int joint) {
        selectedJoint = joint;
    }
    
    int getHomingMenuSelection() {
        return homingMenuSelection;
    }
    
    bool isHomingStarted() {
        return homingStarted;
    }
    
    int getHomingJointIndex() {
        return homingJointIndex;
    }
    
    void setCalibrationLocked(bool locked) {
        calibrationLock = locked;
    }
    
    void setHomingStarted(bool started) {
        homingStarted = started;
    }
        }
    
    void incrementHomingJointIndex() {
        homingJointIndex++;
    }
    
    RobotKinematics* getKinematics() {
        return robotKin;
    }
    
    bool isWithinWorkspace(const CartesianPose& pose) {
        // Konstanten für Begrenzungen aus Konfiguration
        const float a1 = robotConfig.dhParams[1].a;
        const float a2 = robotConfig.dhParams[2].a;
        const float maxRad = a1 + a2;
        const float minZ = robotConfig.dhParams[0].d;
        const float maxZ = minZ + maxRad;
        
        // Prüfe, ob die Position innerhalb der Grenzen liegt
        if (pose.x < -maxRad || pose.x > maxRad) return false;
        if (pose.y < -maxRad || pose.y > maxRad) return false;
        if (pose.z < minZ || pose.z > maxZ) return false;
        
        // Zusätzliche Prüfung der radialen Entfernung
        float radius = sqrt(pose.x*pose.x + pose.y*pose.y);
        if (radius > maxRad) return false;
        
        return true;
    }
    
    void enableSteppers() {
        for (int i = 0; i < 6; i++) {
            digitalWrite(_pinConfig.stepperPins[i][2], LOW);  // Enable ist aktiv LOW
        }
    }
    
    void disableSteppers() {
        for (int i = 0; i < 6; i++) {
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);  // Disable ist HIGH
        }
    }
    
    void emergencyStop() {
        // Alle Motoren sofort stoppen und deaktivieren
        for (int i = 0; i < 6; i++) {
            _steppers[i]->setSpeed(0);
            _steppers[i]->stop();
            digitalWrite(_pinConfig.stepperPins[i][2], HIGH);  // Disable
        }
        
        setState(EMERGENCY_STOP);
        DisplaySystem::showMessage("EMERGENCY STOP", "System halted!");
    }


