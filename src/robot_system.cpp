#include "robot_system.h"
#include "Debug.h"
#include "display_system.h"
#include "joystick_system.h"
#include "stepper_control.h"
#include <EEPROM.h>
#include <SD.h>

#define ROBOT_HOME_MAGIC 0x42ABCDEF

namespace RobotSystem {
    // Definition der externen Variablen
    RobotKinematics* robotKin = nullptr;
    RobotConfig robotConfig;
    SystemState currentState = STATE_STARTUP;
    unsigned long stateChangeTime = 0;
    int homingJointIndex = 0;
    bool homingStarted = false;
    int homingMenuSelection = 0;
    bool calibrationLocked = false;
    
    // Hilfsfunktionen für Homing
    static bool _homeJoint(int jointIndex);
    static void _moveToCenter();
    
    // Initialisierung des Roboter-Systems
    void init() {
        Debug::println(F("Initializing robot system..."));
        
        // Roboter-Konfiguration initialisieren
        initRobotConfig();
        
        // Kinematik-Objekt erstellen
        robotKin = new RobotKinematics(robotConfig);
        
        // Gelenk-Winkel auf 0 initialisieren
        JointAngles init = {{0, 0, 0, 0, 0, 0}};
        robotKin->setCurrentJointAngles(init);
        
        // System-Zustand setzen
        currentState = STATE_STARTUP;
        stateChangeTime = millis();
        
        Debug::println(F("Robot system initialized"));
    }
    
    // Konfiguration des Roboters initialisieren
    void initRobotConfig() {
        Debug::println(F("Initializing robot configuration"));
        
        // Gelenk-Winkelgrenzen (in Grad)
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
    }
    
    // Haupt-Update-Funktion
    void update() {
        // Zustandsmachine verarbeiten
        processCurrentState();
    }
    
    // Zustandsmachine verarbeiten
    void processCurrentState() {
        switch (currentState) {
            case STATE_STARTUP:
                if (millis() - stateChangeTime > 2000) {
                    changeState(STATE_JOINT_MODE);
                }
                break;
                
            case STATE_JOINT_MODE:
                JoystickSystem::processJointModeJoysticks();
                break;
                
            case STATE_KINEMATIC_MODE:
                JoystickSystem::processKinematicModeJoysticks();
                break;
                
            case STATE_HOMING_MODE:
                processHomingMode();
                break;
                
            case STATE_CALIBRATION_MODE:
                // Anzeige aktualisieren
                DisplaySystem::displayCalibrationMode();
                break;
                
            case STATE_GEAR_MENU:
                DisplaySystem::processGearMenu();
                break;
                
            case STATE_CONFIG_MODE:
                // Konfigurationsmodus-Logik hier
                break;
                
            case STATE_ERROR:
                // Fehlerbehandlung hier
                DisplaySystem::showError("System Error");
                break;
        }
    }
    
    // Getter für den aktuellen Systemzustand
    SystemState getState() {
        return currentState;
    }
    
    // Setter für den Systemzustand
    void setState(SystemState state) {
        currentState = state;
        stateChangeTime = millis();
    }
    
    // Getter für den Zeitpunkt des letzten Zustandswechsels
    unsigned long getStateChangeTime() {
        return stateChangeTime;
    }
    
    // Setter für den Zeitpunkt des letzten Zustandswechsels
    void setStateChangeTime(unsigned long time) {
        stateChangeTime = time;
    }
    
    // Zustandswechsel mit Logging
    void changeState(SystemState newState) {
        Debug::print(F("Changing system state from "));
        Debug::print((int)currentState);
        Debug::print(F(" to "));
        Debug::println((int)newState);
        
        setState(newState);
    }
    
    // Getter für das Kinematik-Objekt
    RobotKinematics* getKinematics() {
        return robotKin;
    }
    
    // Prüfen, ob eine Pose innerhalb des Arbeitsbereichs liegt
    bool isWithinWorkspace(const CartesianPose& pose) {
        // Einfache Reichweiten-Prüfung
        float a1 = robotConfig.dhParams[1].a;
        float a2 = robotConfig.dhParams[2].a;
        float maxReach = a1 + a2;
        float minZ = robotConfig.dhParams[0].d;
        float maxZ = minZ + maxReach;
        
        // Euklidische Distanz zur Basis
        float dist = sqrt(pose.x*pose.x + pose.y*pose.y + 
                         (pose.z-minZ)*(pose.z-minZ));
        
        return (dist <= maxReach && pose.z >= minZ && pose.z <= maxZ);
    }
    
    // Bewegen zu einer bestimmten Pose
    void moveToPose(const CartesianPose& pose, bool waitForCompletion) {
        if (!isWithinWorkspace(pose)) {
            Debug::println(F("Target pose outside workspace!"));
            return;
        }
        
        // Inverse Kinematik berechnen
        JointAngles targetAngles;
        if (robotKin->inverseKinematics(pose, targetAngles)) {
            // Zielpositionen berechnen
            long targetSteps[6];
            for (int i = 0; i < 6; ++i) {
                float deg = targetAngles.angles[i] * 180.0 / M_PI;
                targetSteps[i] = deg * StepperSystem::enhancedSteppers[i].stepsPerDegree;
                StepperSystem::steppers[i]->moveTo(targetSteps[i]);
            }
            
            // Auf Abschluss warten, falls gewünscht
            if (waitForCompletion) {
                bool allDone = false;
                while (!allDone) {
                    allDone = true;
                    for (int i = 0; i < 6; ++i) {
                        if (StepperSystem::steppers[i]->distanceToGo() != 0) {
                            StepperSystem::steppers[i]->run();
                            allDone = false;
                        }
                    }
                }
                
                // Kinematik mit Stepper-Positionen synchronisieren
                for (int i = 0; i < 6; ++i) {
                    float posDegrees = StepperSystem::steppers[i]->currentPosition() / 
                 StepperSystem::enhancedSteppers[i].stepsPerDegree;
                    targetAngles.angles[i] = posDegrees * M_PI / 180.0;
                }
                
                robotKin->setCurrentJointAngles(targetAngles);
            }
        } else {
            Debug::println(F("Inverse kinematics failed!"));
        }
    }
    
    // Getter für den Kalibrierungs-Lock-Status
    bool isCalibrationLocked() {
        return calibrationLocked;
    }
    
    // Setter für den Kalibrierungs-Lock-Status
    void setCalibrationLocked(bool locked) {
        calibrationLocked = locked;
    }
    
    // Getter für den Homing-Status
    bool isHomingStarted() {
        return homingStarted;
    }
    
    // Setter für den Homing-Status
    void setHomingStarted(bool started) {
        homingStarted = started;
    }
    
    // Getter für den aktuellen Homing-Gelenk-Index
    int getHomingJointIndex() {
        return homingJointIndex;
    }
    
    // Setter für den Homing-Gelenk-Index
    void setHomingJointIndex(int index) {
        homingJointIndex = index;
    }
    
        // Getter für die aktuelle Homing-Menü-Auswahl
    int getHomingMenuSelection() {
        return homingMenuSelection;
    }
    
    // Setter für die Homing-Menü-Auswahl
    void setHomingMenuSelection(int selection) {
        homingMenuSelection = selection;
    }
    
    // Getter für die Anzahl der Homing-Menü-Optionen
    int getHomingMenuOptionCount() {
        return HOMING_MENU_COUNT;
    }
    
    // Getter für den Namen einer Homing-Menü-Option
    const char* getHomingMenuOptionName(int option) {
        switch (option) {
            case HOMING_MENU_START_HOMING: return "Start Homing";
            case HOMING_MENU_TO_CENTER:    return "Move to Center";
            case HOMING_MENU_SAVE_HOME:    return "Save Home";
            case HOMING_MENU_LOAD_HOME:    return "Load Home";
            case HOMING_MENU_CLEAR_HOME:   return "Clear Home";
            case HOMING_MENU_CONFIG:       return "Config";
            default:                       return "Unknown";
        }
    }
    
    // Homing-Modus verarbeiten
    void processHomingMode() {
        // Menü immer anzeigen, wenn kein Homing läuft
        if (!homingStarted) {
            processHomingMenu();
            return;
        }
        
        // Wenn Homing läuft, Fortschritt anzeigen und homeJoint() benutzen
        DisplaySystem::displayHomingProgress(homingJointIndex);
        
        if (homingJointIndex < 6) {
            if (_homeJoint(homingJointIndex)) {
                homingJointIndex++;
                Debug::print(F("Joint "));
                Debug::print(homingJointIndex);
                Debug::println(F(" referenced"));
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
                float posDegrees = StepperSystem::steppers[i]->currentPosition() / 
                 StepperSystem::enhancedSteppers[i].stepsPerDegree;
                homeAngles.angles[i] = posDegrees * M_PI / 180.0;
            }
            robotKin->setCurrentJointAngles(homeAngles);
        }
    }
    
    // Homing-Menü verarbeiten
    void processHomingMenu() {
        // Menü-Auswahl anzeigen
        DisplaySystem::displayHomingMenu(homingMenuSelection, HOMING_MENU_COUNT);
        
        // Menüauswahl mit rechtem Joystick Y
        float rightY = JoystickSystem::getRightJoystick()->getNormalizedY();
        
        // Menü-Navigation (mit Entprellung)
        static unsigned long lastMenuMove = 0;
        if (millis() - lastMenuMove > 200) {
            if (rightY > 0.7f && homingMenuSelection > 0) {
                homingMenuSelection--;
                lastMenuMove = millis();
            } else if (rightY < -0.7f && homingMenuSelection < HOMING_MENU_COUNT-1) {
                homingMenuSelection++;
                lastMenuMove = millis();
            }
        }
        
        // Linker Button: Menüoption auswählen
        static bool lastLeftPressed = false;
        bool leftPressed = JoystickSystem::getLeftJoystick()->isPressed();
        
        if (leftPressed && !lastLeftPressed) {
            processHomingMenuSelection();
        }
        
        lastLeftPressed = leftPressed;
        
        // Rechter Button: Modus wechseln
        static bool lastRightPressed = false;
        bool rightPressed = JoystickSystem::getRightJoystick()->isPressed();
        
        if (rightPressed && !lastRightPressed) {
            if (millis() - stateChangeTime > 500) {
                // Modi durchschalten: Homing -> Calibration -> Joint -> Kinematic -> Homing
                if (currentState == STATE_HOMING_MODE) {
                    changeState(STATE_CALIBRATION_MODE);
                } else if (currentState == STATE_CALIBRATION_MODE) {
                    changeState(STATE_JOINT_MODE);
                } else if (currentState == STATE_JOINT_MODE) {
                    changeState(STATE_KINEMATIC_MODE);
                } else if (currentState == STATE_KINEMATIC_MODE) {
                    changeState(STATE_HOMING_MODE);
                    homingStarted = false;
                    homingJointIndex = 0;
                }
            }
        }
        
        lastRightPressed = rightPressed;
    }
    
    // Verarbeitet eine ausgewählte Option im Homing-Menü
    void processHomingMenuSelection() {
        switch (homingMenuSelection) {
            case HOMING_MENU_START_HOMING:
                homingStarted = true;
                homingJointIndex = 0;
                break;
                
            case HOMING_MENU_TO_CENTER:
                _moveToCenter();
                break;
                
            case HOMING_MENU_SAVE_HOME: {
                JointAngles current = robotKin->getCurrentJointAngles();
                saveRobotHome(current);
                DisplaySystem::showMessage("Home saved!", nullptr, 1200);
                break;
            }
                
            case HOMING_MENU_LOAD_HOME: {
    JointAngles homeAngles;
    if (loadRobotHome(homeAngles)) {
        // KEINE Bewegung! Nur Kinematik und Stepper-Positionen setzen
        robotKin->setCurrentJointAngles(homeAngles);
        for (int i = 0; i < 6; ++i) {
            StepperSystem::steppers[i]->setCurrentPosition(
                homeAngles.angles[i] * 180.0 / M_PI * StepperSystem::enhancedSteppers[i].stepsPerDegree
            );
        } // Missing closing brace for the for loop
        Debug::println(F("Home loaded! Kinematics synchronized, no movement."));
        DisplaySystem::showMessage("Home loaded!", "No movement", 1200);
    } else {
        DisplaySystem::showMessage("No home", "saved!", 1200);
    }
    break;
}
            case HOMING_MENU_CLEAR_HOME: {
                static unsigned long lastClearHomePress = 0;
                static int clearHomePressCount = 0;
                
                if (clearHomePressCount == 0 || millis() - lastClearHomePress > 1500) {
                    clearHomePressCount = 1;
                    lastClearHomePress = millis();
                    DisplaySystem::showMessage("Press again", "to clear home!");
                } else if (clearHomePressCount == 1) {
                    clearRobotHome();
                    DisplaySystem::showMessage("Home cleared!", nullptr, 1200);
                    clearHomePressCount = 0;
                }
                break;
            }
                
            case HOMING_MENU_CONFIG:
                changeState(STATE_CONFIG_MODE);
                break;
        }
    }
    
    // Home-Position-Management
    
    // Home-Position im EEPROM speichern
    void saveRobotHome(const JointAngles& angles) {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data;
        data.magic = ROBOT_HOME_MAGIC;
        
        for (int i = 0; i < 6; ++i) {
            data.jointAngles[i] = angles.angles[i];
        }
        
        EEPROM.put(100, data);
        Debug::println(F("Home position saved to EEPROM"));
    }
    
    // Home-Position aus EEPROM laden
    bool loadRobotHome(JointAngles& angles) {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data;
        EEPROM.get(100, data);
        
        if (data.magic != ROBOT_HOME_MAGIC) {
            Debug::println(F("No valid home data found in EEPROM"));
            return false;
        }
        
        for (int i = 0; i < 6; ++i) {
            angles.angles[i] = data.jointAngles[i];
        }
        
        Debug::println(F("Home position loaded from EEPROM"));
        return true;
    }
    
    // Home-Position im EEPROM löschen
    void clearRobotHome() {
        struct RobotHomeData {
            uint32_t magic;
            float jointAngles[6];
        };
        
        RobotHomeData data = {0};
        EEPROM.put(100, data);
        Debug::println(F("Home position cleared from EEPROM"));
    }
    
    // Home-Position auf SD-Karte speichern
    bool saveRobotHomeToSD(const JointAngles& angles, const char* filename) {
        // Standard-Dateiname, wenn keiner angegeben
        const char* fname = filename ? filename : "HOME.DAT";
        
        // SD-Karte überprüfen
        if (!SD.begin()) {
            Debug::println(F("SD card initialization failed!"));
            return false;
        }
        
        // Datei öffnen/erstellen
        File file = SD.open(fname, FILE_WRITE);
        if (!file) {
            Debug::println(F("Failed to open file for writing"));
            return false;
        }
        
        // Magic-Nummer schreiben
        uint32_t magic = ROBOT_HOME_MAGIC;
        file.write((uint8_t*)&magic, sizeof(magic));
        
        // Gelenk-Winkel schreiben
        for (int i = 0; i < 6; ++i) {
            float angle = angles.angles[i];
            file.write((uint8_t*)&angle, sizeof(float));
        }
        
        file.close();
        Debug::print(F("Home position saved to SD card as "));
        Debug::println(fname);
        return true;
    }
    
    // Home-Position von SD-Karte laden
    bool loadRobotHomeFromSD(JointAngles& angles, const char* filename) {
        // Standard-Dateiname, wenn keiner angegeben
        const char* fname = filename ? filename : "HOME.DAT";
        
        // SD-Karte überprüfen
        if (!SD.begin()) {
            Debug::println(F("SD card initialization failed!"));
            return false;
        }
        
        // Datei öffnen
        File file = SD.open(fname, FILE_READ);
        if (!file) {
            Debug::println(F("Failed to open file for reading"));
            return false;
        }
        
        // Magic-Nummer lesen und überprüfen
        uint32_t magic;
        file.read((uint8_t*)&magic, sizeof(magic));
        
        if (magic != ROBOT_HOME_MAGIC) {
            Debug::println(F("Invalid home file format"));
            file.close();
            return false;
        }
        
        // Gelenk-Winkel lesen
        for (int i = 0; i < 6; ++i) {
            float angle;
            file.read((uint8_t*)&angle, sizeof(float));
            angles.angles[i] = angle;
        }
        
        file.close();
        Debug::print(F("Home position loaded from SD card: "));
        Debug::println(fname);
        return true;
    }
    
    // Home-Position von SD-Karte löschen
    bool clearRobotHomeFromSD(const char* filename) {
        // Standard-Dateiname, wenn keiner angegeben
        const char* fname = filename ? filename : "HOME.DAT";
        
        // SD-Karte überprüfen
        if (!SD.begin()) {
            Debug::println(F("SD card initialization failed!"));
            return false;
        }
        
        // Datei löschen
        if (SD.exists(fname)) {
            if (SD.remove(fname)) {
                Debug::print(F("Home file deleted from SD card: "));
                Debug::println(fname);
                return true;
            } else {
                Debug::println(F("Failed to delete home file"));
                return false;
            }
        } else {
            Debug::println(F("Home file not found on SD card"));
            return false;
        }
    }
    
    // --- Private Hilfsfunktionen ---
    
    // Homing für ein einzelnes Gelenk durchführen
    static bool _homeJoint(int jointIndex) {
        static bool homingJointStarted = false;
        static bool coarseHomingDone = false;
        static int lastSwitchState = HIGH;
        
        extern StepperConfig _stepperConfig[6];
        extern PinConfig _pinConfig;
        
        // Erste Initialisierung für dieses Gelenk
        if (!homingJointStarted) {
            homingJointStarted = true;
            coarseHomingDone = false;
            
            Debug::print(F("Starting homing for joint "));
            Debug::println(jointIndex + 1);
            
            // Motor aktivieren
            digitalWrite(_pinConfig.stepperPins[jointIndex][2], LOW); // Enable aktivieren
            
            // Schnelle Homing-Geschwindigkeit für die Grobsuche
            StepperSystem::steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed);
            
            // Aktuelle Position merken
            StepperSystem::steppers[jointIndex]->setCurrentPosition(0);
            
            // Initialen Endschalter-Zustand lesen
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
                StepperSystem::steppers[jointIndex]->setSpeed(0);
                StepperSystem::steppers[jointIndex]->stop();
                
                delay(100);
                
                // Vom Endschalter zurückfahren (positive Richtung, weg vom Endschalter)
                StepperSystem::steppers[jointIndex]->move(100); // Etwa 5 Grad zurück bei 20 steps/degree
                StepperSystem::steppers[jointIndex]->setSpeed(_stepperConfig[jointIndex].homingSpeed * 0.5);
                
                while (StepperSystem::steppers[jointIndex]->distanceToGo() != 0) {
                    StepperSystem::steppers[jointIndex]->runSpeed();
                }
                
                delay(100);
                
                Debug::print(F("Retreat completed, starting precise homing for joint "));
                Debug::println(jointIndex + 1);
                
                // Langsames Homing einleiten (1/4 der normalen Geschwindigkeit)
                StepperSystem::steppers[jointIndex]->setSpeed(-_stepperConfig[jointIndex].homingSpeed * 0.25);
                
                coarseHomingDone = true;
                lastSwitchState = HIGH; // Zurücksetzen für die Feinsuche
            }
            
            lastSwitchState = currentSwitchState;
            StepperSystem::steppers[jointIndex]->runSpeed();
        }
        // Phase 2: Präzise langsame Suche
        else {
            if (currentSwitchState != lastSwitchState && currentSwitchState == LOW) {
                Debug::print(F("Limit switch for joint "));
                Debug::print(jointIndex + 1);
                Debug::print(F(" reached during precise homing (Pin "));
                Debug::print(limitSwitchPin);
                Debug::println(F(")"));
                
                StepperSystem::steppers[jointIndex]->setSpeed(0);
                StepperSystem::steppers[jointIndex]->stop();
                
                StepperSystem::steppers[jointIndex]->setCurrentPosition(0);
                
                digitalWrite(_pinConfig.stepperPins[jointIndex][2], HIGH);
                
                JointAngles currentAngles = robotKin->getCurrentJointAngles();
                currentAngles.angles[jointIndex] = 0.0f;
                robotKin->setCurrentJointAngles(currentAngles);
                
                homingJointStarted = false;
                coarseHomingDone = false;
                return true; // Homing für dieses Gelenk abgeschlossen
            }
            
            lastSwitchState = currentSwitchState;
            StepperSystem::steppers[jointIndex]->runSpeed();
        }
        
        return false; // Homing für dieses Gelenk läuft noch
    }
    
    // Bewegt den Roboter zur zentralen Position
    static void _moveToCenter() {
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
        
        Debug::print(F("Target pose: X="));
        Debug::print(centerPose.x);
        Debug::print(F(" Y="));
        Debug::print(centerPose.y);
        Debug::print(F(" Z="));
        Debug::println(centerPose.z);
        
        // IK berechnen
        JointAngles centerAngles;
        if (robotKin->inverseKinematics(centerPose, centerAngles)) {
            // Zielpositionen berechnen
            long targetSteps[6];
            long startSteps[6];
            
            for (int i = 0; i < 6; ++i) {
                float deg = centerAngles.angles[i] * 180.0 / M_PI;
                targetSteps[i] = deg * StepperSystem::enhancedSteppers[i].stepsPerDegree;
                startSteps[i] = StepperSystem::steppers[i]->currentPosition();
                StepperSystem::steppers[i]->moveTo(targetSteps[i]);
            }
            
            // Bewegung mit Fortschrittsbalken
            bool allDone = false;
            while (!allDone) {
                allDone = true;
                long maxDist = 0, maxDistToGo = 0;
                
                for (int i = 0; i < 6; ++i) {
                    long dist = abs(targetSteps[i] - startSteps[i]);
                    long distToGo = abs(StepperSystem::steppers[i]->distanceToGo());
                    
                    if (dist > maxDist) maxDist = dist;
                    if (distToGo > maxDistToGo) maxDistToGo = distToGo;
                    
                    if (StepperSystem::steppers[i]->distanceToGo() != 0) {
                        StepperSystem::steppers[i]->run();
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
                
                // Fortschrittsbalken anzeigen
                DisplaySystem::displayHomingCenterProgress(progress);
                
                delay(10);
            }
            
            // Stepper-Positionen aktualisieren
            for (int i = 0; i < 6; ++i) {
                StepperSystem::steppers[i]->setCurrentPosition(StepperSystem::steppers[i]->targetPosition());
            }
            
            // Kinematik aktualisieren
            robotKin->setCurrentJointAngles(centerAngles);
            
            // FK zur Kontrolle
            CartesianPose pose = robotKin->getCurrentPose();
            Debug::print(F("FK End pose: X="));
            Debug::print(pose.x, 2);
            Debug::print(F(" Y="));
            Debug::print(pose.y, 2);
            Debug::print(F(" Z="));
            Debug::print(pose.z, 2);
            Debug::print(F(" | Yaw="));
            Debug::print(pose.yaw * 180.0 / M_PI, 2);
            Debug::print(F(" Pitch="));
            Debug::print(pose.pitch * 180.0 / M_PI, 2);
            Debug::print(F(" Roll="));
            Debug::println(pose.roll * 180.0 / M_PI, 2);
            
            DisplaySystem::showMessage("Robot is now", "in center!", 1500);
        } else {
            Debug::println(F("IK error! Target not reachable."));
            DisplaySystem::showMessage("IK error!", nullptr, 1500);
        }
    }
    
    // Definition des SD-Karten-CS-Pins (falls in Ihrer Konfiguration nicht definiert)
    #ifndef SD_CS_PIN
    #define SD_CS_PIN 53  // Standard für Arduino Mega
    #endif
    
} // namespace RobotSystem


