#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H  

#include "KalmanFilter.h"
#include <EEPROM.h>
#include <Arduino.h>
#include "config.h"
#include "RobotKinematics.h"

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

// Homing-Menü-Optionen
enum HomingMenuOption {
    HOMING_MENU_START_HOMING = 0,
    HOMING_MENU_TO_CENTER,
    HOMING_MENU_SAVE_HOME,
    HOMING_MENU_LOAD_HOME,
    HOMING_MENU_CLEAR_HOME,
    HOMING_MENU_COUNT
};

namespace RobotSystem {
    // Grundlegende Initialisierung
    void init();
    void initRobotConfig();
    
    // Hauptaktualisierungsfunktion
    void update();
    
    // Zustandsverarbeitungsfunktionen
    void processJointControl();
    void processKinematicControl();
    void processHomingMode();
    void processHomingMenu();
    void processButtonInput();
    void processCurrentState();
    
    // Aktueller Systemzustand
    SystemState getCurrentState();
    void setState(SystemState newState);
    void setStateChangeTime(unsigned long time);
    
    // Home-Position-Management
    void saveRobotHome(const JointAngles& angles);
    bool loadRobotHome(JointAngles& angles);
    void clearRobotHome();
    
    // Kinematik-Dienstprogramme
    void synchronizeKinematicsWithSteppers();
    void exitKinematicMode();
    RobotKinematics* getKinematics();
    
    // Getter und Setter
    int getSelectedJoint();
    void setSelectedJoint(int joint);
    int getHomingMenuSelection();
    bool isHomingStarted();
    int getHomingJointIndex();
    void setCalibrationLocked(bool locked);
    
    // Bewegungssteuerung
    void moveToPose(const CartesianPose& targetPose, bool waitForCompletion = true);
    void moveToJointAngles(const JointAngles& targetAngles, bool waitForCompletion = true);
    
    // Sicherheitsfunktionen
    bool isWithinWorkspace(const CartesianPose& pose);
    void enableSteppers();
    void disableSteppers();
    void emergencyStop();
    void resetKalmanFilters();
    
    // Home-Joint-Funktion für den Homing-Prozess
    bool homeJoint(int jointIndex);
    void incrementHomingJointIndex();
    void setHomingStarted(bool started);
}

#endif // ROBOT_SYSTEM_H
