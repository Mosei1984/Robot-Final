#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

#include <Arduino.h>
#include "config.h"
#include "stepper_control.h"
// Forward declarations for needed classes
class RobotKinematics;

namespace RobotSystem {
    // States for the robot system state machine
    enum SystemState {
        STATE_STARTUP,
        STATE_JOINT_MODE,
        STATE_KINEMATIC_MODE,
        STATE_HOMING_MODE,
        STATE_CALIBRATION_MODE,
        STATE_GEAR_MENU,
        STATE_CONFIG_MODE,
        STATE_ERROR
    };
    
    // Homing menu options
    enum HomingMenuOption {
        HOMING_MENU_START_HOMING = 0,
        HOMING_MENU_TO_CENTER,
        HOMING_MENU_SAVE_HOME,
        HOMING_MENU_LOAD_HOME,
        HOMING_MENU_CLEAR_HOME,
        HOMING_MENU_CONFIG,
        HOMING_MENU_COUNT
    };
    
    // DH Parameter structure for robot kinematics
    struct DHParam {
        float a;      // Link length
        float alpha;  // Link twist
        float d;      // Link offset
        float theta;  // Joint angle
    };
    
    // Joint angles structure
    struct JointAngles {
        float angles[6];
    };
    
    // Cartesian pose structure
    struct CartesianPose {
        float x, y, z;
        float roll, pitch, yaw;
    };
    
    // Robot configuration structure
    struct RobotConfig {
        float jointMin[6];
        float jointMax[6];
        DHParam dhParams[6];
        float toolOffsetX;
        float toolOffsetY;
        float toolOffsetZ;
    };
    
    // Class definition for robot kinematics
    class RobotKinematics {
    public:
        RobotKinematics(const RobotConfig& config);
        bool forwardKinematics(const JointAngles& jointAngles, CartesianPose& pose);
        bool inverseKinematics(const CartesianPose& pose, JointAngles& jointAngles);
        void setCurrentJointAngles(const JointAngles& angles);
        JointAngles getCurrentJointAngles() const;
        CartesianPose getCurrentPose() const;
        
    private:
        RobotConfig _config;
        JointAngles _currentJointAngles;
    };
    
    // Function declarations
    void init();
    void initRobotConfig();
    void update();
    void processCurrentState();
    void changeState(SystemState newState);
    SystemState getState();
    void setState(SystemState state);
    unsigned long getStateChangeTime();
    void setStateChangeTime(unsigned long time);
    RobotKinematics* getKinematics();
    bool isWithinWorkspace(const CartesianPose& pose);
    void moveToPose(const CartesianPose& pose, bool waitForCompletion);
    bool isCalibrationLocked();
    void setCalibrationLocked(bool locked);
    bool isHomingStarted();
    void setHomingStarted(bool started);
    int getHomingJointIndex();
    void setHomingJointIndex(int index);
    int getHomingMenuSelection();
    void setHomingMenuSelection(int selection);
    int getHomingMenuOptionCount();
    const char* getHomingMenuOptionName(int option);
    void processHomingMode();
    void processHomingMenu();
    void processHomingMenuSelection();
    void saveRobotHome(const JointAngles& angles);
    bool loadRobotHome(JointAngles& angles);
    void clearRobotHome();
    bool saveRobotHomeToSD(const JointAngles& angles, const char* filename = nullptr);
    bool loadRobotHomeFromSD(JointAngles& angles, const char* filename = nullptr);
    bool clearRobotHomeFromSD(const char* filename = nullptr);
    SystemState getCurrentState();
    
    // External variables
    extern RobotKinematics* robotKin;
    extern RobotConfig robotConfig;
    extern SystemState currentState;
    extern unsigned long stateChangeTime;
    extern int homingJointIndex;
    extern bool homingStarted;
    extern int homingMenuSelection;
    extern bool calibrationLocked;
}
#endif // ROBOT_SYSTEM_H
