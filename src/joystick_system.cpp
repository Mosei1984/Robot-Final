#include "joystick_system.h"
#include "robot_system.h"
#include "Debug.h"

// Define joystick objects
static Joystick* leftJoystick = nullptr;
static Joystick* rightJoystick = nullptr;
static bool calibrating = false;

// Initialize the joystick system
void JoystickSystem::init() {
    Debug::println(F("Initializing joystick system..."));
    
    // Initialize joysticks with appropriate pins
    leftJoystick = new Joystick(A0, A1, 22); // Adjust pins as needed
    rightJoystick = new Joystick(A2, A3, 24);
    
    // Begin joystick initialization
    leftJoystick->begin();
    rightJoystick->begin();
    
    Debug::println(F("Joystick system initialized"));
}

// Update joystick readings
void JoystickSystem::update() {
    // Read joystick values
    if (leftJoystick) leftJoystick->read();
    if (rightJoystick) rightJoystick->read();
    
    // If in calibration mode, process calibration
    if (calibrating) {
        processCalibration();
    }
}

// Get the left joystick object
Joystick* JoystickSystem::getLeftJoystick() {
    return leftJoystick;
}

// Get the right joystick object
Joystick* JoystickSystem::getRightJoystick() {
    return rightJoystick;
}

// Start the calibration process
void JoystickSystem::startCalibration() {
    calibrating = true;
    if (leftJoystick) leftJoystick->startCalibration();
    if (rightJoystick) rightJoystick->startCalibration();
}

// Process ongoing calibration
void JoystickSystem::processCalibration() {
    // Call calibrate() on both joysticks
    if (leftJoystick) leftJoystick->calibrate();
    if (rightJoystick) rightJoystick->calibrate();
    
    // Use a timer approach for calibration
    static unsigned long calibrationStartTime = 0;
    
    // Initialize the timer if it's 0
    if (calibrationStartTime == 0) {
        calibrationStartTime = millis();
    }
    
    // Exit calibration mode after a fixed time (5 seconds)
    if (millis() - calibrationStartTime > 5000) {
        calibrating = false;
        calibrationStartTime = 0;  // Reset for next time
        Debug::println(F("Calibration completed"));
    }
}

// Check if calibration is in progress
bool JoystickSystem::isCalibrating() {
    return calibrating;
}

// Process joysticks in joint control mode
void JoystickSystem::processJointModeJoysticks() {
    // Get current joint
    int jointIndex = StepperSystem::getSelectedJoint();
    
    if (leftJoystick && rightJoystick) {
        // Left joystick X - Move selected joint
        float leftX = leftJoystick->getNormalizedX();
        if (abs(leftX) > 0.1f) {
            float speed = leftX * 20.0f; // Scale appropriately
            StepperSystem::steppers[jointIndex]->setSpeed(speed);
            StepperSystem::steppers[jointIndex]->runSpeed();
        }
        
        // Right joystick Y - Change selected joint
        static unsigned long lastJointChange = 0;
        float rightY = rightJoystick->getNormalizedY();
        
        if (abs(rightY) > 0.7f && millis() - lastJointChange > 300) {
            if (rightY > 0 && jointIndex > 0) {
                jointIndex--;
                lastJointChange = millis();
            } else if (rightY < 0 && jointIndex < 5) {
                jointIndex++;
                lastJointChange = millis();
            }
            StepperSystem::setSelectedJoint(jointIndex);
        }
    }
}

// Process joysticks in kinematic control mode
void JoystickSystem::processKinematicModeJoysticks() {
    // Implement kinematic control
    if (!leftJoystick || !rightJoystick || !RobotSystem::getKinematics()) {
        return;
    }
    
    // Get current pose
    RobotSystem::CartesianPose currentPose = RobotSystem::getKinematics()->getCurrentPose();
    
    // Apply joystick values to modify the pose
    float leftX = leftJoystick->getNormalizedX();
    float leftY = leftJoystick->getNormalizedY();
    float rightX = rightJoystick->getNormalizedX();
    float rightY = rightJoystick->getNormalizedY();
    
    // Apply scaled values
    if (abs(leftX) > 0.1f) currentPose.x += leftX * 2.0f;
    if (abs(leftY) > 0.1f) currentPose.y += leftY * 2.0f;
    if (abs(rightY) > 0.1f) currentPose.z += rightY * 2.0f;
    if (abs(rightX) > 0.1f) currentPose.yaw += rightX * 0.05f;
    
    // Move to the new pose without waiting
    RobotSystem::moveToPose(currentPose, false);
}

// Get normalized values from all joysticks
JoystickValues JoystickSystem::getJoystickValues() {
    JoystickValues values = {};
    
    if (leftJoystick && rightJoystick) {
        values.leftX = leftJoystick->getNormalizedX();
        values.leftY = leftJoystick->getNormalizedY();
        values.rightX = rightJoystick->getNormalizedX();
        values.rightY = rightJoystick->getNormalizedY();
        values.leftButton = leftJoystick->isPressed();
        values.rightButton = rightJoystick->isPressed();
    }
    
    return values;
}