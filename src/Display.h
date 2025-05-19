#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "RobotKinematics.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

namespace DisplaySystem {
    // Initialize the display
    void init();
    
    // Get the display instance
    Adafruit_SSD1306* getDisplay();
    
    // Display screens for different modes
    void displayStartupScreen();
    void displayJointMode(RobotKinematics* kinematics, int selectedJoint);
    void displayKinematicMode(RobotKinematics* kinematics);
    void displayPositionMode(RobotKinematics* kinematics);
    void displayHomingMenu(int selection);
    void displayHomingProgress(int jointIndex);
    void displayCenteringProgress(float progress);
    void displayCalibrationMode();
    void displayGCodeMode();
    void displayRecordMode();
    void displayPlayMode();
    void displayConfigMenu();
    void displaydrawBitmap();
    // Utility functions
    void showMessage(const char* line1, const char* line2 = nullptr, int duration = 2000);
    void drawProgressBar(int x, int y, int width, int height, float progress);
}
