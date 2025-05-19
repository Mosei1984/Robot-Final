#include "Display.h"
#include <SPI.h>
#include <Wire.h>

namespace DisplaySystem {
    // Display instance
    static Adafruit_SSD1306* display = nullptr;
      
    void init() {
      
        // Initialize display
        if (display == nullptr) {
            display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
            
            if (!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
                Serial.println(F("SSD1306 allocation failed"));
                return;
            }
            
            display->clearDisplay();
            display->setTextSize(1);
            display->setTextColor(SSD1306_WHITE);
            display->display();
        }
        
        // Show startup screen
        displayStartupScreen();
    }
    
    Adafruit_SSD1306* getDisplay() {
        return display;
    }
    
    void displayStartupScreen() {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->setTextSize(2);
        display->println(F("6DOF Robot"));
        display->setTextSize(1);
        display->println();
        display->println(F("Controller v1.0"));
        display->println();
        display->println(F("Initializing..."));
        display->display();
        
        delay(2000);
        
        // Draw robot arm logo
        display->clearDisplay();
        // Add bitmap drawing code here if needed
        display->display();
        
        delay(1000);
    }
    
    void displayJointMode(RobotKinematics* kinematics, int selectedJoint) {
        if (!display || !kinematics) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("JOINT MODE"));
        
        // Display joint angles in degrees
        JointAngles angles = kinematics->getCurrentJointAngles();
        
        for (int i = 0; i < 6; i++) {
            display->print(F("J"));
            display->print(i+1);
            display->print(F(":"));
            display->print(angles.angles[i] * 180.0f / M_PI, 0);
            display->print(i == selectedJoint ? F("* ") : F("  "));
            
            if (i == 2) display->println();
        }
        display->println();
        
        // Display end effector position
        CartesianPose pose = kinematics->getCurrentPose();
        display->print(F("X:"));
        display->print(pose.x, 1);
        display->print(F(" Y:"));
        display->println(pose.y, 1);
        display->print(F("Z:"));
        display->print(pose.z, 1);
        display->println();
        
        // Show controls
        display->println(F("R Joy X: Select joint"));
        display->println(F("L Joy X: +/- Movement"));
        
        display->display();
    }
    
    void displayKinematicMode(RobotKinematics* kinematics) {
        if (!display || !kinematics) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("KINEMATIC MODE"));
        
        // Display joint angles in degrees (first row)
        JointAngles angles = kinematics->getCurrentJointAngles();
        
        for (int i = 0; i < 3; i++) {
            display->print(F("J"));
            display->print(i+1);
            display->print(F(":"));
            display->print(angles.angles[i] * 180.0f / M_PI, 0);
            display->print(F(" "));
        }
        display->println();
        
        // Display joint angles (second row)
        for (int i = 3; i < 6; i++) {
            display->print(F("J"));
            display->print(i+1);
            display->print(F(":"));
            display->print(angles.angles[i] * 180.0f / M_PI, 0);
            display->print(F(" "));
        }
        display->println();
        
        // Display end effector position and orientation
        CartesianPose pose = kinematics->getCurrentPose();
        display->print(F("X:"));
        display->print(pose.x, 1);
        display->print(F(" Y:"));
        display->println(pose.y, 1);
        display->print(F("Z:"));
        display->print(pose.z, 1);
        display->print(F(" Yaw:"));
        display->println(pose.yaw * 180.0f / M_PI, 1);
        
        // Display reach information
        
        float maxReach = kinematics->getConfig().dhParams[1].a + 
                         kinematics->getConfig().dhParams[2].a;
        float currentDistance = sqrt(pose.x*pose.x + pose.y*pose.y +
                                  (pose.z-kinematics->getConfig().dhParams[0].d)*
                                  (pose.z-kinematics->getConfig().dhParams[0].d));
        
        display->print(F("Dist:"));
        display->print(currentDistance, 1);
        display->print(F("/"));
        display->print(maxReach, 1);
        display->println(F("mm"));
        
        display->display();
    }
    
    void displayPositionMode(RobotKinematics* kinematics) {
        if (!display || !kinematics) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("POSITION MODE"));
        
        // Display joint angles in degrees (first row)
        JointAngles angles = kinematics->getCurrentJointAngles();
        
        for (int i = 0; i < 3; i++) {
            display->print(F("J"));
            display->print(i+1);
            display->print(F(":"));
            display->print(angles.angles[i] * 180.0f / M_PI, 0);
            display->print(F(" "));
        }
        display->println();
        
        // Display joint angles (second row)
        for (int i = 3; i < 6; i++) {
            display->print(F("J"));
            display->print(i+1);
            display->print(F(":"));
            display->print(angles.angles[i] * 180.0f / M_PI, 0);
            display->print(F(" "));
        }
        display->println();
        
        // Display end effector position
        CartesianPose pose = kinematics->getCurrentPose();
        display->print(F("X:"));
        display->print(pose.x, 1);
        display->print(F(" Y:"));
        display->println(pose.y, 1);
                display->print(F("Z:"));
        display->print(pose.z, 1);
        display->println();
        
        // Show controls
        display->println(F("L Joy: XY movement"));
        display->println(F("R Joy: Z movement"));
        
        display->display();
    }
    
    void displayHomingMenu(int selection) {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("HOMING MENU"));
        display->println(F("--------------------"));
        display->println(F("Select with R-Joy:"));
        display->println();
        
        const char* menuItems[] = {
            "Start Homing",
            "Move to Center",
            "Save Home Position",
            "Load Home Position",
            "Clear Home Position"
        };
        
        const int menuCount = 5;
        const int visibleItems = 3;
        
        // Calculate which items to show based on current selection
        int startItem = selection - 1;
        if (startItem < 0) startItem = 0;
        if (startItem > menuCount - visibleItems) startItem = menuCount - visibleItems;
        if (startItem < 0) startItem = 0;
        
        // Display menu items
        for (int i = startItem; i < startItem + visibleItems && i < menuCount; i++) {
            if (i == selection) {
                display->print(F("> "));
            } else {
                display->print(F("  "));
            }
            display->println(menuItems[i]);
        }
        
        display->println();
        display->println(F("Press L-Button to select"));
        
        display->display();
    }
    
    void displayHomingProgress(int jointIndex) {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("HOMING MODE"));
        display->println(F("----------------"));
        
        display->print(F("Homing Joint "));
        display->println(jointIndex + 1);
        
        // Draw progress bar
        drawProgressBar(0, 30, 128, 10, jointIndex / 6.0f);
        
        display->setCursor(0, 45);
        display->println(F("Please wait..."));
        
        display->display();
    }
    
    void displayCenteringProgress(float progress) {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("MOVING TO CENTER"));
        display->println(F("--------------------"));
        display->println(F("Processing..."));
        display->println();
        
        // Draw progress bar
        drawProgressBar(14, 40, 100, 8, progress);
        
        display->display();
    }
    
    void displayCalibrationMode() {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("CALIBRATION MODE"));
        display->println(F("----------------"));
        display->println(F("Press left button"));
        display->println(F("to start calibration"));
        display->println();
        display->println(F("Right button: Switch mode"));
        
        display->display();
    }
    
    void displayGCodeMode() {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("GCODE MODE"));
        display->println(F("----------------"));
        display->println(F("Processing GCode..."));
        
        // Add GCode-specific information here
        
        display->display();
    }
    
    void displayRecordMode() {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("RECORD MODE"));
        display->println(F("----------------"));
        
        // Add recording-specific information here
        
        display->display();
    }
    
    void displayPlayMode() {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("PLAYBACK MODE"));
        display->println(F("----------------"));
        
        // Add playback-specific information here
        
        display->display();
    }
    
    void displayConfigMenu() {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 0);
        display->println(F("CONFIGURATION MENU"));
        display->println(F("----------------"));
        
        // Add config menu items here
        
        display->display();
    }
    
    void showMessage(const char* line1, const char* line2, int duration) {
        if (!display) return;
        
        display->clearDisplay();
        display->setCursor(0, 10);
        display->println(line1);
        
        if (line2) {
            display->setCursor(0, 30);
            display->println(line2);
        }
        
        display->display();
        
        if (duration > 0) {
            delay(duration);
        }
    }
    
    void drawProgressBar(int x, int y, int width, int height, float progress) {
        if (!display) return;
        
        // Draw outline
        display->drawRect(x, y, width, height, SSD1306_WHITE);
        
        // Calculate fill width
        int fillWidth = progress * width;
        if (fillWidth < 0) fillWidth = 0;
        if (fillWidth > width) fillWidth = width;
        
        // Fill bar
        if (fillWidth > 0) {
            display->fillRect(x, y, fillWidth, height, SSD1306_WHITE);
            
        }
    }
}

