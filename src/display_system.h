#ifndef DISPLAY_SYSTEM_H
#define DISPLAY_SYSTEM_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RobotKinematics.h"
#include "Debug.h"

namespace DisplaySystem {
    // Initialisierung
    void init();
    
    // Haupt-Update-Funktion
    void update();
    
    // Zugriff auf das Display-Objekt
    Adafruit_SSD1306* getDisplay();
    
    // Verschiedene Anzeigemodi
    void displayStartupScreen();
    void displayJointMode(RobotKinematics* kinematics, int selectedJoint);
    void displayPositionMode(RobotKinematics* kinematics);
    void displayFullPoseMode(RobotKinematics* kinematics);
    void displayTeachingMode();
    void displayCalibrationMode();
    void displayHomingMode(int jointIndex = -1);
    void displayHomingMenu(int selection, int totalOptions);
    void displaySettingsMenu();
    void displayDebugScreen();
    
    // Spezialisierte Anzeigefunktionen
    void showMessage(const char* line1, const char* line2 = nullptr, int delayMs = 0);
    void showModeChange(int newMode);
    void showError(const char* errorMsg);
    void showCalibrationInProgress();
    void showCalibrationComplete();
    
    // Fortschrittsanzeige
    void displayHomingProgress(int jointIndex, float progress = 0.0f);
    void displayHomingCenterProgress(float progress);
    void drawProgressBar(int x, int y, int width, int height, float progress);
    
    // Getriebe-Menü-Funktionen
    void displayGearMenu();
    void processGearMenu();
    void saveGearConfigToSD();
    bool loadGearConfigFromSD();
    bool isGearMenuActive();
    void setGearMenuActive(bool active);
    int getSelectedGearAxis();
    void setSelectedGearAxis(int axis);
    
    // Home-Position-Verwaltung
    void saveHomePositionToSD(const JointAngles& angles);
    bool loadHomePositionFromSD(JointAngles& angles);
    void listHomePositionsOnSD();
    void deleteHomePositionFromSD(const char* filename);
}

#endif // DISPLAY_SYSTEM_H
