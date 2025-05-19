#ifndef JOYSTICK_SYSTEM_H
#define JOYSTICK_SYSTEM_H

#include <Arduino.h>
#include "joystick.h"
#include "kalmanfilter.h"

// Struktur für Joystick-Werte
struct JoystickValues {
    float leftX;
    float leftY;
    float rightX;
    float rightY;
    bool leftButton;
    bool rightButton;
};

namespace JoystickSystem {
    // Initialisierung
    void init();
    
    // Hauptupdate-Funktion
    void update();
    
    // Joystick-Werte lesen
    JoystickValues readValues();
    
    // Zugriff auf Joystick-Objekte
    Joystick* getLeftJoystick();
    Joystick* getRightJoystick();
    
    // Kalibrierungsfunktionen
    void calibrateJoysticks();
    void startFullCalibration();
    
    // Button-Input verarbeiten
    void processButtonInput();
    
    // Getter für Joystick-Verarbeitungsfunktionen
    JoystickValues getJoystickValues();
    bool isLeftButtonPressed();
    bool isRightButtonPressed();
    
    // Hilfsfunktionen für verschiedene Modi
    void processJointModeJoysticks();
    void processKinematicModeJoysticks();
    void processMenuJoysticks();
}

#endif // JOYSTICK_SYSTEM_H
