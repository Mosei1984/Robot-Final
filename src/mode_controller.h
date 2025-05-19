#ifndef MODE_CONTROLLER_H
#define MODE_CONTROLLER_H

#include <Arduino.h>

// Betriebsmodi des Roboters
enum Mode {
    MODE_JOINT,              // Gelenksteuerung
    MODE_POSITION_ONLY,      // Positionssteuerung (X,Y,Z)
    MODE_FULL_POSE,          // Vollständige Pose-Steuerung (X,Y,Z,Roll,Pitch,Yaw)
    MODE_TEACHING,           // Teach-In-Modus
    MODE_HOMING,             // Referenzierungsmodus
    MODE_SETTINGS,           // Einstellungsmenü
    MODE_DEBUG               // Debug-Modus
};

// Debug-Level
enum DebugLevel {
    DEBUG_NONE = 0,
    DEBUG_ERROR,
    DEBUG_WARNING,
    DEBUG_INFO,
    DEBUG_VERBOSE
};

namespace ModeController {
    // Initialisierung
    void init();
    
    // Aktualisierung (wird vom Hauptloop aufgerufen)
    void update();
    
    // Zeitüberwachung für Langdruck
    void handleLongPress();
    
    // Modus-Funktionen
    Mode getCurrentMode();
    void setMode(Mode newMode);
    const char* getModeString();
    
    // Debug-Funktionen
    void toggleDebug();
    bool isDebugEnabled();
    void setDebugLevel(DebugLevel level);
    DebugLevel getDebugLevel();
    
    // Systemzustand aktualisieren
    void updateSystemState();
    
    // Externe Variablen für Zustandsverwaltung
    extern bool buttonWasPressed;
    extern unsigned long buttonPressTime;
    extern bool debugState;
}

#endif // MODE_CONTROLLER_H
