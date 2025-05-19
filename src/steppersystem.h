#ifndef STEPPER_SYSTEM_H
#define STEPPER_SYSTEM_H

#include <Arduino.h>
#include <AccelStepper.h>

// Anzahl der Achsen/Gelenke
#define NUM_JOINTS 6

namespace StepperSystem {
    // Erweiterte Stepper-Struktur mit Konfiguration
    struct EnhancedStepper {
        AccelStepper* stepper;
        float stepsPerDegree;
        float homingSpeed;
        float maxSpeed;
        float acceleration;
        int limitSwitchPin;
        int enablePin;
    };
    
    // Array von Stepper-Motoren
    extern AccelStepper* steppers[NUM_JOINTS];
    
    // Array von erweiterten Stepper-Informationen
    extern EnhancedStepper enhancedSteppers[NUM_JOINTS];
    
    // Initialisiert das Stepper-System
    void init();
    
    // Aktualisiert die Stepper-Motoren (sollte in der Hauptschleife aufgerufen werden)
    void update();
    
    // Führt alle Stepper-Operationen aus
    void runSteppers();
    
    // Joint-Auswahl für manuelle Steuerung
    int getSelectedJoint();
    void setSelectedJoint(int joint);
    
    // Home-Funktion für eine bestimmte Achse
    bool homeJoint(int jointIndex);
    
    // Aktivieren/Deaktivieren von Motoren
    void enableAllMotors();
    void disableAllMotors();
}

#endif // STEPPER_SYSTEM_H
