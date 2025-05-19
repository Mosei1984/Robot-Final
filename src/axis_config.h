#ifndef AXIS_CONFIG_H
#define AXIS_CONFIG_H

#include <Arduino.h>
#include <AccelStepper.h>

// Anzahl der Achsen
#define NUM_AXES 6
#define TP_SD_CS  10                // Setze hier deinen echten SD-Karten-CS-Pin
#define GEAR_CONFIG_FILE "/gear_config.json"
#define NUM_AXES 6
#define STEPS_PER_REV 800     
// Struktur für Achskonfiguration
struct AxisConfig {
  uint8_t stepPin;
  uint8_t dirPin;
  uint8_t enablePin;
  float stepsPerDegree;
  float maxSpeed;
  float acceleration;
  bool hasEndstop;
  uint8_t endstopPin;
};

// Globale Konfigurationsarrays
extern AxisConfig axisConfigs[NUM_AXES];
extern AccelStepper* steppers[NUM_AXES];
extern float gearRatio[NUM_AXES];
extern bool invertDirection[NUM_AXES];
extern float stepsPerDeg[NUM_AXES];

// Funktionen zur Initialisierung und Steuerung
void setupAxes();
void enableAllSteppers();
void disableAllSteppers();
bool isEndstopTriggered(int axis);
void resetAllStepperPositions();

#endif
