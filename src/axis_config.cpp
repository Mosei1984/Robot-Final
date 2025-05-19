#include "axis_config.h"

// Achskonfiguration: Pins, Steps/Grad, Speed, Accel, Endstop, EndstopPin
AxisConfig axisConfigs[NUM_AXES] = {
  {2, 3, 4,   55.56f, 1000.0f, 500.0f, true, 22},   // Achse 0: Basis
  {5, 6, 7,   222.22f, 1000.0f, 500.0f, true, 23},  // Achse 1: Schulter
  {8, 9, 10,  44.44f, 1000.0f, 500.0f, true, 24},   // Achse 2: Ellenbogen
  {11, 12, 41, 33.33f, 1000.0f, 500.0f, true, 25},  // Achse 3: Wrist Pitch
  {14, 15, 16, 17.78f, 1000.0f, 500.0f, true, 26},  // Achse 4: Wrist Roll
  {17, 20, 21, 8.89f, 1000.0f, 500.0f, false, 255}  // Achse 5: Tool (kein Endschalter)
};

// Stepper-Objekte
AccelStepper* steppers[NUM_AXES];

void setupAxes() {
  for (int i = 0; i < NUM_AXES; i++) {
    AxisConfig& cfg = axisConfigs[i];

    steppers[i] = new AccelStepper(AccelStepper::DRIVER, cfg.stepPin, cfg.dirPin);
    steppers[i]->setMaxSpeed(cfg.maxSpeed);
    steppers[i]->setAcceleration(cfg.acceleration);
    steppers[i]->setCurrentPosition(0);

    // Debug-Pins für Logic Analyzer: Step & Dir
    pinMode(cfg.stepPin, OUTPUT);
    pinMode(cfg.dirPin, OUTPUT);
    digitalWrite(cfg.stepPin, LOW);
    digitalWrite(cfg.dirPin, LOW);

    // Enable-Pin initialisieren
    pinMode(cfg.enablePin, OUTPUT);
    digitalWrite(cfg.enablePin, LOW); // Motor aktivieren (LOW = ON)

    // Endschalter (wenn vorhanden)
    if (cfg.hasEndstop) {
      pinMode(cfg.endstopPin, INPUT_PULLUP);
    }

    // Debug-Infos
    Serial.printf("Stepper %d Pins - STEP: %d, DIR: %d, ENABLE: %d\n", i, cfg.stepPin, cfg.dirPin, cfg.enablePin);
    Serial.printf("Stepper %d Config - MaxSpeed: %.2f, Accel: %.2f, StepsPerDegree: %.2f\n",
                  i, cfg.maxSpeed, cfg.acceleration, cfg.stepsPerDegree);
  }
}

void enableAllSteppers() {
  for (int i = 0; i < NUM_AXES; i++) {
    digitalWrite(axisConfigs[i].enablePin, LOW); // Aktivieren (LOW)
  }
}

void disableAllSteppers() {
  for (int i = 0; i < NUM_AXES; i++) {
    digitalWrite(axisConfigs[i].enablePin, HIGH); // Deaktivieren (HIGH)
  }
}

bool isEndstopTriggered(int axis) {
  if (axisConfigs[axis].hasEndstop) {
    return digitalRead(axisConfigs[axis].endstopPin) == LOW;
  }
  return false; // Kein Endschalter
}

void resetAllStepperPositions() {
  for (int i = 0; i < NUM_AXES; i++) {
    steppers[i]->setCurrentPosition(0);
  }
}
