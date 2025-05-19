#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

// Struktur zur Speicherung von Kalibrierungswerten
struct AxisCalibration {
  int minVal;
  int maxVal;
  int centerVal;
};

// Anzahl Joystick-Achsen (z. B. 4: X, Y, Z, Yaw)
#define NUM_JOYSTICK_AXES 4

// Globale Kalibrierungsdaten
extern AxisCalibration calibrationData[NUM_JOYSTICK_AXES];

// Kalibrierung durchführen
void performJoystickCalibration();

// Mapping der Rohwerte auf -500 bis +500 Bereich
int mapJoystickValue(int rawValue, int axisIndex);

// Speichern/Laden der Kalibrierung
void saveCalibrationToEEPROM();
void loadCalibrationFromEEPROM();

// Debug/Testpunkt-Ausgabe
void printCalibrationData();

#endif
