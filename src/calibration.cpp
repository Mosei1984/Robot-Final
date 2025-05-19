#include "calibration.h"
#include <EEPROM.h>

AxisCalibration calibrationData[NUM_JOYSTICK_AXES];

// EEPROM-Adresse für Start der Kalibrierung
const int EEPROM_CALIBRATION_START = 0;

void performJoystickCalibration() {
  Serial.println(F("Starte Joystick-Kalibrierung..."));
  delay(500);

  for (int axis = 0; axis < NUM_JOYSTICK_AXES; axis++) {
    int minVal = 1023;
    int maxVal = 0;
    int samples = 200;

    Serial.printf("→ Achse %d: Bitte Min-Max ausfahren...\n", axis);

    for (int i = 0; i < samples; i++) {
      int val = analogRead(axis); // ACHTUNG: Achse = Pin! Muss angepasst werden!
      minVal = min(minVal, val);
      maxVal = max(maxVal, val);
      delay(10);
    }

    calibrationData[axis].minVal = minVal;
    calibrationData[axis].maxVal = maxVal;
    calibrationData[axis].centerVal = (minVal + maxVal) / 2;

    Serial.printf("Achse %d Kalibriert: min=%d | max=%d | center=%d\n",
                  axis, minVal, maxVal, calibrationData[axis].centerVal);
  }

  saveCalibrationToEEPROM();
}

// Mapped Rohwert auf -500 bis +500 Bereich
int mapJoystickValue(int rawValue, int axisIndex) {
  AxisCalibration cal = calibrationData[axisIndex];
  if (rawValue < cal.centerVal) {
    return map(rawValue, cal.minVal, cal.centerVal, -500, 0);
  } else {
    return map(rawValue, cal.centerVal, cal.maxVal, 0, 500);
  }
}

// Speichert alle Kalibrierungsdaten in EEPROM
void saveCalibrationToEEPROM() {
  for (int i = 0; i < NUM_JOYSTICK_AXES; i++) {
    int addr = EEPROM_CALIBRATION_START + i * sizeof(AxisCalibration);
    EEPROM.put(addr, calibrationData[i]);
  }
  Serial.println(F("Kalibrierung in EEPROM gespeichert."));
}

// Lädt Kalibrierungsdaten aus EEPROM
void loadCalibrationFromEEPROM() {
  for (int i = 0; i < NUM_JOYSTICK_AXES; i++) {
    int addr = EEPROM_CALIBRATION_START + i * sizeof(AxisCalibration);
    EEPROM.get(addr, calibrationData[i]);
  }
  Serial.println(F("Kalibrierung aus EEPROM geladen."));
}

// Debug-Funktion: Gibt Kalibrierungswerte aus
void printCalibrationData() {
  Serial.println(F("=== Kalibrierungsdaten ==="));
  for (int i = 0; i < NUM_JOYSTICK_AXES; i++) {
    Serial.printf("Achse %d → Min: %d | Max: %d | Center: %d\n",
                  i,
                  calibrationData[i].minVal,
                  calibrationData[i].maxVal,
                  calibrationData[i].centerVal);
  }
  Serial.println("===========================");
}
