#include "joystick.h"

struct AxisCalibration {
  int minVal;
  int maxVal;
};

AxisCalibration calibrationData[4];
bool joystickCalibrated = false;

// Aktuelle Normalwerte
JoystickValues currentValues;

// Initialisierung der Pins und Kalibrierung laden
void initializeJoystick() {
  pinMode(JOY_MODE_BTN, INPUT_PULLUP);
  pinMode(JOY_CONFIRM_BTN, INPUT_PULLUP);
  loadJoystickCalibration();
  Serial.println("Joystick system initialized");
}

// Normiert analogen Joystick-Wert (-500 bis +500) mit Totzone
int normalize(int raw, int minVal, int maxVal) {
  float norm = map(raw, minVal, maxVal, JOY_OUTPUT_MIN, JOY_OUTPUT_MAX);
  if (abs(norm) < JOYSTICK_DEADZONE) return 0;
  return constrain(norm, JOY_OUTPUT_MIN, JOY_OUTPUT_MAX);
}

// Lies aktuelle Werte (gemappt & normalisiert)
JoystickValues readJoystick() {
  currentValues.leftX  = normalize(analogRead(JOY_LEFT_X_PIN),  calibrationData[0].minVal, calibrationData[0].maxVal);
  currentValues.leftY  = normalize(analogRead(JOY_LEFT_Y_PIN),  calibrationData[1].minVal, calibrationData[1].maxVal);
  currentValues.rightX = normalize(analogRead(JOY_RIGHT_X_PIN), calibrationData[2].minVal, calibrationData[2].maxVal);
  currentValues.rightY = normalize(analogRead(JOY_RIGHT_Y_PIN), calibrationData[3].minVal, calibrationData[3].maxVal);
  return currentValues;
}

// Debug-Ausgabe für alle Achsen
void debugJoystickValues() {
  Serial.printf("Joystick values: LX:%d  LY:%d  RX:%d  RY:%d\n",
                currentValues.leftX, currentValues.leftY,
                currentValues.rightX, currentValues.rightY);
}

// Kalibrieren mit min/max-Erkennung
void calibrateJoystick() {
  Serial.println("Starte Joystick-Kalibrierung… Bitte jede Achse vollständig bewegen.");

  int axes[4] = {JOY_LEFT_X_PIN, JOY_LEFT_Y_PIN, JOY_RIGHT_X_PIN, JOY_RIGHT_Y_PIN};

  for (int i = 0; i < 4; i++) {
    calibrationData[i].minVal = 1023;
    calibrationData[i].maxVal = 0;
  }

  unsigned long calibrationStart = millis();
  while (millis() - calibrationStart < 5000) {
    for (int i = 0; i < 4; i++) {
      int val = analogRead(axes[i]);
      if (val < calibrationData[i].minVal) calibrationData[i].minVal = val;
      if (val > calibrationData[i].maxVal) calibrationData[i].maxVal = val;
    }
    delay(5);
  }

  // In EEPROM schreiben
  for (int i = 0; i < 4; i++) {
    EEPROM.put(EEPROM_JOY_OFFSET + i * sizeof(AxisCalibration), calibrationData[i]);
  }

 
  joystickCalibrated = true;
  Serial.println("Joystick-Kalibrierung abgeschlossen.");
}

// Lade Kalibrierwerte aus EEPROM
void loadJoystickCalibration() {
  for (int i = 0; i < 4; i++) {
    EEPROM.get(EEPROM_JOY_OFFSET + i * sizeof(AxisCalibration), calibrationData[i]);
    if (calibrationData[i].maxVal - calibrationData[i].minVal < 100) {
      joystickCalibrated = false;
      Serial.println("Ungültige Joystick-Kalibrierung gefunden!");
      return;
    }
  }
  joystickCalibrated = true;
  Serial.println("Joystick-Kalibrierung geladen.");
}
