#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include <EEPROM.h>

// Joystick-Achsenpins (angepasst auf deine Hardware)
#define JOY_LEFT_X_PIN  41
#define JOY_LEFT_Y_PIN  40
#define JOY_RIGHT_X_PIN 38  // Yaw
#define JOY_RIGHT_Y_PIN 39  // Z

// Taster (für Modi / Kalibrierung)
#define JOY_MODE_BTN    27
#define JOY_CONFIRM_BTN 26

// Totzonenradius
#define JOYSTICK_DEADZONE 20

// Normalisierter Bereich: -500 bis +500
#define JOY_OUTPUT_MIN -500
#define JOY_OUTPUT_MAX  500

// EEPROM-Adresse (max. 8 Werte: min/max für 4 Achsen)
#define EEPROM_JOY_OFFSET 0

// Joystickstruktur für Normalwerte
struct JoystickValues {
  int leftX;
  int leftY;
  int rightX;
  int rightY;
};

// Initialisierung
void initializeJoystick();

// Lesen, Mappen, Filtern
JoystickValues readJoystick();

// Kalibrierung starten & Daten speichern
void calibrateJoystick();

// EEPROM laden
void loadJoystickCalibration();

// Debug-Ausgabe
void debugJoystickValues();
extern bool joystickCalibrated;

#endif
