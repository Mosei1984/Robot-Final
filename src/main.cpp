#include <Arduino.h>
#include <Wire.h>
#include "mode_controller.h"
#include "joystick.h"
#include "axis_config.h"

// Globale Instanzen/Variablen
extern AccelStepper* steppers[NUM_AXES];      // Stepper aus axis_config.h
extern AxisConfig axisConfigs[NUM_AXES];      // Achskonfiguration

// Status-LED/Debug-Testpunkte (siehe platformio.ini build_flags)
#define TESTPOINT_INIT_DONE    29
#define TESTPOINT_SD_FAIL      28
#define TESTPOINT_0           22
#define TESTPOINT_1           23
#define TESTPOINT_2           24
#define TESTPOINT_3           25
#define TESTPOINT_4           26
#define TESTPOINT_5           27

void setup() {
  Serial.begin(115200);
  delay(2000); // Warte auf Serial Monitor

  // Debug/Testpunkt: Initialisierung startet
  pinMode(TESTPOINT_INIT_DONE, OUTPUT);
  digitalWrite(TESTPOINT_INIT_DONE, LOW);

  // Stepper/Achsen initialisieren
  setupAxes();

  // Joystick initialisieren und ggf. kalibrieren
  initializeJoystick();
  if (!joystickCalibrated) {
    Serial.println("Starte Kalibrierung!");
    calibrateJoystick();
  }

  // Modus-/Menücontroller initialisieren (inkl. OLED & ADXL)
  ModeController::init();

  // Debug: Initialisierung abgeschlossen
  digitalWrite(TESTPOINT_INIT_DONE, HIGH);

  Serial.println("Setup abgeschlossen. Warte auf Eingaben...");
}

void loop() {
  // Modus-Schalter prüfen und ggf. Menü/Anzeige wechseln
  ModeController::update();

  // Joystick-Werte regelmäßig auslesen (z.B. für Menü-Scroll, spätere Steuerung)
  JoystickValues jv = readJoystick();
  debugJoystickValues();

  // Optional: Testpunkt toggeln für Oszi/Logic-Analyzer
  digitalWrite(TESTPOINT_0, !digitalRead(TESTPOINT_0));

  // Hier Platz für weitere Routinen:
  // - Input Shaping: ADXL345 auswerten und anzeigen
  // - Stepper-Steuerung (Kinematik/Move-Aufruf)
  // - Menü-/Display-Logik

  delay(20); // Loop-Rate (50 Hz)
}

