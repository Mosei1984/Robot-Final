#include <Arduino.h>
#include "config.h"
#include "display.h"
#include "joystick.h"
#include "axis_config.h"
#include "calibration.h"
#include "stepper_control.h"
#include "robot_control.h"
#include "sdcard.h"
#include "menu.h"
#include "input_shaping.h"
#include "recordplay.h"
#include "gcode.h"

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(TESTPOINT_INIT_DONE, OUTPUT);
  digitalWrite(TESTPOINT_INIT_DONE, LOW); // LOW = Start läuft

  Serial.println("6DOF Robot Controller Starting...");

  if (!initSDCard()) {
    Serial.println("SD card initialization failed");
    digitalWrite(TESTPOINT_SD_FAIL, HIGH); // Fehleranzeige
  }

  initDisplay();       // OLED Anzeige
  initJoystick();      // Joystick mit Kalibrierung
  initStepperSystem(); // Motoren & Getriebe laden
  initRobot();         // FK/VK vorbereiten
  initRecordPlay();    // CSV Record & Playback
  initMenuSystem();    // Menü vorbereiten
  initGCode();         // GCode Parser aktivieren

  digitalWrite(TESTPOINT_INIT_DONE, HIGH); // System ready
  Serial.println("System initialized, starting in Homing mode");
  changeState(HOMING);
}

void loop() {
  handleMenu();       // Menüführung
  handleJoystick();   // Live-Steuerung
  updateDisplay();    // Statusanzeige
  updateRecordPlay(); // Record / Play Mode
  runRobot();         // Inverse Kinematik, Steuerung
  runGCode();         // GCode-Befehle ausführen
}
