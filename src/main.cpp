#include "config.h"
#include "joystick.h"
#include "calibration.h"
#include "axis_config.h"
#include "display.h"
#include "RobotKinematics.h"
#include "StepperController.h"
#include <ArduinoJson.h>
#include <SD.h>

// Globale Objekte
Joystick* leftJoystick;
Joystick* rightJoystick;
RobotKinematics* robotKin;
StepperController steppers;

// Zustände
enum SystemState {
  STATE_STARTUP,
  STATE_JOINT_MODE,
  STATE_KINEMATIC_MODE,
  STATE_HOMING_MODE,
  STATE_CALIBRATION_MODE
};
SystemState currentState = STATE_STARTUP;

// Weitere globale Variablen
unsigned long stateChangeTime = 0;
bool rightButtonPressed = false;
int selectedJoint = 0;

// Setup-Funktion
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("6DOF Roboter startet...");

  // Initialisierung: Pins, Joysticks, Stepper, Display
  loadDefaultPinConfig();
  pinMode(_pinConfig.errorLedPin, OUTPUT);
  
  Wire.begin();
  initDisplay();  // Im display.h enthalten
  displayStartupScreen();

  loadDefaultJoystickConfig();
  leftJoystick  = new Joystick(_pinConfig.leftXPin, _pinConfig.leftYPin, _pinConfig.leftBtnPin);
  rightJoystick = new Joystick(_pinConfig.rightXPin, _pinConfig.rightYPin, _pinConfig.rightBtnPin);
  leftJoystick->begin();
  rightJoystick->begin();

  loadAxesConfig();  // Invertierung/Skalierung
  Calibration::calibrateJoysticks(); // Zentrum-Kalibrierung

  loadDefaultStepperConfig();
  steppers.begin(); // Stepper + Endschalter konfigurieren

  initRobotConfig();  // DH-Parameter setzen
  robotKin = new RobotKinematics(robotConfig);

  // SD-Karte initialisieren
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD-Karte konnte nicht initialisiert werden!");
    digitalWrite(_pinConfig.errorLedPin, HIGH);
  } else {
    Serial.println("SD-Karte bereit.");
    loadOffsetsFromSD();  // Optional
    loadGearRatiosFromSD(); // Optional
  }

  currentState = STATE_HOMING_MODE;
  stateChangeTime = millis();
  Serial.println("Initialisierung abgeschlossen.");
}

// Haupt-Loop
void loop() {
  leftJoystick->read();
  rightJoystick->read();

  switch (currentState) {
    case STATE_STARTUP:
      displayStartupScreen();
      delay(2000);
      currentState = STATE_HOMING_MODE;
      break;

    case STATE_HOMING_MODE:
      processHomingMode();
      break;

    case STATE_CALIBRATION_MODE:
      displayCalibrationMode();
      if (leftJoystick->isPressed()) {
        startCalibration();
        currentState = STATE_JOINT_MODE;
      }
      break;

    case STATE_JOINT_MODE:
      processJointControl();
      break;

    case STATE_KINEMATIC_MODE:
      processKinematicControl();
      break;
  }

  // Buttons abfragen
  static bool lastRightBtn = false;
  bool currentRightBtn = rightJoystick->isPressed();
  if (currentRightBtn && !lastRightBtn) {
    toggleMode();  // Modus wechseln
  }
  lastRightBtn = currentRightBtn;
}
