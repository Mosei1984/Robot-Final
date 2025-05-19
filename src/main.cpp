#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include "config.h"
#include "Debug.h"
#include "TimerLoop.h"
#include "stepper_control.h"
#include "display_system.h"
#include "joystick_system.h"
#include "robot_system.h"
#include "mode_controller.h"

// Externe Konfigurationsvariablen
extern PinConfig _pinConfig;
extern JoystickConfig _joystickConfig;
extern StepperConfig _stepperConfig[6];
extern AccelStepper* _steppers[6];

// Funktionsdeklarationen
void controlCallback();
void joystickCallback();
void displayCallback();

// SD-Karten-Pin (fehlte vorher)
#define SD_CS_PIN 10  // Setzen Sie den tatsächlichen Wert für Ihren Pin

void setup() {
    // Serielle Kommunikation initialisieren
    Serial.begin(115200);
    Serial.println(F("6DOF Robot Controller Initializing..."));
    
    // Debug-Ausgabe aktivieren
    Debug::enabled = true;
    Debug::println(F("Debug output enabled"));
    
    // Konfiguration laden
    ConfigSystem::init();
    
    // Hardware initialisieren
    pinMode(_pinConfig.errorLedPin, OUTPUT);
    digitalWrite(_pinConfig.errorLedPin, LOW);  // Anfangs kein Fehler
    
    // I2C initialisieren
    Wire.begin();
    
    // Subsysteme initialisieren
    DisplaySystem::init();
    JoystickSystem::init();
    StepperSystem::init();
    RobotSystem::init();
    ModeController::init();
    
    // SD-Karte initialisieren, falls vorhanden
    if (!SD.begin(SD_CS_PIN)) {
        Debug::println(F("SD card initialization failed. Continuing without SD."));
    } else {
        Debug::println(F("SD card initialized successfully."));
    }
    
    // Timer für Echtzeitsteuerung starten
    TimerLoop::begin(controlCallback);
    
    Debug::println(F("Initialization complete, starting main loop."));
}

void loop() {
    // Haupttimer-Schleife verarbeitet alles
    TimerLoop::loop(joystickCallback, displayCallback);
    
    // Überwachung für Notaus
    if (Serial.available() && Serial.read() == '!') {
        // Notaus - alle Motoren deaktivieren
        StepperSystem::disableAllMotors();
        Debug::println(F("EMERGENCY STOP triggered!"));
        DisplaySystem::showMessage("EMERGENCY STOP", "System halted!", 0);
        
        // Auf Reset warten
        while(true) {
            // Display weiter aktualisieren
            DisplaySystem::update();
            delay(100);
        }
    }
}

// Control-Callback - läuft mit 1kHz vom Timer
void controlCallback() {
    // Stepper-Motoren aktualisieren
    StepperSystem::update();
}

// Joystick-Callback - läuft mit 100Hz
void joystickCallback() {
    // Joystick-Werte aktualisieren
    JoystickSystem::update();
    
    // Roboterzustand verarbeiten
    RobotSystem::update();
    
    // Modus-Controller aktualisieren
    ModeController::update();
}

// Display-Callback - läuft mit 50Hz
void displayCallback() {
    // Display-Informationen aktualisieren
    DisplaySystem::update();
}
