#ifndef MODE_CONTROLLER_H
#define MODE_CONTROLLER_H

#include <Arduino.h>

// Pin-Definitionen für Modusumschalter und Debug-Pin
#define MODE_SWITCH_PIN 28
#define DEBUG_MODE_PIN 33

// OLED Display Parameter (z.B. 128x64 OLED)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET_PIN -1 // Reset-Pin bei SSD1306, -1 falls nicht verwendet

/**
 * @brief Klasse zur Steuerung der Betriebsmodi des Roboters.
 * 
 * Implementiert verschiedene Steuerungsmodi (Position, Position+Orientierung, Gelenksteuerung).
 * Überwacht den Taster zum Umschalten und aktualisiert OLED-Anzeige und Debug-Pin.
 */
class ModeController {
public:
    // Verfügbare Modi
    enum Mode {
        MODE_POSITION_ONLY = 0,       // Nur Position (XY-Z-Steuerung, Werkzeug waagerecht)
        MODE_POSITION_ORIENTATION,    // Position + Orientierung (XYZ + Roll/Pitch/Yaw)
        MODE_JOINT                    // Gelenksteuerung (direkt manuelle Ansteuerung)
    };

    /**
     * @brief Initialisiert den ModeController.
     * - Konfiguriert Pins und Display.
     * - Initialisiert ADXL345 (Stabilitätskontrolle vorbereitet).
     * - Initialisiert den Watchdog (Hook, derzeit ohne konkrete Umsetzung).
     * - Liest ggf. den gespeicherten Modus (EEPROM/SD) und setzt den aktuellen Modus.
     */
    static void init();

    /**
     * @brief Muss in der Hauptschleife aufgerufen werden.
     * Überwacht Taster-Eingaben zum Moduswechsel.
     */
    static void update();

    /**
     * @brief Liefert den aktuellen Modus zurück.
     */
    static Mode getCurrentMode();

private:
    // Tastendruck-Erkennung (geschaltet beim Loslassen nach 0,5s Haltezeit)
    static constexpr unsigned long BUTTON_PRESS_TIME = 500; // ms

    // Aktueller Modus und Hilfsvariablen
    static Mode currentMode;
    static bool buttonWasPressed;
    static unsigned long buttonPressTime;

    // Toggle-Status des Debug-Pins
    static bool debugState;

    // Methoden zur Implementierung der Funktionen
    static void switchMode();
    static void showAnimation();
    static void initADXL();
    static void initDisplay();
    static void setupWatchdog();
    static void toggleDebugPin();
};

#endif // MODE_CONTROLLER_H
