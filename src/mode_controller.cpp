#include "mode_controller.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// Globale OLED-Instanz (I2C, Default-Adresse 0x3C)
static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);

// Initialisieren statischer Mitglieder
ModeController::Mode ModeController::currentMode = ModeController::MODE_POSITION_ONLY;
bool ModeController::buttonWasPressed = false;
unsigned long ModeController::buttonPressTime = 0;
bool ModeController::debugState = false;

void ModeController::init() {
    // Pin-Konfiguration für Mode-Schalter und Debug-Pin
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(DEBUG_MODE_PIN, OUTPUT);
    digitalWrite(DEBUG_MODE_PIN, LOW);
    debugState = false;
    buttonWasPressed = false;
    buttonPressTime = 0;

    // (Optional) Modus aus EEPROM/SD laden - hier implementierbar
    // uint8_t savedMode = EEPROM.read(Adresse);
    // if (savedMode < 3) currentMode = (Mode) savedMode;

    // ADXL345 initialisieren (bereit für Stabilitätsmessung)
    initADXL();

    // OLED-Display initialisieren
    initDisplay();

    // Watchdog-Hook (vorbereitet, hier noch ohne konkrete Umsetzung)
    setupWatchdog();

    // (Optional) Anfangsmodus anzeigen
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Anfangsmodus:"));
    switch(currentMode) {
        case MODE_POSITION_ONLY:
            display.println(F("Position Only"));
            break;
        case MODE_POSITION_ORIENTATION:
            display.println(F("Position+Orientation"));
            break;
        case MODE_JOINT:
            display.println(F("Joint Mode"));
            break;
    }
    display.display();
    delay(1000);
}

void ModeController::update() {
    // Modus-Taster (active low) auslesen
    bool pressed = (digitalRead(MODE_SWITCH_PIN) == LOW);
    if (pressed && !buttonWasPressed) {
        // Erster Tastendruck erkannt (Entprellung)
        delay(50);
        if (digitalRead(MODE_SWITCH_PIN) == LOW) {
            buttonWasPressed = true;
            buttonPressTime = millis();
        }
    }
    else if (!pressed && buttonWasPressed) {
        // Taster wurde losgelassen
        if (millis() - buttonPressTime >= BUTTON_PRESS_TIME) {
            // Kurzer Tastendruck (>=0.5s) erkannt -> Modus wechseln
            switchMode();
        }
        buttonWasPressed = false;
    }
}

// Gibt den aktuellen Modus zurück
ModeController::Mode ModeController::getCurrentMode() {
    return currentMode;
}

void ModeController::switchMode() {
    // Nächsten Modus zyklisch einstellen
    currentMode = (Mode)((currentMode + 1) % 3);

    // Debug-Pin toggeln (HIGH/LOW wechseln)
    toggleDebugPin();

    // Anzeige und Animation für den neuen Modus
    showAnimation();

    // Debug-Ausgabe zum seriellen Monitor
    Serial.print(F("Modus gewechselt auf "));
    switch(currentMode) {
        case MODE_POSITION_ONLY:
            Serial.println(F("Position Only"));
            break;
        case MODE_POSITION_ORIENTATION:
            Serial.println(F("Position+Orientation"));
            break;
        case MODE_JOINT:
            Serial.println(F("Joint Mode"));
            break;
    }

    // (Optional) Neuen Modus in EEPROM/SD speichern
    // EEPROM.write(Adresse, (uint8_t)currentMode);
}

void ModeController::showAnimation() {
    // Textanzeige für den aktuellen Modus
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    switch(currentMode) {
        case MODE_POSITION_ONLY:
            display.println(F("Modus 1: Position Only"));
            break;
        case MODE_POSITION_ORIENTATION:
            display.println(F("Modus 2: Position + Orientation"));
            break;
        case MODE_JOINT:
            display.println(F("Modus 3: Joint Mode"));
            break;
    }
    display.display();
    delay(1000);

    // Kleine Animation je Modus (wird einmal abgespielt)
    switch(currentMode) {
        case MODE_POSITION_ONLY:
            // Modus 1: gesamter "Arm" bewegt sich in 3 Positionen
            for (int i = 0; i < 3; i++) {
                display.clearDisplay();
                if (i == 0) {
                    // Arm vertikal nach oben
                    display.drawLine(64, 12, 64, 52, SSD1306_WHITE);
                    display.drawLine(64, 52, 44, 62, SSD1306_WHITE);
                } else if (i == 1) {
                    // Arm diagonal
                    display.drawLine(64, 20, 84, 52, SSD1306_WHITE);
                    display.drawLine(84, 52, 104, 62, SSD1306_WHITE);
                } else {
                    // Arm horizontal nach rechts
                    display.drawLine(64, 32, 96, 32, SSD1306_WHITE);
                    display.drawLine(96, 32, 116, 52, SSD1306_WHITE);
                }
                display.display();
                delay(300);
            }
            break;

        case MODE_POSITION_ORIENTATION:
            // Modus 2: nur Werkzeugteil rotiert (Linie wechselt von vertikal zu horizontal)
            display.clearDisplay();
            // Werkzeugteil vertikal
            display.drawLine(64, 32, 64, 12, SSD1306_WHITE);
            display.display();
            delay(300);
            display.clearDisplay();
            // Werkzeugteil horizontal
            display.drawLine(64, 32, 84, 32, SSD1306_WHITE);
            display.display();
            delay(300);
            break;

        case MODE_JOINT:
            // Modus 3: Gelenkmodus (Plus- und Kreuz-Symbole)
            // Plus-Symbol
            display.clearDisplay();
            display.drawLine(64, 12, 64, 52, SSD1306_WHITE);
            display.drawLine(32, 32, 96, 32, SSD1306_WHITE);
            display.display();
            delay(300);
            // Kreuz-Symbol (diagonal)
            display.clearDisplay();
            display.drawLine(44, 12, 84, 52, SSD1306_WHITE);
            display.drawLine(44, 52, 84, 12, SSD1306_WHITE);
            display.display();
            delay(300);
            break;
    }

    // Anzeige des Modus-Text nach der Animation (ruhig verbleibend)
    display.clearDisplay();
    display.setCursor(0, 0);
    switch(currentMode) {
        case MODE_POSITION_ONLY:
            display.println(F("Modus 1: Position Only"));
            break;
        case MODE_POSITION_ORIENTATION:
            display.println(F("Modus 2: Position + Orientation"));
            break;
        case MODE_JOINT:
            display.println(F("Modus 3: Joint Mode"));
            break;
    }
    display.display();
}

void ModeController::initADXL() {
    // ADXL345 über I2C initialisieren (Standardadresse 0x53)
    const uint8_t ADXL_ADDR = 0x53;
    const uint8_t REG_POWER_CTL = 0x2D;
    const uint8_t REG_DATA_FORMAT = 0x31;
    const uint8_t REG_BW_RATE = 0x2C;

    Wire.begin();
    // Messmodus einschalten
    Wire.beginTransmission(ADXL_ADDR);
    Wire.write(REG_POWER_CTL);
    Wire.write(0x08);
    Wire.endTransmission();

    // ±16g Vollauflösung
    Wire.beginTransmission(ADXL_ADDR);
    Wire.write(REG_DATA_FORMAT);
    Wire.write(0x0B);
    Wire.endTransmission();

    // Datenrate 3200 Hz
    Wire.beginTransmission(ADXL_ADDR);
    Wire.write(REG_BW_RATE);
    Wire.write(0x0F);
    Wire.endTransmission();
}

void ModeController::initDisplay() {
    // Initialisiert das OLED-Display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        // Fehlerbehandlung, falls Display nicht gefunden wurde
        Serial.println(F("OLED-Display konnte nicht initialisiert werden!"));
        return;
    }
    display.clearDisplay();
    display.display();
}

void ModeController::setupWatchdog() {
    // TODO: Watchdog initialisieren (z.B. Timer, Interrupt)
    // (Aktuell nur Platzhalter)
}

void ModeController::toggleDebugPin() {
    debugState = !debugState;
    digitalWrite(DEBUG_MODE_PIN, debugState ? HIGH : LOW);
}
