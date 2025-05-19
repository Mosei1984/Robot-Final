#pragma once
#include <Arduino.h>
#include <SD.h>
#include "axis_config.h"

// Initialisiert die SD-Karte. Liefert true bei Erfolg, false bei Fehler.
bool initialisiereSD() {
    // SD-Karten-Initialisierung am definierten CS-Pin
    if (!SD.begin(TP_SD_CS)) {
        return false;
    }
    return true;
}

// Lädt die Achsen-Konfiguration (Übersetzungen und Richtungen) aus der JSON-Datei.
// Bei Fehler (SD nicht verfügbar oder Datei fehlt/fehlerhaft) werden Standardwerte gesetzt.
// Gibt true zurück, wenn erfolgreich (inkl. Fallback), false bei kritischem Fehler.
bool ladeAchsenKonfiguration() {
    bool success = false;
    if (SD.exists(GEAR_CONFIG_FILE)) {
        File file = SD.open(GEAR_CONFIG_FILE, FILE_READ);
        if (file) {
            // Gesamten Datei-Inhalt einlesen
            String content = "";
            while (file.available()) {
                content += (char)file.read();
            }
            file.close();
            // JSON-Inhalt auswerten (einfaches Parsen nach bekannten Schlüsseln)
            int idx = content.indexOf("gearRatios");
            if (idx >= 0) {
                int startBracket = content.indexOf('[', idx);
                int endBracket = content.indexOf(']', startBracket);
                if (startBracket >= 0 && endBracket >= 0) {
                    // Werte zwischen den Klammern extrahieren und ins Array schreiben
                    int start = startBracket + 1;
                    for (int i = 0; i < NUM_AXES; i++) {
                        int end = (i < NUM_AXES - 1) ? content.indexOf(',', start) : content.indexOf(']', start);
                        String token = content.substring(start, end);
                        token.trim();
                        gearRatio[i] = token.toFloat();
                        start = end + 1;
                    }
                    success = true;
                }
            }
            idx = content.indexOf("invertMotorDirection");
            if (idx >= 0) {
                int startBracket = content.indexOf('[', idx);
                int endBracket = content.indexOf(']', startBracket);
                if (startBracket >= 0 && endBracket >= 0) {
                    int start = startBracket + 1;
                    for (int i = 0; i < NUM_AXES; i++) {
                        int end = (i < NUM_AXES - 1) ? content.indexOf(',', start) : content.indexOf(']', start);
                        String token = content.substring(start, end);
                        token.trim();
                        // Token zu bool konvertieren
                        if (token == "true" || token == "True" || token == "1") {
                            invertDirection[i] = true;
                        } else {
                            invertDirection[i] = false;
                        }
                        start = end + 1;
                    }
                } else {
                    // Falls invertMotorDirection fehlt/fehlerhaft, Standard (false) annehmen
                    for (int i = 0; i < NUM_AXES; i++) {
                        if (!success) gearRatio[i] = 1.0;
                        invertDirection[i] = false;
                    }
                }
            } else {
                // Kein invertMotorDirection vorhanden
                for (int i = 0; i < NUM_AXES; i++) {
                    if (!success) gearRatio[i] = 1.0;
                    invertDirection[i] = false;
                }
            }
        }
    }
    if (!success) {
        // SD nicht verfügbar oder Config nicht geladen -> Standardwerte verwenden
        Serial.println(F("[SD] Konnte Konfig nicht laden, Standardwerte werden genutzt."));
        float defaultGear[NUM_AXES] = {6.25, 25.0, 5.0, 3.75, 2.0, 1.0};
        bool defaultInvert[NUM_AXES] = {false, true, false, true, false, false};
        for (int i = 0; i < NUM_AXES; i++) {
            gearRatio[i] = defaultGear[i];
            invertDirection[i] = defaultInvert[i];
        }
    }
    // Schritte pro Grad für jede Achse berechnen
    for (int i = 0; i < NUM_AXES; i++) {
        stepsPerDeg[i] = (gearRatio[i] * STEPS_PER_REV) / 360.0f;
    }
    return true;
}
