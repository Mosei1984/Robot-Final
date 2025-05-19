#pragma once
#include <Arduino.h>
#include <SD.h>
#include "axis_config.h"
#include "joystick.h"
#include "display.h"

// Globale Datei-Handle für Aufzeichnung/Wiedergabe
extern File aufzeichnungDatei;

// Startet eine neue Aufzeichnung: Öffnet/erstellt die CSV-Datei (überschreibt vorhandene).
bool starteAufzeichnung(const char* dateiname) {
    // Existierende Datei löschen, um neu zu beginnen
    if (SD.exists(dateiname)) {
        SD.remove(dateiname);
    }
    aufzeichnungDatei = SD.open(dateiname, FILE_WRITE);
    if (!aufzeichnungDatei) {
        return false;
    }
    // (Optional: CSV-Header schreiben - hier nicht benötigt)
    return true;
}

// Speichert die aktuelle Roboterposition (aller Achsen) als Zeile in der CSV-Datei.
void speichereAktuellePosition() {
    // Alle Achsenpositionen in Schritten erfassen und schreiben
    for (int i = 0; i < NUM_AXES; i++) {
        long steps = 0;
        extern AccelStepper* steppers[NUM_AXES];
        steps = steppers[i]->currentPosition();  // aktuelle Schrittposition der Achse
        aufzeichnungDatei.print(steps);
        if (i < NUM_AXES - 1) {
            aufzeichnungDatei.print(',');
        }
    }
    aufzeichnungDatei.println();
    aufzeichnungDatei.flush();  // sofort auf Karte schreiben
}

// Beendet die Aufzeichnung: Schließt die Datei.
void stoppeAufzeichnung() {
    if (aufzeichnungDatei) {
        aufzeichnungDatei.close();
    }
}

// Startet die Wiedergabe einer aufgezeichneten Sequenz von der CSV-Datei.
// Liest zeilenweise und bewegt die Achsen entsprechend. Gibt false bei Abbruch/Fehler zurück.
bool starteWiedergabe(const char* dateiname) {
    File file = SD.open(dateiname);
    if (!file) {
        return false;
    }
    char line[100];
    while (file.available()) {
        // Zeile auslesen
        int len = file.readBytesUntil('\n', line, sizeof(line) - 1);
        if (len <= 0) break;
        line[len] = '\0';
        // 6 Werte aus der Zeile parsen
        long targetSteps[NUM_AXES];
        int valuesRead = sscanf(line, "%ld,%ld,%ld,%ld,%ld,%ld",
                                 &targetSteps[0], &targetSteps[1], &targetSteps[2],
                                 &targetSteps[3], &targetSteps[4], &targetSteps[5]);
        if (valuesRead < NUM_AXES) {
            continue; // unvollständige Zeile überspringen
        }
        // Zielpositionen für alle Achsen setzen
        extern AccelStepper* steppers[NUM_AXES];
        for (int i = 0; i < NUM_AXES; i++) {
            steppers[i]->moveTo(targetSteps[i]);
        }
        // Achsen synchron bewegen (mit Beschleunigung)
        bool allDone = false;
        while (!allDone) {
            allDone = true;
            for (int i = 0; i < NUM_AXES; i++) {
                if (steppers[i]->distanceToGo() != 0) {
                    steppers[i]->run();
                    allDone = false;
                }
            }
            // Abbruchbedingung: Not-Aus
            if (digitalRead(TP_KILL_SWITCH) == LOW) {
                for (int i = 0; i < NUM_AXES; i++) {
                    steppers[i]->stop();
                    steppers[i]->disableOutputs();
                }
                file.close();
                return false;
            }
            // Abbruch durch Benutzer (rechter Taster)
            if (digitalRead(TP_JOYSTICK_BTN_RIGHT) == LOW) {
                file.close();
                for (int i = 0; i < NUM_AXES; i++) {
                    steppers[i]->disableOutputs();
                }
                while (digitalRead(TP_JOYSTICK_BTN_RIGHT) == LOW) {
                    if (digitalRead(TP_KILL_SWITCH) == LOW) break;
                    delay(10);
                }
                return false;
            }
        }
    }
    file.close();
    return true;
}
