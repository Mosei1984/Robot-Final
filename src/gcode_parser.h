#pragma once
#include <Arduino.h>
#include <math.h>
#include "axis_config.h"
#include "joystick.h"

// Interne DH-Parameter des Roboters (für Kinematik-Berechnung)
const float DH_d1 = 50.0f;   // Basis-Höhe (Link-Offset)
const float DH_a2 = 80.0f;   // Länge des ersten Glieds (Schulter)
const float DH_a3 = 80.0f;   // Länge des zweiten Glieds (Ellbogen)
const float DH_d4 = 80.0f;   // Länge zum Handgelenk (Pitch) entlang Z vom Ellbogen
const float DH_d6 = 40.0f;   // Werkzeug-Offset in Z (Greifer-Länge)

// Aktuell angenommene Pose (Position und Orientierung) des Endeffektors
static float currentX = 0.0;
static float currentY = 0.0;
static float currentZ = DH_d1 + (DH_a2 + DH_a3) / 2.0; // mittlere Höhe
static float currentYaw = 0.0;
static float currentPitch = 0.0;
static float currentRoll = 0.0;

// Berechnet eine inverse Kinematik-Lösung (einfacher Ansatz) für die gegebene Pose (X, Y, Z, Yaw, Pitch, Roll).
// Gibt false zurück, wenn die Pose nicht erreichbar ist.
bool berechneInverseKinematik(float x, float y, float z, float yawDeg, float pitchDeg, float rollDeg, float outAngles[6]) {
    // Winkel in Radianten umrechnen
    float yaw = yawDeg * M_PI / 180.0;
    float pitch = pitchDeg * M_PI / 180.0;
    float roll = rollDeg * M_PI / 180.0;
    // Basiswinkel bestimmen
    float baseNeeded = atan2(y, x);
    // Prüfen, ob gewünschte Yaw zur Position passt (falls nicht ~gleich, Ziel evtl. unerreichbar)
    if (fabs(yaw - baseNeeded) > 0.1) {
        return false;
    }
    // Basisgelenk-Winkel setzen (Yaw)
    float theta0 = yaw;
    // Handgelenkszentrum (Wrist Center) berechnen, Tool-Offset abziehen
    float wx = x - DH_d6 * cos(pitch) * cos(yaw);
    float wy = y - DH_d6 * cos(pitch) * sin(yaw);
    float wz = z - DH_d6 * sin(pitch);
    // Planarer Abstand von der Basisachse
    float r = sqrt(wx * wx + wy * wy);
    // Vertikale Distanz von Basis-Schulter-Gelenk
    float z_eff = wz - DH_d1;
    // Reichweite prüfen
    float reach = sqrt(r * r + z_eff * z_eff);
    float maxReach = DH_a2 + DH_a3;
    if (reach > maxReach || reach < fabs(DH_a2 - DH_a3)) {
        return false;
    }
    // Ellbogenwinkel (Beta) berechnen
    float cosBeta = (r * r + z_eff * z_eff - DH_a2 * DH_a2 - DH_a3 * DH_a3) / (2 * DH_a2 * DH_a3);
    if (cosBeta < -1 || cosBeta > 1) {
        return false;
    }
    float Beta = acos(cosBeta);
    // Schulterwinkel (Alpha) berechnen
    float cosAlpha = (DH_a2 * DH_a2 + reach * reach - DH_a3 * DH_a3) / (2 * DH_a2 * reach);
    if (cosAlpha < -1 || cosAlpha > 1) {
        return false;
    }
    float Alpha = acos(cosAlpha);
    float gamma = atan2(z_eff, r);
    // Ellbogen unten-Lösung wählen (Schulter etwas angehoben)
    float theta1 = gamma + Alpha;
    // Ellbogen-Winkel (0 = gestreckt, positiv = nach unten gebeugt)
    float theta2 = Beta;
    // Handgelenk Pitch: Differenz aus gewünschtem Pitch und Armneigung
    float forearmAngle = theta1 - theta2;
    float theta3 = pitch - forearmAngle;
    // Handgelenk Roll: Vereinfachung - gesamte Rollrotation auf letztes Gelenk
    float theta4 = 0;
    float theta5 = roll;
    // Winkel in Grad umrechnen
    outAngles[0] = theta0 * 180.0 / M_PI;
    outAngles[1] = theta1 * 180.0 / M_PI;
    outAngles[2] = theta2 * 180.0 / M_PI;
    outAngles[3] = theta3 * 180.0 / M_PI;
    outAngles[4] = theta4 * 180.0 / M_PI;
    outAngles[5] = theta5 * 180.0 / M_PI;
    return true;
}

// Verarbeitet einen G1-GCode-Befehl mit X/Y/Z/A/B/C-Achsenwerten und bewegt den Roboter entsprechend.
// Gibt false zurück bei Abbruch (Not-Aus oder Benutzerabbruch), sonst true.
bool verarbeiteGCodeZeile(String zeile) {
    zeile.trim();
    if (zeile.length() == 0) return true;
    zeile.toUpperCase();
    if (!zeile.startsWith("G1")) {
        return true;  // nur G1 wird unterstützt
    }
    // Aktuelle Soll-Werte verwenden, falls Achse im Befehl fehlt
    float targetX = currentX;
    float targetY = currentY;
    float targetZ = currentZ;
    float targetYaw = currentYaw * 180.0 / M_PI;
    float targetPitch = currentPitch * 180.0 / M_PI;
    float targetRoll = currentRoll * 180.0 / M_PI;
    // X-Wert parsen
    int idx = zeile.indexOf('X');
    if (idx != -1) {
        int endIdx = zeile.indexOf(' ', idx);
        String val = (endIdx != -1) ? zeile.substring(idx + 1, endIdx) : zeile.substring(idx + 1);
        targetX = val.toFloat();
    }
    // Y-Wert parsen
    idx = zeile.indexOf('Y');
    if (idx != -1) {
        int endIdx = zeile.indexOf(' ', idx);
        String val = (endIdx != -1) ? zeile.substring(idx + 1, endIdx) : zeile.substring(idx + 1);
        targetY = val.toFloat();
    }
    // Z-Wert parsen
    idx = zeile.indexOf('Z');
    if (idx != -1) {
        int endIdx = zeile.indexOf(' ', idx);
        String val = (endIdx != -1) ? zeile.substring(idx + 1, endIdx) : zeile.substring(idx + 1);
        targetZ = val.toFloat();
    }
    // A (Yaw)
    idx = zeile.indexOf('A');
    if (idx != -1) {
        int endIdx = zeile.indexOf(' ', idx);
        String val = (endIdx != -1) ? zeile.substring(idx + 1, endIdx) : zeile.substring(idx + 1);
        targetYaw = val.toFloat();
    }
    // B (Pitch)
    idx = zeile.indexOf('B');
    if (idx != -1) {
        int endIdx = zeile.indexOf(' ', idx);
        String val = (endIdx != -1) ? zeile.substring(idx + 1, endIdx) : zeile.substring(idx + 1);
        targetPitch = val.toFloat();
    }
    // C (Roll)
    idx = zeile.indexOf('C');
    if (idx != -1) {
        int endIdx = zeile.indexOf(' ', idx);
        String val = (endIdx != -1) ? zeile.substring(idx + 1, endIdx) : zeile.substring(idx + 1);
        targetRoll = val.toFloat();
    }
    // Inverse Kinematik berechnen
    float jointAngles[6];
    if (!berechneInverseKinematik(targetX, targetY, targetZ, targetYaw, targetPitch, targetRoll, jointAngles)) {
        Serial.println(F("[GCode] Pose ungueltig oder nicht erreichbar."));
        return true;
    }
    // Gelenkwinkel -> Schrittziele berechnen und Bewegung ausführen
    extern AccelStepper* steppers[NUM_AXES];
    long targetSteps[NUM_AXES];
    for (int i = 0; i < NUM_AXES; i++) {
        targetSteps[i] = (long)(jointAngles[i] * stepsPerDeg[i]);
        steppers[i]->moveTo(targetSteps[i]);
    }
    bool allDone = false;
    while (!allDone) {
        allDone = true;
        for (int i = 0; i < NUM_AXES; i++) {
            if (steppers[i]->distanceToGo() != 0) {
                steppers[i]->run();
                allDone = false;
            }
        }
        if (digitalRead(TP_KILL_SWITCH) == LOW) {
            for (int i = 0; i < NUM_AXES; i++) {
                steppers[i]->stop();
                steppers[i]->disableOutputs();
            }
            return false;
        }
        if (digitalRead(TP_JOYSTICK_BTN_RIGHT) == LOW) {
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
    //  // Aktuelle Pose aktualisieren
    currentX = targetX;
    currentY = targetY;
    currentZ = targetZ;
    currentYaw = targetYaw * M_PI / 180.0;
    currentPitch = targetPitch * M_PI / 180.0;
    currentRoll = targetRoll * M_PI / 180.0;
    return true;
}
