#pragma once

#include <Arduino.h>
#include <math.h>

// Pose im kartesischen Raum (Position + ZYX-Orientierung)
struct CartesianPose {
    float x, y, z;            // Position (mm)
    float yaw, pitch, roll;   // Orientierung (Euler-Winkel Z-Y-X, in Rad)
};

// Gelenkwinkel-Container (6 DOF)
struct JointAngles {
    float angles[6];          // Gelenkwinkel in Radiant
};

// DH-Parameter für ein Gelenk (Denavit-Hartenberg)
struct DHParams {
    float a;      // Link-Length (Abstand entlang x-Achse)
    float alpha;  // Link-Twist (Winkel um x-Achse)
    float d;      // Link-Offset (Abstand entlang z-Achse)
    float theta;  // Joint-Angle-Offset (Konstante Verschiebung auf θ)
};

// Struktur der Roboter-Konfiguration (DH-Parameter, Gelenklimits, Tool-Offset)
struct RobotConfig {
    DHParams dhParams[6];   // DH-Parameter für Gelenke 0…5
    float    jointMin[6];   // minimale Gelenkwinkel (Rad)
    float    jointMax[6];   // maximale Gelenkwinkel (Rad)
    float    toolOffsetX;   // Werkzeug-Offset in X-Richtung (im Tool-KS)
    float    toolOffsetY;   // Werkzeug-Offset in Y-Richtung
    float    toolOffsetZ;   // Werkzeug-Offset in Z-Richtung
};

// Rohwerte des Beschleunigungssensors (zum Messen von Schwingungen)
struct AccelData {
    int16_t x;   // Rohbeschleunigung X-Achse
    int16_t y;   // Rohbeschleunigung Y-Achse
    int16_t z;   // Rohbeschleunigung Z-Achse
};

class RobotKinematics {
public:
    // Konstruktor – initialisiert mit gegebener Roboterkonfiguration
    RobotKinematics(const RobotConfig& config);

    // Getter/Setter für aktuelle Gelenkwinkel & Pose
    void          setCurrentJointAngles(const JointAngles& angles);
    JointAngles   getCurrentJointAngles() const;
    CartesianPose getCurrentPose() const;

    // Werkzeug-Offset (z.B. nach Werkzeugwechsel) ändern
    void setToolOffset(float x, float y, float z);

    // Vorwärts-Kinematik: berechnet Pose aus gegebenen Gelenkwinkeln
    CartesianPose forwardKinematics(const JointAngles& angles);

    // Inverse Kinematik: berechnet Gelenkwinkel für gegebene Ziel-Pose.
    // Rückgabe true bei Erfolg (outAngles gefüllt), false falls keine Lösung gefunden.
    bool inverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles);

    // Reichweiten-Check: prüft grob, ob eine Zielpose prinzipiell erreichbar ist
    bool isPoseReachable(const CartesianPose& pose);

    // Hilfsfunktion: Normalisiert Winkel auf [−π, +π]
    static float normalizeAngle(float angle);

    // Liest die aktuellen Beschleunigungswerte des ADXL345 am Tool
    AccelData getToolAcceleration();

private:
    // Hilfsfunktion: Berechnet die 4x4-DH-Transformationsmatrix für Gelenk i bei Winkel theta
    void computeDHMatrix(int i, float theta, float T[4][4]);

    RobotConfig  _config;         // Roboter-Konfiguration (DH-Parameter, Offsets, Limits)
    JointAngles  _currentAngles;  // aktuelle Gelenkwinkel
    CartesianPose _currentPose;   // aktuelle Pose des Tools (bezogen auf Basis)
};
