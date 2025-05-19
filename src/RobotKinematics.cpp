#include "RobotKinematics.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Wire.h>

// I2C-Adresse des ADXL345 (alternativ 0x1D, abhängig von Verbindungs-Pin)
#define ADXL345_ADDR  0x53

// Registerdefinitionen ADXL345
#define REG_POWER_CTL  0x2D
#define REG_DATA_FORMAT 0x31
#define REG_BW_RATE    0x2C
#define REG_DATAX0     0x32  // ab DATAX0 folgen 6 Bytes für X0, X1, Y0, Y1, Z0, Z1

RobotKinematics::RobotKinematics(const RobotConfig& config) {
    _config = config;
    // Aktuelle Winkel initialisieren (Standard: alle 0)
    for (int i = 0; i < 6; ++i) {
        _currentAngles.angles[i] = 0.0f;
    }
    // Aktuelle Pose initial berechnen
    _currentPose = forwardKinematics(_currentAngles);

    // Debug-Testpunkte als Ausgänge konfigurieren
    pinMode(TESTPOINT_0, OUTPUT);
    pinMode(TESTPOINT_1, OUTPUT);
    // (weitere Testpunkte können analog konfiguriert werden)
    digitalWrite(TESTPOINT_0, LOW);
    digitalWrite(TESTPOINT_1, LOW);

    // I2C initialisieren (für ADXL345 und evtl. OLED bereits aktiv)
    Wire.begin();
    // ADXL345 in Messmodus versetzen
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(REG_POWER_CTL);
    Wire.write(0x08);              // Measure = 1 (Bit3), Wakeup = 0 (Bit1-0), Standby->Measurement
    Wire.endTransmission();
    // Messbereich ±16g, Vollauflösung
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(REG_DATA_FORMAT);
    Wire.write(0x0B);              // 0x0B: FULL_RES=1 (Bit3), Range=3 (Bits1-0 => ±16g), right-justify
    Wire.endTransmission();
    // Datenrate auf 3200 Hz setzen (hohe Bandbreite für Vibrationen)
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(REG_BW_RATE);
    Wire.write(0x0F);              // 0x0F: ODR=3200Hz (LOW_POWER=0, Rate code 0xF)
    Wire.endTransmission();
}

void RobotKinematics::setCurrentJointAngles(const JointAngles& angles) {
    // Aktuelle Gelenkwinkel setzen und Pose neu berechnen
    _currentAngles = angles;
    _currentPose = forwardKinematics(_currentAngles);
}

JointAngles RobotKinematics::getCurrentJointAngles() const {
    return _currentAngles;
}

CartesianPose RobotKinematics::getCurrentPose() const {
    return _currentPose;
}

void RobotKinematics::setToolOffset(float x, float y, float z) {
    _config.toolOffsetX = x;
    _config.toolOffsetY = y;
    _config.toolOffsetZ = z;
    // Bei geänderten Offsets aktuelle Pose neu berechnen, da Position sich verschiebt
    _currentPose = forwardKinematics(_currentAngles);
}

CartesianPose RobotKinematics::forwardKinematics(const JointAngles& angles) {
    // Debug Testpunkt 1: Start Forward-Kinematics
    digitalWrite(TESTPOINT_1, HIGH);

    // Verwende Eigen für Matrix-Berechnungen
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();  // Transformationsmatrix Basis->Tool
    // Transformation bis zum Endeffektor (Frame 5, ohne Tool-Offset)
    for (int i = 0; i < 6; ++i) {
        // DH-Parameter aus Config
        float a = _config.dhParams[i].a;
        float d = _config.dhParams[i].d;
        float alpha = _config.dhParams[i].alpha;
        float theta_offset = _config.dhParams[i].theta;
        // Gelenkwinkel + Offset
        float theta = angles.angles[i] + theta_offset;
        float cth = cos(theta);
        float sth = sin(theta);
        float ca = cos(alpha);
        float sa = sin(alpha);
        // Denavit-Hartenberg Transformationsmatrix für dieses Gelenk
        Eigen::Matrix4f A;
        A << cth, -sth * ca,  sth * sa,  a * cth,
             sth,  cth * ca, -cth * sa,  a * sth,
             0.0,       sa,       ca,       d,
             0.0,      0.0,      0.0,     1.0;
        // kumulative Transformation multiplizieren
        T = T * A;
    }
    // Am Ende von T: Basis -> letztes Gelenk (Greifer-Flansch)
    // Tool-Offset in Weltkoordinaten hinzufügen
    Eigen::Vector3f toolOffsetVec(_config.toolOffsetX, _config.toolOffsetY, _config.toolOffsetZ);
    Eigen::Vector3f pos_flange = T.block<3,1>(0,3);         // Translationsteil (x,y,z) aus T
    Eigen::Matrix3f R_flange = T.block<3,3>(0,0);           // Rotationsmatrix Teil aus T
    Eigen::Vector3f pos_tool = pos_flange + R_flange * toolOffsetVec;  // Tool-Spitze Position

    // Euler-Winkel (ZYX) aus der Gesamt-Rotationsmatrix (R_total = R_flange, da Tool-Offset reiner Translationsoffset)
    // R_flange entspricht der Orientierung des Tools (letztes Gelenk definiert Ausrichtung der Toolachse).
    float yaw   = atan2(R_flange(1,0), R_flange(0,0));
    float pitch = atan2(-R_flange(2,0), sqrt(pow(R_flange(2,1),2) + pow(R_flange(2,2),2)));
    float roll  = atan2(R_flange(2,1), R_flange(2,2));

    // Ergebnis in CartesianPose-Struktur verpacken
    CartesianPose result;
    result.x = pos_tool(0);
    result.y = pos_tool(1);
    result.z = pos_tool(2);
    result.yaw = yaw;
    result.pitch = pitch;
    result.roll = roll;

    // Debug Testpunkt 1: Ende Forward-Kinematics
    digitalWrite(TESTPOINT_1, LOW);
    return result;
}

bool RobotKinematics::inverseKinematics(const CartesianPose& targetPose, JointAngles& outAngles) {
    // Debug Testpunkt 0: Start Inverse-Kinematics
    digitalWrite(TESTPOINT_0, HIGH);

    // Start mit aktuellem Winkelzustand (Start der Iteration)
    Eigen::Matrix<float, 6, 1> theta; 
    for (int i = 0; i < 6; ++i) {
        theta(i) = _currentAngles.angles[i];
    }

    // Festlegen der Soll-Orientierung je nach Modus:
    // Hinweis: Hier wird angenommen, dass bei Modus1 targetPose bereits mit fixierter Orientierung übergeben wird (z.B. roll=0,pitch=0,yaw entsprechend Ausrichtung).
    // Bei Modus2 enthält targetPose die gewünschte Orientierung vom Nutzer.
    // (Eine externe Logik muss dafür sorgen, dass im Modus1 targetPose.pitch=0, targetPose.roll=0 gehalten werden.)

    const int MAX_ITER = 100;
    const float POS_EPS = 0.5f;        // Toleranz Position (mm)
    const float ORI_EPS = 0.01f;       // Toleranz Orientierung (rad ~0,57°)
    const float LAMBDA = 0.5f;         // Dämpfungsfaktor für Konvergenz

    bool success = false;
    for (int iter = 0; iter < MAX_ITER; ++iter) {
        // Berechne Vorwärtskinematik für aktuellen Schätzwert theta
        JointAngles guessAngles;
        for (int j = 0; j < 6; ++j) guessAngles.angles[j] = theta(j);
        CartesianPose currentPose = forwardKinematics(guessAngles);

        // Fehlervektoren (Position und Orientierung)
        Eigen::Vector3f pos_err;
        pos_err << targetPose.x - currentPose.x,
                   targetPose.y - currentPose.y,
                   targetPose.z - currentPose.z;
        // Orientierungsfehler als Rotations-Vektor (Achse*Winkel)
        // Berechne Rotationsmatrix für aktuelle Pose und Zielpose
        float cy = cos(targetPose.yaw),   sy = sin(targetPose.yaw);
        float cp = cos(targetPose.pitch), sp = sin(targetPose.pitch);
        float cr = cos(targetPose.roll),  sr = sin(targetPose.roll);
        // Ziel-Rotationsmatrix (ZYX aus yaw,pitch,roll)
        Eigen::Matrix3f R_target;
        R_target << cy*cp,    cy*sp*sr - sy*cr,    cy*sp*cr + sy*sr,
                    sy*cp,    sy*sp*sr + cy*cr,    sy*sp*cr - cy*sr,
                    -sp,      cp*sr,               cp*cr;
        // Aktuelle Rotationsmatrix aus Pose (ZYX berechnet, nun invertieren um Matrix zu erhalten)
        float cy_c = cos(currentPose.yaw),   sy_c = sin(currentPose.yaw);
        float cp_c = cos(currentPose.pitch), sp_c = sin(currentPose.pitch);
        float cr_c = cos(currentPose.roll),  sr_c = sin(currentPose.roll);
        Eigen::Matrix3f R_current;
        R_current << cy_c*cp_c,    cy_c*sp_c*sr_c - sy_c*cr_c,    cy_c*sp_c*cr_c + sy_c*sr_c,
                     sy_c*cp_c,    sy_c*sp_c*sr_c + cy_c*cr_c,    sy_c*sp_c*cr_c - cy_c*sr_c,
                     -sp_c,        cp_c*sr_c,                    cp_c*cr_c;
        // Rotationsfehler R_err = R_target * R_current^T
        Eigen::Matrix3f R_err = R_target * R_current.transpose();
        // Achse-Angle aus R_err extrahieren
        float angle_err = acosf(fmin(1.0f, (R_err.trace() - 1.0f) / 2.0f));  // Winkel
        Eigen::Vector3f axis_err;
        if (angle_err < 1e-6f) {
            axis_err << 0, 0, 0;
        } else {
            axis_err(0) = R_err(2,1) - R_err(1,2);
            axis_err(1) = R_err(0,2) - R_err(2,0);
            axis_err(2) = R_err(1,0) - R_err(0,1);
            axis_err /= (2.0f * sin(angle_err));
        }
        Eigen::Vector3f ori_err = angle_err * axis_err;  // Orientierungsfehler-Vektor (im Welt-KS)

        // Prüfen, ob Fehler klein genug für Abbruch
        if (pos_err.norm() < POS_EPS && ori_err.norm() < ORI_EPS) {
            success = true;
            break;
        }

        // Jacobi-Matrix J (6x6)
        Eigen::Matrix<float, 6, 6> J;
        // Für Jacobi: Zwischenpositionen und Achsrichtungen berechnen
        // Transformations-Matrix schrittweise aufbauen
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        // Basis-Parameter
        Eigen::Vector3f origin_prev = Eigen::Vector3f(0,0,0);    // O_{-1}: Basisursprung
        Eigen::Vector3f z_prev = Eigen::Vector3f(0,0,1);         // z_{-1}: Basis z-Achse (angenommen vertikal)
        for (int j = 0; j < 6; ++j) {
            // Achse j: z_prev (Ausrichtung der (j)-ten Joint-Achse in Weltkoordinaten)
            // Ursprungsvektor von Gelenk j: origin_prev
            // Position des Tool relativ zu origin_prev:
            Eigen::Vector3f origin_tool;
            origin_tool << currentPose.x, currentPose.y, currentPose.z;
            // Linearer Anteil: z_prev x (P_tool - O_prev)
            Eigen::Vector3f lin = z_prev.cross(origin_tool - origin_prev);
            // Rotationsanteil: z-Achse
            Eigen::Vector3f rot = z_prev;
            // Spalte j in J einsetzen
            J(0, j) = lin(0);
            J(1, j) = lin(1);
            J(2, j) = lin(2);
            J(3, j) = rot(0);
            J(4, j) = rot(1);
            J(5, j) = rot(2);
            // Als Nächstes Transform bis Frame j berechnen (inklusive der Rotation dieses Gelenks)
            float a = _config.dhParams[j].a;
            float d = _config.dhParams[j].d;
            float alpha = _config.dhParams[j].alpha;
            float theta_offset = _config.dhParams[j].theta;
            float theta_j = theta(j) + theta_offset;
            float cth = cos(theta_j);
            float sth = sin(theta_j);
            float ca = cos(alpha);
            float sa = sin(alpha);
            // DH-Teiltransformationsmatrix A_j
            Eigen::Matrix4f A;
            A << cth, -sth * ca,  sth * sa,  a * cth,
                 sth,  cth * ca, -cth * sa,  a * sth,
                 0.0,       sa,       ca,       d,
                 0.0,      0.0,      0.0,     1.0;
            T = T * A;
            // Aktualisiere origin_prev und z_prev für nächstes Gelenk
            origin_prev = T.block<3,1>(0,3);
            Eigen::Matrix3f R_prev = T.block<3,3>(0,0);
            z_prev = R_prev.col(2);
        }

        // Jacobi-Pseudoinverse berechnen via SVD
        Eigen::JacobiSVD<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> svd(
            J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const float tolerance = 1e-6f;
        int cols = J.cols();
        int rows = J.rows();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> SigmaInv(cols, rows);
        SigmaInv.setZero();
        // Invertiere nicht-Null Singularwerte
        for (int k = 0; k < svd.singularValues().size(); ++k) {
            float sigma = svd.singularValues()(k);
            if (sigma > tolerance) {
                SigmaInv(k, k) = 1.0f / sigma;
            } else {
                SigmaInv(k, k) = 0.0f;
            }
        }
        // Pseudoinverse J^+ = V * SigmaInv * U^T
        Eigen::Matrix<float, 6, 6> J_pinv = svd.matrixV() * SigmaInv * svd.matrixU().transpose();

        // Gelenkwinkel-Inkrement Δθ = λ * J^+ * [pos_err; ori_err]
        Eigen::Matrix<float, 6, 1> err;
        err << pos_err(0), pos_err(1), pos_err(2), ori_err(0), ori_err(1), ori_err(2);
        Eigen::Matrix<float, 6, 1> delta_theta = LAMBDA * (J_pinv * err);

        // Aktualisiere Schätzung der Gelenkwinkel
        theta += delta_theta;

        // Gelenkwinkel Normalisieren und ggf. in sichere Bereiche bringen
        for (int j = 0; j < 6; ++j) {
            // Begrenzung auf gültige Min/Max-Werte
            theta(j) = normalizeAngle(theta(j));
            if (theta(j) < _config.jointMin[j] - 1e-3f || theta(j) > _config.jointMax[j] + 1e-3f) {
                // Winkel außerhalb zulässigem Bereich -> Abbruch
                success = false;
                iter = MAX_ITER; // Schleife beenden
                break;
            }
        }
    }

    if (success) {
        // Ergebnis in outAngles übertragen
        for (int i = 0; i < 6; ++i) {
            outAngles.angles[i] = theta(i);
        }
        // (Hinweis: _currentAngles/_currentPose wird **nicht** hier gesetzt, 
        // da die Aufrufer-Funktion nach erfolgreicher IK z.B. setCurrentJointAngles nutzen kann,
        // um auch Motor-Sollwerte zu setzen und Pose zu aktualisieren.)
    }

    // Debug Testpunkt 0: Ende Inverse-Kinematics
    digitalWrite(TESTPOINT_0, LOW);
    return success;
}

bool RobotKinematics::isPoseReachable(const CartesianPose& pose) {
    // Einfache Reichweitenprüfung: Abstand vom Robotersockel
    // Berücksichtige Basis-Höhe:
    float baseHeight = _config.dhParams[0].d;
    float dx = pose.x;
    float dy = pose.y;
    float dz = pose.z - baseHeight;
    // Abstand vom Basisgelenk zum Zielpunkt
    float dist = sqrt(dx*dx + dy*dy + dz*dz);
    // Maximale Reichweite (angenommen Hauptarme 1 und 2 gestreckt)
    float a1 = _config.dhParams[1].a;
    float a2 = _config.dhParams[2].a;
    float maxReach = a1 + a2;
    if (dist > maxReach + 1e-6f) {
        return false; // zu weit weg
    }
    if (pose.z < 0.0f) {
        return false; // unter Grundniveau (Arm kann nicht unter Basis)
    }
    // (Optional könnten weitere Kriterien geprüft werden, z.B. min. Abstand, Gelenkwinkelgrenzen etc.)
    return true;
}

float RobotKinematics::normalizeAngle(float angle) {
    // Winkel auf [-π, π] begrenzen
    const float PI_F = 3.14159265358979f;
    while (angle > PI_F)  angle -= 2*PI_F;
    while (angle < -PI_F) angle += 2*PI_F;
    return angle;
}

AccelData RobotKinematics::getToolAcceleration() {
    AccelData data;
    uint8_t raw[6];
    // Lese 6 Bytes ab DATAX0
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(REG_DATAX0);
    Wire.endTransmission(false);  // Restart
    Wire.requestFrom(ADXL345_ADDR, (uint8_t)6);
    for (int i = 0; i < 6; ++i) {
        if (Wire.available()) {
            raw[i] = Wire.read();
        } else {
            raw[i] = 0;
        }
    }
    // Zusammenfügen der 16-bit Werte (Little-Endian)
    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);
    data.x = x;
    data.y = y;
    data.z = z;
    return data;
}
