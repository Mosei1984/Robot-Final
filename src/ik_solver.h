#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;
#include <MatrixMath.h>
#include <Wire.h>

using namespace Eigen;

// Struktur für Zielpose
struct TargetPose {
    Vector3f position;     // Zielposition X, Y, Z
    Vector3f orientation;  // Roll, Pitch, Yaw
};

// Struktur für den aktuellen Modus (Position / Position + Orientierung)
enum class IKMode {
    POSITION_ONLY,
    POSITION_AND_ORIENTATION
};

class IKSolver {
public:
    IKSolver();
    void setDHParameters(const MatrixXf& dhTable);
    void setMode(IKMode mode);
    bool solveIK(const TargetPose& target, VectorXf& result);
    Matrix4f forwardKinematics(const VectorXf& jointAngles);
    void toggleMode();       // Zwischen den Modi wechseln
    IKMode getMode() const;  // Aktuellen Modus abrufen

private:
    MatrixXf dhTable;
    IKMode currentMode;
    bool computeJacobian(const VectorXf& jointAngles, MatrixXf& jacobian);
};

#endif
