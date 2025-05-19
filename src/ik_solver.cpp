#include "ik_solver.h"

IKSolver::IKSolver() : currentMode(IKMode::POSITION_ONLY) {
    dhTable = MatrixXf::Zero(6, 4);
}

void IKSolver::setDHParameters(const MatrixXf& table) {
    dhTable = table;
}

void IKSolver::setMode(IKMode mode) {
    currentMode = mode;
}

IKMode IKSolver::getMode() const {
    return currentMode;
}

void IKSolver::toggleMode() {
    currentMode = (currentMode == IKMode::POSITION_ONLY) ?
                  IKMode::POSITION_AND_ORIENTATION :
                  IKMode::POSITION_ONLY;
}

// Forward-Kinematik mit Transformationen
Matrix4f IKSolver::forwardKinematics(const VectorXf& jointAngles) {
    Matrix4f T = Matrix4f::Identity();
    for (int i = 0; i < 6; ++i) {
        float theta = jointAngles[i];
        float d = dhTable(i, 0);
        float r = dhTable(i, 1);
        float alpha = dhTable(i, 2);

        Matrix4f A;
        A << cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), r*cos(theta),
             sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), r*sin(theta),
             0,          sin(alpha),             cos(alpha),             d,
             0,          0,                      0,                      1;
        T *= A;
    }
    return T;
}

// Inverse-Kinematik (vereinfacht)
bool IKSolver::solveIK(const TargetPose& target, VectorXf& result) {
    const float epsilon = 0.001;
    const int maxIterations = 100;
    result = VectorXf::Zero(6);
    for (int iter = 0; iter < maxIterations; ++iter) {
        Matrix4f currentFK = forwardKinematics(result);
        Vector3f currentPos = currentFK.block<3, 1>(0, 3);

        Vector3f posError = target.position - currentPos;

        // Optional: Orientierung als Fehler berechnen
        Vector3f orientError = Vector3f::Zero();
        if (currentMode == IKMode::POSITION_AND_ORIENTATION) {
            // Nur Roll, Pitch, Yaw Zielorientierung später implementieren
        }

        VectorXf error(3);
        error << posError;

        if (error.norm() < epsilon) {
            return true;
        }

        MatrixXf J;
        if (!computeJacobian(result, J)) {
            return false;
        }

        MatrixXf J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        result += J_pinv * error;
    }
    return false;
}

// Numerisch berechnete Jacobi-Matrix
bool IKSolver::computeJacobian(const VectorXf& jointAngles, MatrixXf& jacobian) {
    const float delta = 0.01;
    jacobian = MatrixXf(3, 6);
    for (int i = 0; i < 6; ++i) {
        VectorXf plus = jointAngles;
        VectorXf minus = jointAngles;
        plus[i] += delta;
        minus[i] -= delta;

        Vector3f pPlus = forwardKinematics(plus).block<3, 1>(0, 3);
        Vector3f pMinus = forwardKinematics(minus).block<3, 1>(0, 3);
        jacobian.block<3, 1>(0, i) = (pPlus - pMinus) / (2.0f * delta);
    }
    return true;
}
