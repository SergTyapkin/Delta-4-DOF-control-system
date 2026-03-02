#include <Arduino.h>
#include "Kinematics.h"
#include "../../../src/utils/Logger.h"
#include "../../../src/utils/MathUtils.h"
#include "../../../src/utils/Matrix.h"

Kinematics::Kinematics() {
  // Конструктор по умолчанию
}

void Kinematics::init(const Kinematics::DeltaConfig& config) {
  if (!config.isValid()) {
    Logger::error("Invalid Delta robot configuration");
    return;
  }

  config_ = config;

  Logger::info("Kinematics initialized:");
  Logger::info("  Arm length: %.1f mm", config_.arm_length);
  Logger::info("  Effector height: %.1f mm", config_.effector_height);
  Logger::info("  Columns joins positions and effector joints positions configured (values omitted)");
}


Kinematics::Solution Kinematics::inverseKinematics(const Vector6& position) {
  // Проверка рабочего пространства
  if (!checkWorkspaceBounds(position.toPosition())) {
    Solution solution;
    solution.valid = false;
    solution.error_code = 31;
    snprintf(solution.error_message, sizeof(solution.error_message), "Point position is outside of the workspace limits");
    return solution;
  }
  // Проверка углов наклона эффектора
  if (!checkEffectorAnglesBounds(position.toOrientation())) {
    Solution solution;
    solution.valid = false;
    solution.error_code = 32;
    snprintf(solution.error_message, sizeof(solution.error_message), "Target angles are out of limits");
    return solution;
  }

  Solution solution = solveInverseKinematics(position);

  // Проверка валидности решения
  if (!solution.valid) {
    return solution;
  }

  // Проверка на коллизии
  if (!checkCollisions(position, solution.joints_positions_z)) {
    solution.valid = false;
    solution.error_code = 33;
    snprintf(solution.error_message, sizeof(solution.error_message), "Possible collision detected");
    Logger::warning("Inverse kinematics solve unsafe: possible collision detected");
  }

  // Проверка на сингулярности
  SingularityType singularity = checkSingularity(position, solution.joints_positions_z);
  if (singularity != SING_NONE) {
    solution.valid = false;
    solution.error_code = 40 + singularity;

    Logger::warning("Inverse kinematics solve unsafe: singularity detected (type %d)", singularity);
  }

  // Проверка допустимости смещений по стойке
  for (uint8_t i = 0; i < RobotParams::MOTORS_COUNT; i++) {
    if (!Limits::JOINT_LIMITS[i].isPositionZValid(solution.joints_positions_z[i])) {
      solution.valid = false;
      solution.error_code = 32;
      snprintf(solution.error_message, sizeof(solution.error_message),
               "Distance %d out of limits: %.3f", i, solution.joints_positions_z[i]);

      Logger::warning("Inverse kinematics solve unsafe: joint %d position Z (%.2f mm) out of limits",
                      i, solution.joints_positions_z[i]);
      break;
    }
  }

  return solution;
}

bool Kinematics::forwardKinematics(const float joints_angles[RobotParams::MOTORS_COUNT], Vector6& position) {
  return solveForwardKinematics(joints_angles, position);
}

bool Kinematics::computeAnglesJacobian(const Vector6& position, float jacobian[RobotParams::MOTORS_COUNT][3]) {
  const float DELTA = 0.01f; // (mm)

  Solution base_solution = solveInverseKinematics(position);
  if (!base_solution.valid) {
    return false;
  }

  Vector6 variations[3] = {
      Vector6(DELTA, 0, 0),
      Vector6(0, DELTA, 0),
      Vector6(0, 0, DELTA)
  };

  for (uint8_t i = 0; i < 3; i++) {
    Vector6 varied_pos = position + variations[i];
    Solution varied_solution = solveInverseKinematics(varied_pos);

    if (!varied_solution.valid) {
      return false;
    }

    for (uint8_t j = 0; j < RobotParams::MOTORS_COUNT; j++) {
      jacobian[j][i] = (varied_solution.joints_angles[j] - base_solution.joints_angles[j]) / DELTA;
    }
  }

  return true;
}

// TODO: привести метод в порядок. Он не работает
bool Kinematics::inverseKinematicsVelocity(
    const Vector6& position,
    const Vector6& velocity,
    float joints_velocities[RobotParams::MOTORS_COUNT]
) {
  float jacobian[RobotParams::MOTORS_COUNT][3];
  if (!computeAnglesJacobian(position, jacobian)) {
    return false;
  }

  float J_pseudo[3][RobotParams::MOTORS_COUNT];
  if (!Matrix::pseudoInverseJacobian(jacobian, J_pseudo)) {
    return false;
  }

  // Compute: joint_velocity = J_pseudo^T * velocity
  float J_pseudo_transposed[RobotParams::MOTORS_COUNT][3];
  Matrix::transposeJacobian(J_pseudo, J_pseudo_transposed);
  Matrix::multiply(J_pseudo_transposed, velocity.toPosition(), joints_velocities);

  return true;
}

bool Kinematics::isSolutionPhysical(float joint_position_z) {
  return !(
      isnan(joint_position_z) ||
      isinf(joint_position_z) ||
      joint_position_z > 0
  );
}

const Kinematics::DeltaConfig& Kinematics::getConfig() const {
  return config_;
}

bool Kinematics::checkWorkspaceBounds(const Vector3& point) const {
  return Limits::SafetyCheck::isWorkspacePointSafe(point);
}
bool Kinematics::checkEffectorAnglesBounds(const Vector3& angles) const {
  return Limits::SafetyCheck::isEffectorAnglesSafe(angles);
}
