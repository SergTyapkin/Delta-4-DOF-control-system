#include <Arduino.h>
#include "Kinematics.h"
#include "../../../src/utils/Logger.h"

Kinematics::Kinematics() {
  // Конструктор по умолчанию
}

void Kinematics::init(const DeltaSolver::DeltaConfig& config) {
  solver_.init(config);
  Logger::info("Kinematics initialized");
}

bool Kinematics::forward(const float angles[3], Vector3& position) {
  return solver_.forwardKinematics(angles, position);
}

Kinematics::Result Kinematics::inverse(const Vector3& position) {
  Result result;
  DeltaSolver::Solution solution = solver_.inverseKinematics(position);

  result.valid = solution.valid;
  if (result.valid) {
    result.joint_angles[0] = solution.angles[0];
    result.joint_angles[1] = solution.angles[1];
    result.joint_angles[2] = solution.angles[2];
  } else {
    result.error_code = solution.error_code;
    snprintf(result.error_message, sizeof(result.error_message),
             "IK failed: %s", solution.error_message);
  }

  return result;
}

Kinematics::Result Kinematics::inverseSafe(const Vector3& position) {
  Result result;

  // Проверка рабочего пространства
  if (!checkWorkspaceBounds(position)) {
    result.valid = false;
    result.error_code = 3001;
    snprintf(result.error_message, sizeof(result.error_message),
             "Point outside workspace");
    return result;
  }

  DeltaSolver::Solution solution = solver_.inverseKinematicsSafe(position);

  result.valid = solution.valid;
  if (result.valid) {
    result.joint_angles[0] = solution.angles[0];
    result.joint_angles[1] = solution.angles[1];
    result.joint_angles[2] = solution.angles[2];

    // Дополнительная проверка безопасности
    if (!areAnglesSafe(result.joint_angles)) {
      result.valid = false;
      result.error_code = 3002;
      snprintf(result.error_message, sizeof(result.error_message),
               "Joint angles not safe");
    }

    // Проверка на коллизии
    if (!checkCollisions(position, result.joint_angles)) {
      result.valid = false;
      result.error_code = 3003;
      snprintf(result.error_message, sizeof(result.error_message),
               "Possible collision detected");
    }
  } else {
    result.error_code = solution.error_code;
    snprintf(result.error_message, sizeof(result.error_message),
             "IK failed: %s", solution.error_message);
  }

  return result;
}

bool Kinematics::isReachable(const Vector3& position) {
  return solver_.isReachable(position);
}

bool Kinematics::isSafe(const Vector3& position) const {
  if (!checkWorkspaceBounds(position)) {
    return false;
  }

  Result result = const_cast<Kinematics*>(this)->inverseSafe(position);
  return result.valid;
}

bool Kinematics::areAnglesSafe(const float angles[3]) const {
  return checkJointLimits(angles);
}

bool Kinematics::velocityMapping(const Vector3& position,
                                 const Vector3& task_velocity,
                                 float joint_velocities[3]) {
  float jacobian[3][3];
  if (!solver_.computeJacobian(position, jacobian)) {
    return false;
  }

  return solver_.taskToJointVelocity(position, task_velocity, joint_velocities);
}

void Kinematics::getWorkspaceBounds(float& min_radius, float& max_radius,
                                    float& min_z, float& max_z) const {
  solver_.getWorkspaceBounds(min_radius, max_radius, min_z, max_z);
}

bool Kinematics::getJacobian(const Vector3& position, float jacobian[3][3]) {
  return solver_.computeJacobian(position, jacobian);
}

float Kinematics::getJacobianDeterminant(const Vector3& position) {
  float jacobian[3][3];
  if (!getJacobian(position, jacobian)) {
    return 0.0f;
  }

  float det = jacobian[0][0] * (jacobian[1][1] * jacobian[2][2] - jacobian[1][2] * jacobian[2][1]) -
              jacobian[0][1] * (jacobian[1][0] * jacobian[2][2] - jacobian[1][2] * jacobian[2][0]) +
              jacobian[0][2] * (jacobian[1][0] * jacobian[2][1] - jacobian[1][1] * jacobian[2][0]);

  return det;
}

bool Kinematics::isSingular(const Vector3& position, float threshold) {
  float det = getJacobianDeterminant(position);
  return fabs(det) < threshold;
}

const DeltaSolver::DeltaConfig& Kinematics::getConfig() const {
  return solver_.getConfig();
}

bool Kinematics::checkWorkspaceBounds(const Vector3& position) const {
  return Limits::SafetyCheck::isWorkspacePointSafe(position.x, position.y, position.z);
}

bool Kinematics::checkJointLimits(const float angles[3]) const {
  return Limits::SafetyCheck::areJointAnglesSafe(angles);
}

bool Kinematics::checkCollisions(const Vector3& position,
                                 const float angles[3]) const {
  // Упрощенная проверка коллизий
  if (position.z > -50.0f) {
    return false;
  }

  const auto& config = solver_.getConfig();

  for (uint8_t i = 0; i < 3; i++) {
    Vector3 effector_joint = solver_.getEffectorJointPosition(position, i);
    Vector3 upper_joint = solver_.getUpperJointPosition(angles[i], i);

    float distance = upper_joint.distanceTo(effector_joint);
    float min_distance = fabs(config.arm_length - config.forearm_length);
    float max_distance = config.arm_length + config.forearm_length;

    if (distance < min_distance + 5.0f || distance > max_distance - 5.0f) {
      return false;
    }
  }

  return true;
}
