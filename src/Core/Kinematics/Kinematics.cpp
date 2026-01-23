#include "Kinematics.h"
#include "utils/Logger.h"
#include <cmath>

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

bool Kinematics::forward(const std::array<float, 3>& angles, Vector3& position) {
  float angles_array[3] = {angles[0], angles[1], angles[2]};
  return forward(angles_array, position);
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
    result.error_message = String("IK failed: ") + solution.error_message;
  }

  return result;
}

Kinematics::Result Kinematics::inverseSafe(const Vector3& position) {
  Result result;

  // Проверка рабочего пространства
  if (!checkWorkspaceBounds(position)) {
    result.valid = false;
    result.error_code = 3001;
    result.error_message = "Point outside workspace";
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
      result.error_message = "Joint angles not safe";
    }

    // Проверка на коллизии
    if (!checkCollisions(position, result.joint_angles)) {
      result.valid = false;
      result.error_code = 3003;
      result.error_message = "Possible collision detected";
    }
  } else {
    result.error_code = solution.error_code;
    result.error_message = String("IK failed: ") + solution.error_message;
  }

  return result;
}

bool Kinematics::isReachable(const Vector3& position) const {
  return solver_.isReachable(position);
}

bool Kinematics::isSafe(const Vector3& position) const {
  // Проверка рабочего пространства
  if (!checkWorkspaceBounds(position)) {
    return false;
  }

  // Проверка через обратную кинематику
  Result result = const_cast<Kinematics*>(this)->inverseSafe(position);
  return result.valid;
}

bool Kinematics::areAnglesSafe(const std::array<float, 3>& angles) const {
  return checkJointLimits(angles);
}

bool Kinematics::velocityMapping(const Vector3& position,
                                 const Vector3& task_velocity,
                                 std::array<float, 3>& joint_velocities) {
  float jacobian[3][3];
  if (!solver_.computeJacobian(position, jacobian)) {
    return false;
  }

  float task_vel_array[3] = {task_velocity.x, task_velocity.y, task_velocity.z};
  float joint_vel_array[3];

  if (!solver_.taskToJointVelocity(position, task_velocity, joint_vel_array)) {
    return false;
  }

  joint_velocities[0] = joint_vel_array[0];
  joint_velocities[1] = joint_vel_array[1];
  joint_velocities[2] = joint_vel_array[2];

  return true;
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

  // Вычисление определителя матрицы 3x3
  float det = jacobian[0][0] * (jacobian[1][1] * jacobian[2][2] - jacobian[1][2] * jacobian[2][1])
              - jacobian[0][1] * (jacobian[1][0] * jacobian[2][2] - jacobian[1][2] * jacobian[2][0])
              + jacobian[0][2] * (jacobian[1][0] * jacobian[2][1] - jacobian[1][1] * jacobian[2][0]);

  return det;
}

bool Kinematics::isSingular(const Vector3& position, float threshold) {
  float det = getJacobianDeterminant(position);
  return fabs(det) < threshold;
}

const DeltaSolver::DeltaConfig& Kinematics::getConfig() const {
  return solver_.getConfig();
}

Vector3 Kinematics::getJointPositions(float angle, uint8_t joint_index) const {
  if (joint_index >= 3) {
    return Vector3(0, 0, 0);
  }

  return solver_.getUpperJointPosition(angle, joint_index);
}

float Kinematics::getJointDistance(const Vector3& position, uint8_t joint_index) const {
  if (joint_index >= 3) {
    return 0.0f;
  }

  auto config = solver_.getConfig();
  Vector3 effector_joint = solver_.getEffectorJointPosition(position, joint_index);
  Vector3 upper_joint = getJointPositions(0, joint_index); // Нулевой угол для упрощения

  return upper_joint.distanceTo(effector_joint);
}

bool Kinematics::checkWorkspaceBounds(const Vector3& position) const {
  return Limits::SafetyCheck::isWorkspacePointSafe(position.x, position.y, position.z);
}

bool Kinematics::checkJointLimits(const std::array<float, 3>& angles) const {
  return Limits::SafetyCheck::areJointAnglesSafe(angles.data());
}

bool Kinematics::checkCollisions(const Vector3& position,
                                 const std::array<float, 3>& angles) const {
  // Упрощенная проверка коллизий
  // В реальном проекте нужно реализовать полную проверку

  // Проверка минимальной высоты
  if (position.z > -50.0f) { // Минимум 50 мм от основания
    return false;
  }

  // Проверка расстояний между рычагами
  for (uint8_t i = 0; i < 3; i++) {
    float distance = getJointDistance(position, i);
    auto config = solver_.getConfig();

    // Проверка на полное вытягивание/складывание
    float min_distance = fabs(config.arm_length - config.forearm_length);
    float max_distance = config.arm_length + config.forearm_length;

    if (distance < min_distance + 5.0f || distance > max_distance - 5.0f) {
      return false;
    }
  }

  return true;
}
