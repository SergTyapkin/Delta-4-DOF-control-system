#include <Arduino.h>
#include "DeltaSolver.h"
#include "../../../src/utils/Logger.h"
#include "../../../src/utils/MathUtils.h"

DeltaSolver::DeltaSolver() {
  init(DeltaConfig());
}

void DeltaSolver::init(const DeltaConfig& config) {
  if (!config.isValid()) {
    Logger::error("Invalid Delta robot configuration");
    return;
  }

  config_ = config;
  precomputeConstants();

  Logger::info("DeltaSolver initialized:");
  Logger::info("  Base radius: %.1f mm", config_.base_radius);
  Logger::info("  Effector radius: %.1f mm", config_.effector_radius);
  Logger::info("  Arm length: %.1f mm", config_.arm_length);
  Logger::info("  Forearm length: %.1f mm", config_.forearm_length);
}

void DeltaSolver::precomputeConstants() {
  forearm_squared_ = config_.forearm_length * config_.forearm_length;

  for (int i = 0; i < 3; i++) {
    cos_base_angles_[i] = cosf(config_.base_angles[i]);
    sin_base_angles_[i] = sinf(config_.base_angles[i]);
  }
}

bool DeltaSolver::forwardKinematics(const float angles[3], Vector3& position) {
  Vector3 guess(0, 0, (Limits::WORKSPACE.min_z + Limits::WORKSPACE.max_z) / 2);

  const uint8_t MAX_ITERATIONS = 50;
  const float TOLERANCE = 0.001f;

  for (uint8_t iter = 0; iter < MAX_ITERATIONS; iter++) {
    Solution solution = inverseKinematics(guess);

    if (!solution.valid) {
      Logger::warning("Forward kinematics: invalid solution at iteration %d", iter);
      return false;
    }

    float error[3];
    float max_error = 0;
    for (int i = 0; i < 3; i++) {
      error[i] = angles[i] - solution.angles[i];
      if (fabs(error[i]) > max_error) {
        max_error = fabs(error[i]);
      }
    }

    if (max_error < TOLERANCE) {
      position = guess;
      return true;
    }

    float jacobian[3][3];
    if (!computeJacobian(guess, jacobian)) {
      Logger::warning("Forward kinematics: failed to compute Jacobian");
      return false;
    }

    float jacobian_inv[3][3];

    float det = jacobian[0][0] * (jacobian[1][1] * jacobian[2][2] - jacobian[1][2] * jacobian[2][1])
                - jacobian[0][1] * (jacobian[1][0] * jacobian[2][2] - jacobian[1][2] * jacobian[2][0])
                + jacobian[0][2] * (jacobian[1][0] * jacobian[2][1] - jacobian[1][1] * jacobian[2][0]);

    if (fabs(det) < 0.0001f) {
      Logger::warning("Forward kinematics: singular Jacobian matrix");
      return false;
    }

    float inv_det = 1.0f / det;
    jacobian_inv[0][0] = (jacobian[1][1] * jacobian[2][2] - jacobian[2][1] * jacobian[1][2]) * inv_det;
    jacobian_inv[0][1] = (jacobian[0][2] * jacobian[2][1] - jacobian[0][1] * jacobian[2][2]) * inv_det;
    jacobian_inv[0][2] = (jacobian[0][1] * jacobian[1][2] - jacobian[0][2] * jacobian[1][1]) * inv_det;
    jacobian_inv[1][0] = (jacobian[1][2] * jacobian[2][0] - jacobian[1][0] * jacobian[2][2]) * inv_det;
    jacobian_inv[1][1] = (jacobian[0][0] * jacobian[2][2] - jacobian[0][2] * jacobian[2][0]) * inv_det;
    jacobian_inv[1][2] = (jacobian[0][2] * jacobian[1][0] - jacobian[0][0] * jacobian[1][2]) * inv_det;
    jacobian_inv[2][0] = (jacobian[1][0] * jacobian[2][1] - jacobian[1][1] * jacobian[2][0]) * inv_det;
    jacobian_inv[2][1] = (jacobian[0][1] * jacobian[2][0] - jacobian[0][0] * jacobian[2][1]) * inv_det;
    jacobian_inv[2][2] = (jacobian[0][0] * jacobian[1][1] - jacobian[0][1] * jacobian[1][0]) * inv_det;

    float delta_pos[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        delta_pos[i] += jacobian_inv[i][j] * error[j];
      }
    }

    guess.x += delta_pos[0] * 0.5f;
    guess.y += delta_pos[1] * 0.5f;
    guess.z += delta_pos[2] * 0.5f;
  }

  Logger::warning("Forward kinematics: did not converge");
  return false;
}

DeltaSolver::Solution DeltaSolver::inverseKinematics(const Vector3& position) {
  Solution solution;

  if (!isReachable(position)) {
    solution.valid = false;
    solution.error_code = 1;
    snprintf(solution.error_message, sizeof(solution.error_message), "Point not reachable");
    return solution;
  }

  for (int i = 0; i < 3; i++) {
    solution.angles[i] = solveForArm(position, i);

    if (!isSolutionPhysical(solution.angles[i], i)) {
      solution.valid = false;
      solution.error_code = 2;
      snprintf(solution.error_message, sizeof(solution.error_message),
               "Non-physical solution for arm %d: %.3f", i, solution.angles[i]);
      return solution;
    }
  }

  solution.valid = true;
  solution.error_message[0] = '\0';
  return solution;
}

float DeltaSolver::solveForArm(const Vector3& position, int arm_index) {
  Vector3 effector_joint = getEffectorJointPosition(position, arm_index);

  float base_joint_x = config_.base_radius * cos_base_angles_[arm_index];
  float base_joint_y = config_.base_radius * sin_base_angles_[arm_index];
  float base_joint_z = 0;

  float dx = effector_joint.x - base_joint_x;
  float dy = effector_joint.y - base_joint_y;
  float dz = effector_joint.z - base_joint_z;

  float L = sqrtf(dx * dx + dy * dy);
  float M = L - config_.effector_radius;
  float N = dz;
  float D = sqrtf(M * M + N * N);

  if (D > config_.arm_length + config_.forearm_length ||
      D < fabs(config_.arm_length - config_.forearm_length)) {
    return NAN;
  }

  float numerator = config_.arm_length * config_.arm_length + D * D - config_.forearm_length * config_.forearm_length;
  float denominator = 2.0f * config_.arm_length * D;

  if (denominator < 0.0001f) {
    return NAN;
  }

  float cos_theta = numerator / denominator;

  if (cos_theta > 1.0f) cos_theta = 1.0f;
  if (cos_theta < -1.0f) cos_theta = -1.0f;

  float theta = acosf(cos_theta);
  float phi = atan2f(N, M);
  float angle = theta + phi;

  return angle;
}

DeltaSolver::Solution DeltaSolver::inverseKinematicsSafe(const Vector3& position) {
  Solution solution = inverseKinematics(position);

  if (!solution.valid) {
    return solution;
  }

  for (int i = 0; i < 3; i++) {
    if (!Limits::JOINT_LIMITS[i].isAngleValid(solution.angles[i])) {
      solution.valid = false;
      solution.error_code = 3;
      snprintf(solution.error_message, sizeof(solution.error_message),
               "Angle %d out of limits: %.3f", i, solution.angles[i]);

      Logger::warning("IK unsafe: angle %d (%.2f deg) out of limits",
                      i, solution.angles[i] * MathUtils::RAD_TO_DEG);
      break;
    }
  }

  SingularityType singularity = checkSingularity(position, solution.angles);
  if (singularity != SING_NONE) {
    solution.valid = false;
    solution.error_code = 4 + singularity;

    Logger::warning("IK unsafe: singularity detected (type %d)", singularity);
  }

  return solution;
}

bool DeltaSolver::isReachable(const Vector3& position) {
  float radius = sqrtf(position.x * position.x + position.y * position.y);

  if (radius > Limits::WORKSPACE.max_x) {
    return false;
  }

  if (position.z < Limits::WORKSPACE.min_z || position.z > Limits::WORKSPACE.max_z) {
    return false;
  }

  Solution test_solution = inverseKinematics(position);
  return test_solution.valid;
}

DeltaSolver::SingularityType DeltaSolver::checkSingularity(const Vector3& position,
                                                           const float angles[3]) {
  for (int i = 0; i < 3; i++) {
    Vector3 upper_joint = getUpperJointPosition(angles[i], i);
    Vector3 effector_joint = getEffectorJointPosition(position, i);

    float distance = upper_joint.distanceTo(effector_joint);

    if (fabs(distance - (config_.arm_length + config_.forearm_length)) < 1.0f) {
      return SING_EXTENDED;
    }

    if (fabs(distance - fabs(config_.arm_length - config_.forearm_length)) < 1.0f) {
      return SING_RETRACTED;
    }
  }

  return SING_NONE;
}

bool DeltaSolver::computeJacobian(const Vector3& position, float jacobian[3][3]) {
  const float DELTA = 0.01f;

  Solution base_solution = inverseKinematics(position);
  if (!base_solution.valid) {
    return false;
  }

  Vector3 variations[3] = {
      Vector3(DELTA, 0, 0),
      Vector3(0, DELTA, 0),
      Vector3(0, 0, DELTA)
  };

  for (int i = 0; i < 3; i++) {
    Vector3 varied_pos = position + variations[i];
    Solution varied_solution = inverseKinematics(varied_pos);

    if (!varied_solution.valid) {
      return false;
    }

    for (int j = 0; j < 3; j++) {
      jacobian[j][i] = (varied_solution.angles[j] - base_solution.angles[j]) / DELTA;
    }
  }

  return true;
}

bool DeltaSolver::taskToJointVelocity(const Vector3& position,
                                      const Vector3& task_velocity,
                                      float joint_velocity[3]) {
  float jacobian[3][3];

  if (!computeJacobian(position, jacobian)) {
    return false;
  }

  float JJT[3][3] = {{0}};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        JJT[i][j] += jacobian[i][k] * jacobian[j][k];
      }
    }
  }

  float det = JJT[0][0] * (JJT[1][1] * JJT[2][2] - JJT[1][2] * JJT[2][1])
              - JJT[0][1] * (JJT[1][0] * JJT[2][2] - JJT[1][2] * JJT[2][0])
              + JJT[0][2] * (JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]);

  if (fabs(det) < 0.0001f) {
    return false;
  }

  float inv_JJT[3][3];
  float inv_det = 1.0f / det;

  inv_JJT[0][0] = (JJT[1][1] * JJT[2][2] - JJT[2][1] * JJT[1][2]) * inv_det;
  inv_JJT[0][1] = (JJT[0][2] * JJT[2][1] - JJT[0][1] * JJT[2][2]) * inv_det;
  inv_JJT[0][2] = (JJT[0][1] * JJT[1][2] - JJT[0][2] * JJT[1][1]) * inv_det;
  inv_JJT[1][0] = (JJT[1][2] * JJT[2][0] - JJT[1][0] * JJT[2][2]) * inv_det;
  inv_JJT[1][1] = (JJT[0][0] * JJT[2][2] - JJT[0][2] * JJT[2][0]) * inv_det;
  inv_JJT[1][2] = (JJT[0][2] * JJT[1][0] - JJT[0][0] * JJT[1][2]) * inv_det;
  inv_JJT[2][0] = (JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]) * inv_det;
  inv_JJT[2][1] = (JJT[0][1] * JJT[2][0] - JJT[0][0] * JJT[2][1]) * inv_det;
  inv_JJT[2][2] = (JJT[0][0] * JJT[1][1] - JJT[0][1] * JJT[1][0]) * inv_det;

  float J_pseudo[3][3] = {{0}};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        J_pseudo[i][j] += jacobian[k][i] * inv_JJT[k][j];
      }
    }
  }

  float tv[3] = {task_velocity.x, task_velocity.y, task_velocity.z};
  for (int i = 0; i < 3; i++) {
    joint_velocity[i] = 0;
    for (int j = 0; j < 3; j++) {
      joint_velocity[i] += J_pseudo[i][j] * tv[j];
    }
  }

  return true;
}

void DeltaSolver::getWorkspaceBounds(float& min_radius, float& max_radius,
                                     float& min_z, float& max_z) const {
  min_radius = 0;
  max_radius = Limits::WORKSPACE.max_x;
  min_z = Limits::WORKSPACE.min_z;
  max_z = Limits::WORKSPACE.max_z;
}

Vector3 DeltaSolver::getUpperJointPosition(float angle, int arm_index) const {
  Vector3 position;

  position.x = config_.base_radius * cos_base_angles_[arm_index]
               + config_.arm_length * cosf(angle) * cos_base_angles_[arm_index];
  position.y = config_.base_radius * sin_base_angles_[arm_index]
               + config_.arm_length * cosf(angle) * sin_base_angles_[arm_index];
  position.z = config_.arm_length * sinf(angle);

  return position;
}

Vector3 DeltaSolver::getEffectorJointPosition(const Vector3& effector_pos,
                                              int arm_index) const {
  Vector3 position;

  position.x = effector_pos.x + config_.effector_radius * cos_base_angles_[arm_index];
  position.y = effector_pos.y + config_.effector_radius * sin_base_angles_[arm_index];
  position.z = effector_pos.z;

  return position;
}

float DeltaSolver::calculateJointDistance(const Vector3& upper_joint,
                                          const Vector3& effector_joint) const {
  float dx = effector_joint.x - upper_joint.x;
  float dy = effector_joint.y - upper_joint.y;
  float dz = effector_joint.z - upper_joint.z;

  return sqrtf(dx * dx + dy * dy + dz * dz);
}

bool DeltaSolver::isSolutionPhysical(float angle, int arm_index) {
  if (isnan(angle)) {
    return false;
  }

  if (isinf(angle)) {
    return false;
  }

  if (angle < -MathUtils::PI || angle > MathUtils::PI) {
    return false;
  }

  return true;
}

bool DeltaSolver::solveQuadraticEquation(float a, float b, float c,
                                         float& root1, float& root2) const {
  float discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    return false;
  }

  float sqrt_discriminant = sqrtf(discriminant);
  root1 = (-b + sqrt_discriminant) / (2 * a);
  root2 = (-b - sqrt_discriminant) / (2 * a);

  return true;
}
