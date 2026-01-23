#include "DeltaSolver.h"
#include "utils/Logger.h"
#include <cmath>

DeltaSolver::DeltaSolver() {
  init(DeltaConfig()); // Инициализация конфигурацией по умолчанию
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
  // Для дельта-робота прямая кинематика сложнее обратной
  // Используем итеративный метод (метод Ньютона)

  // Начальное предположение - центр рабочей зоны
  Vector3 guess(0, 0, (Limits::WORKSPACE.min_z + Limits::WORKSPACE.max_z) / 2);

  const uint8_t MAX_ITERATIONS = 50;
  const float TOLERANCE = 0.001f; // 1 микрон

  for (uint8_t iter = 0; iter < MAX_ITERATIONS; iter++) {
    // Вычисляем углы для текущего предположения
    Solution solution = inverseKinematics(guess);

    if (!solution.valid) {
      Logger::warning("Forward kinematics: invalid solution at iteration %d", iter);
      return false;
    }

    // Вычисляем ошибку
    float error[3];
    float max_error = 0;
    for (int i = 0; i < 3; i++) {
      error[i] = angles[i] - solution.angles[i];
      if (fabs(error[i]) > max_error) {
        max_error = fabs(error[i]);
      }
    }

    // Проверка сходимости
    if (max_error < TOLERANCE) {
      position = guess;
      return true;
    }

    // Вычисляем матрицу Якобиана
    float jacobian[3][3];
    if (!computeJacobian(guess, jacobian)) {
      Logger::warning("Forward kinematics: failed to compute Jacobian");
      return false;
    }

    // Решаем систему уравнений: J * Δp = Δθ
    // Используем простейший метод: Δp = J⁻¹ * Δθ
    // Для упрощения используем метод Гаусса-Жордана для матрицы 3x3

    float jacobian_inv[3][3];

    // Вычисляем определитель
    float det = jacobian[0][0] * (jacobian[1][1] * jacobian[2][2] - jacobian[1][2] * jacobian[2][1])
                - jacobian[0][1] * (jacobian[1][0] * jacobian[2][2] - jacobian[1][2] * jacobian[2][0])
                + jacobian[0][2] * (jacobian[1][0] * jacobian[2][1] - jacobian[1][1] * jacobian[2][0]);

    if (fabs(det) < 0.0001f) {
      Logger::warning("Forward kinematics: singular Jacobian matrix");
      return false;
    }

    // Вычисляем обратную матрицу
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

    // Вычисляем коррекцию позиции
    float delta_pos[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        delta_pos[i] += jacobian_inv[i][j] * error[j];
      }
    }

    // Применяем коррекцию
    guess.x += delta_pos[0] * 0.5f; // Коэффициент для стабильности
    guess.y += delta_pos[1] * 0.5f;
    guess.z += delta_pos[2] * 0.5f;
  }

  Logger::warning("Forward kinematics: did not converge");
  return false;
}

DeltaSolver::Solution DeltaSolver::inverseKinematics(const Vector3& position) {
  Solution solution;

  // Проверка базовой достижимости
  if (!isReachable(position)) {
    solution.valid = false;
    solution.error_code = 1;
    return solution;
  }

  // Решаем для каждого рычага
  for (int i = 0; i < 3; i++) {
    solution.angles[i] = solveForArm(position, i);

    if (!isSolutionPhysical(solution.angles[i], i)) {
      solution.valid = false;
      solution.error_code = 2;
      solution.error_message = solution.angles[i];
      return solution;
    }
  }

  solution.valid = true;
  return solution;
}

float DeltaSolver::solveForArm(const Vector3& position, int arm_index) {
  // Позиция шарнира на эффекторе для данного рычага
  Vector3 effector_joint = getEffectorJointPosition(position, arm_index);

  // Координаты шарнира на основании
  float base_joint_x = config_.base_radius * cos_base_angles_[arm_index];
  float base_joint_y = config_.base_radius * sin_base_angles_[arm_index];
  float base_joint_z = 0;

  // Вектор от шарнира основания к шарниру эффектора
  float dx = effector_joint.x - base_joint_x;
  float dy = effector_joint.y - base_joint_y;
  float dz = effector_joint.z - base_joint_z;

  // Расстояние между шарнирами по горизонтали
  float L = sqrtf(dx * dx + dy * dy);

  // Расчет угла для дельта-робота
  // Используем геометрию параллелограмма

  // Длина проекции на горизонтальную плоскость
  float M = L - config_.effector_radius;

  // Вертикальное расстояние
  float N = dz;

  // Расстояние между шарнирами
  float D = sqrtf(M * M + N * N);

  // Проверка на достижимость
  if (D > config_.arm_length + config_.forearm_length ||
      D < fabs(config_.arm_length - config_.forearm_length)) {
    return NAN; // Невозможно достичь
  }

  // Используем закон косинусов для треугольника:
  // arm_length, forearm_length, D
  // cos(θ) = (arm_length² + D² - forearm_length²) / (2 * arm_length * D)

  float numerator = config_.arm_length * config_.arm_length + D * D
                    - config_.forearm_length * config_.forearm_length;
  float denominator = 2.0f * config_.arm_length * D;

  if (denominator < 0.0001f) {
    return NAN; // Деление на ноль
  }

  float cos_theta = numerator / denominator;

  // Ограничение значения косинуса
  cos_theta = MathUtils::clamp(cos_theta, -1.0f, 1.0f);

  // Угол между плечом и горизонталью
  float theta = acosf(cos_theta);

  // Угол между вектором D и горизонталью
  float phi = atan2f(N, M);

  // Итоговый угол рычага
  float angle = theta + phi;

  return angle;
}

DeltaSolver::Solution DeltaSolver::inverseKinematicsSafe(const Vector3& position) {
  Solution solution = inverseKinematics(position);

  if (!solution.valid) {
    return solution;
  }

  // Проверка пределов углов
  for (int i = 0; i < 3; i++) {
    if (!Limits::JOINT_LIMITS[i].isAngleValid(solution.angles[i])) {
      solution.valid = false;
      solution.error_code = 3;
      solution.error_message = solution.angles[i];

      Logger::warning("IK unsafe: angle %d (%.2f deg) out of limits",
                      i, MathUtils::toDegrees(solution.angles[i]));
      break;
    }
  }

  // Проверка сингулярностей
  SingularityType singularity = checkSingularity(position, solution.angles);
  if (singularity != SingularityType::NONE) {
    solution.valid = false;
    solution.error_code = 4 + static_cast<uint8_t>(singularity);

    Logger::warning("IK unsafe: singularity detected (type %d)",
                    static_cast<int>(singularity));
  }

  return solution;
}

bool DeltaSolver::isReachable(const Vector3& position) {
  // Быстрая проверка на нахождение в цилиндре
  float radius = sqrtf(position.x * position.x + position.y * position.y);

  if (radius > Limits::WORKSPACE.max_x) {
    return false;
  }

  if (position.z < Limits::WORKSPACE.min_z || position.z > Limits::WORKSPACE.max_z) {
    return false;
  }

  // Более точная проверка через обратную кинематику
  Solution test_solution = inverseKinematics(position);

  return test_solution.valid;
}

DeltaSolver::SingularityType DeltaSolver::checkSingularity(const Vector3& position,
                                                           const float angles[3]) {
  // Проверка на полностью вытянутое положение
  for (int i = 0; i < 3; i++) {
    Vector3 upper_joint = getUpperJointPosition(angles[i], i);
    Vector3 effector_joint = getEffectorJointPosition(position, i);

    float distance = upper_joint.distanceTo(effector_joint);

    // Если расстояние почти равно сумме длин
    if (fabs(distance - (config_.arm_length + config_.forearm_length)) < 1.0f) {
      return SingularityType::EXTENDED;
    }

    // Если расстояние почти равно разности длин
    if (fabs(distance - fabs(config_.arm_length - config_.forearm_length)) < 1.0f) {
      return SingularityType::RETRACTED;
    }
  }

  // Проверка на параллельность рычагов
  // (упрощенная проверка - в реальности нужен более сложный анализ)

  return SingularityType::NONE;
}

bool DeltaSolver::computeJacobian(const Vector3& position, float jacobian[3][3]) {
  // Численное вычисление Якобиана методом конечных разностей

  const float DELTA = 0.01f; // 0.01 мм

  // Базовое решение
  Solution base_solution = inverseKinematics(position);
  if (!base_solution.valid) {
    return false;
  }

  // Вариации по X, Y, Z
  Vector3 variations[3] = {
      Vector3(DELTA, 0, 0),
      Vector3(0, DELTA, 0),
      Vector3(0, 0, DELTA)
  };

  for (int i = 0; i < 3; i++) { // По координатам
    Vector3 varied_pos = position + variations[i];
    Solution varied_solution = inverseKinematics(varied_pos);

    if (!varied_solution.valid) {
      return false;
    }

    for (int j = 0; j < 3; j++) { // По рычагам
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

  // joint_velocity = J⁻¹ * task_velocity
  // Для простоты используем псевдообратную матрицу: J⁺ = Jᵀ * (J * Jᵀ)⁻¹

  // Вычисляем J * Jᵀ
  float JJT[3][3] = {{0}};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        JJT[i][j] += jacobian[i][k] * jacobian[j][k];
      }
    }
  }

  // Вычисляем определитель (J * Jᵀ)
  float det = JJT[0][0] * (JJT[1][1] * JJT[2][2] - JJT[1][2] * JJT[2][1])
              - JJT[0][1] * (JJT[1][0] * JJT[2][2] - JJT[1][2] * JJT[2][0])
              + JJT[0][2] * (JJT[1][0] * JJT[2][1] - JJT[1][1] * JJT[2][0]);

  if (fabs(det) < 0.0001f) {
    // Сингулярность - не можем преобразовать скорость
    return false;
  }

  // Вычисляем обратную матрицу (J * Jᵀ)⁻¹
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

  // Вычисляем псевдообратную матрицу J⁺ = Jᵀ * (J * Jᵀ)⁻¹
  float J_pseudo[3][3] = {{0}};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        J_pseudo[i][j] += jacobian[k][i] * inv_JJT[k][j];
      }
    }
  }

  // joint_velocity = J⁺ * task_velocity
  for (int i = 0; i < 3; i++) {
    joint_velocity[i] = 0;
    for (int j = 0; j < 3; j++) {
      joint_velocity[i] += J_pseudo[i][j] * (&task_velocity.x)[j];
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
  // Проверка на NaN
  if (isnan(angle)) {
    return false;
  }

  // Проверка на конечность
  if (!isfinite(angle)) {
    return false;
  }

  // Проверка на разумные пределы
  if (angle < -MathUtils::PI || angle > MathUtils::PI) {
    return false;
  }

  return true;
}

bool DeltaSolver::solveQuadraticEquation(float a, float b, float c,
                                         float& root1, float& root2) const {
  float discriminant = b * b - 4 * a * c;

  if (discriminant < 0) {
    return false; // Нет действительных корней
  }

  float sqrt_discriminant = sqrtf(discriminant);
  root1 = (-b + sqrt_discriminant) / (2 * a);
  root2 = (-b - sqrt_discriminant) / (2 * a);

  return true;
}
