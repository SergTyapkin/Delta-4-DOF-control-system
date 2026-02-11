#pragma once

#include <Arduino.h>
#include "../../../src/utils/Vector3.h"
#include "../../../config/robot_params.h"
#include "../../../config/limits.h"
#include "../../../src/utils/MathUtils.h"

class DeltaSolver {
public:
  // Структура конфигурации робота
  struct DeltaConfig {
    float base_radius;      // Радиус основания (мм)
    float effector_radius;  // Радиус эффектора (мм)
    float arm_length;       // Длина верхнего рычага (мм)
    float forearm_length;   // Длина нижнего рычага (мм)
    float base_angles[3];   // Углы расположения рычагов на основании (рад)

    DeltaConfig() :
        base_radius(RobotParams::BASE_RADIUS),
        effector_radius(RobotParams::EFFECTOR_RADIUS),
        arm_length(RobotParams::ARM_LENGTH),
        forearm_length(RobotParams::FOREARM_LENGTH) {

      base_angles[0] = RobotParams::BASE_ANGLES[0] * MathUtils::DEG_TO_RAD;
      base_angles[1] = RobotParams::BASE_ANGLES[1] * MathUtils::DEG_TO_RAD;
      base_angles[2] = RobotParams::BASE_ANGLES[2] * MathUtils::DEG_TO_RAD;
    }

    bool isValid() const {
      return (base_radius > 0 && effector_radius > 0 &&
              arm_length > 0 && forearm_length > 0 &&
              forearm_length > arm_length);
    }
  };

  // Результат решения
  struct Solution {
    float angles[3];        // Углы рычагов (радианы)
    bool valid;             // Решение валидно
    uint8_t error_code;     // Код ошибки
    char error_message[48]; // Сообщение об ошибке

    Solution() : valid(false), error_code(0) {
      angles[0] = angles[1] = angles[2] = 0;
      error_message[0] = '\0';
    }
  };

  // Типы сингулярностей
  enum SingularityType {
    SING_NONE,
    SING_PARALLEL,
    SING_EXTENDED,
    SING_RETRACTED,
    SING_COLLISION
  };

  // Конструктор
  DeltaSolver();

  // Инициализация с кастомной конфигурацией
  void init(const DeltaConfig& config);

  // Прямая кинематика: углы → позиция эффектора
  bool forwardKinematics(const float angles[3], Vector3& position);

  // Обратная кинематика: позиция эффектора → углы
  Solution inverseKinematics(const Vector3& position);

  // Обратная кинематика с проверкой пределов
  Solution inverseKinematicsSafe(const Vector3& position);

  // Проверка достижимости точки
  bool isReachable(const Vector3& position);

  // Получение текущей конфигурации
  const DeltaConfig& getConfig() const { return config_; }

  // Проверка сингулярностей
  SingularityType checkSingularity(const Vector3& position, const float angles[3]);

  // Вычисление матрицы Якобиана
  bool computeJacobian(const Vector3& position, float jacobian[3][3]);

  // Преобразование скорости
  bool taskToJointVelocity(const Vector3& position, const Vector3& task_velocity,
                           float joint_velocity[3]);

  // Получение границ рабочего пространства
  void getWorkspaceBounds(float& min_radius, float& max_radius,
                          float& min_z, float& max_z) const;

  // Вычисление позиций шарниров
  Vector3 getUpperJointPosition(float angle, int arm_index) const;
  Vector3 getEffectorJointPosition(const Vector3& effector_pos, int arm_index) const;

private:
  // Конфигурация робота
  DeltaConfig config_;

  // Предвычисленные значения
  float cos_base_angles_[3];
  float sin_base_angles_[3];
  float forearm_squared_;

  // Приватные методы
  void precomputeConstants();

  // Решение для одного рычага
  float solveForArm(const Vector3& position, int arm_index);

  // Проверка решения
  bool isSolutionPhysical(float angle, int arm_index);

  // Расчет расстояния между шарнирами
  float calculateJointDistance(const Vector3& upper_joint,
                               const Vector3& effector_joint) const;

  // Решение квадратного уравнения
  bool solveQuadraticEquation(float a, float b, float c,
                              float& root1, float& root2) const;
};
