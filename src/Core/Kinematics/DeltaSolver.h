#pragma once

#include <cstdint>
#include <string>
#include "../../../src/utils/Vector3.h"
#include "../../../src/utils/MathUtils.h"
#include "../../../src/utils/Utils.h"
#include "../../../config/robot_params.h"
#include "../../../config/limits.h"

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

      for (int i = 0; i < 3; i++) {
        base_angles[i] = MathUtils::toRadians(RobotParams::BASE_ANGLES[i]);
      }
    }

    // Валидация конфигурации
    bool isValid() const {
      return (base_radius > 0 && effector_radius > 0 &&
              arm_length > 0 && forearm_length > 0 &&
              forearm_length > arm_length); // Для дельта-робота важно
    }
  };

  // Результат решения
  struct Solution {
    float angles[3];          // Углы рычагов (радианы)
    bool valid;               // Решение валидно
    uint8_t error_code;       // Код ошибки
    std::string error_message;      // Сообщение об ошибке (если есть)

    Solution() : valid(false), error_code(0), error_message(0) {
      angles[0] = angles[1] = angles[2] = 0;
    }
  };

  // Конструктор
  DeltaSolver();

  // Инициализация с кастомной конфигурацией
  void init(const DeltaConfig& config);

  // Прямая кинематика: углы → позиция эффектора
  // angles[3] - углы трех рычагов в радианах
  bool forwardKinematics(const float angles[3], Vector3& position);

  // Обратная кинематика: позиция эффектора → углы
  // position - целевая позиция эффектора (мм)
  Solution inverseKinematics(const Vector3& position);

  // Обратная кинематика с проверкой пределов
  Solution inverseKinematicsSafe(const Vector3& position);

  // Проверка достижимости точки
  bool isReachable(const Vector3& position);

  // Получение текущей конфигурации
  const DeltaConfig& getConfig() const { return config_; }

  // Проверка сингулярностей в точке
  enum class SingularityType {
    NONE,           // Нет сингулярности
    PARALLEL,       // Параллельное положение
    EXTENDED,       // Полностью вытянуто
    RETRACTED,      // Полностью сложено
    COLLISION       // Возможное столкновение
  };

  SingularityType checkSingularity(const Vector3& position, const float angles[3]);

  // Вычисление матрицы Якобиана
  bool computeJacobian(const Vector3& position, float jacobian[3][3]);

  // Преобразование скорости в пространстве задач в скорость в пространстве шарниров
  bool taskToJointVelocity(const Vector3& position, const Vector3& task_velocity,
                           float joint_velocity[3]);

  // Получение границ рабочего пространства
  void getWorkspaceBounds(float& min_radius, float& max_radius,
                          float& min_z, float& max_z) const;

  // Вычисление позиции шарнира верхнего рычага
  Vector3 getUpperJointPosition(float angle, int arm_index) const;

  // Вычисление позиции шарнира нижнего рычага на эффекторе
  Vector3 getEffectorJointPosition(const Vector3& effector_pos, int arm_index) const;

private:
  // Конфигурация робота
  DeltaConfig config_;

  // Предвычисленные значения для оптимизации
  float cos_base_angles_[3];
  float sin_base_angles_[3];
  float forearm_squared_;  // Квадрат длины предплечья

  // Приватные методы
  void precomputeConstants();

  // Решение для одного рычага
  float solveForArm(const Vector3& position, int arm_index);

  // Проверка решения на физическую реализуемость
  bool isSolutionPhysical(float angle, int arm_index);

  // Расчет расстояния между шарнирами верхнего и нижнего рычагов
  float calculateJointDistance(const Vector3& upper_joint,
                               const Vector3& effector_joint) const;

  // Вспомогательные геометрические вычисления
  bool solveQuadraticEquation(float a, float b, float c,
                              float& root1, float& root2) const;
};
