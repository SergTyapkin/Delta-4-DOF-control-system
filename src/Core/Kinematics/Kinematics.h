#pragma once

#include <Arduino.h>
#include "DeltaSolver.h"
#include "../../../src/utils/Vector3.h"
#include "../../../src/utils/MathUtils.h"
#include "../../../config/limits.h"

class Kinematics {
public:
  // Результат вычислений
  struct Result {
    bool valid;
    float joint_angles[RobotParams::MOTORS_COUNT];  // В радианах
    uint16_t error_code;
    char error_message[48];

    Result() : valid(false), error_code(0) {
      joint_angles[0] = joint_angles[1] = joint_angles[2] = joint_angles[3] = 0;
      error_message[0] = '\0';
    }
  };

  // Конструктор
  Kinematics();

  // Инициализация
  void init(const DeltaSolver::DeltaConfig& config);

  // Прямая кинематика: углы -> позиция
  bool forward(const float angles[RobotParams::MOTORS_COUNT], Vector3& position);

  // Обратная кинематика: позиция -> углы
  Result inverse(const Vector3& position);

  // Обратная кинематика с проверкой безопасности
  Result inverseSafe(const Vector3& position);

  // Проверка достижимости точки
  bool isReachable(const Vector3& position);

  // Проверка безопасности точки
  bool isSafe(const Vector3& position) const;

  // Проверка безопасности углов
  bool areAnglesSafe(const float angles[RobotParams::MOTORS_COUNT]) const;

  // Преобразование скорости в пространстве задач в скорости шарниров
  bool velocityMapping(const Vector3& position, const Vector3& task_velocity,
                       float joint_velocities[RobotParams::MOTORS_COUNT]);

  // Получение границ рабочего пространства
  void getWorkspaceBounds(float& min_radius, float& max_radius,
                          float& min_z, float& max_z) const;

  // Получение матрицы Якобиана
  bool getJacobian(const Vector3& position, float jacobian[RobotParams::MOTORS_COUNT][3]);

  // Получение определителя Якобиана
  float getJacobianDeterminant(const Vector3& position);

  // Получение текущей конфигурации
  const DeltaSolver::DeltaConfig& getConfig() const;

private:
  DeltaSolver solver_;

  // Проверка границ рабочего пространства
  bool checkWorkspaceBounds(const Vector3& position) const;

  // Проверка угловых ограничений
  bool checkJointLimits(const float angles[RobotParams::MOTORS_COUNT]) const;

  // Проверка на коллизии
  bool checkCollisions(const Vector3& position, const float angles[RobotParams::MOTORS_COUNT]) const;
};
