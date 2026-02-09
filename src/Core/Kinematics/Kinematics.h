#pragma once

#include <string>
#include "DeltaSolver.h"
#include "../../../src/utils/Vector3.h"
#include "../../../config/limits.h"

class Kinematics {
public:
  // Результат вычислений
  struct Result {
    bool valid;
    std::array<float, 3> joint_angles; // В радианах
    uint16_t error_code;
    std::string error_message;

    Result() : valid(false), error_code(0) {
      joint_angles.fill(0);
    }
  };

  // Конструктор
  Kinematics();

  // Инициализация
  void init(const DeltaSolver::DeltaConfig& config);

  // Прямая кинематика: углы -> позиция
  // angles - углы трех рычагов в радианах
  bool forward(const float angles[3], Vector3& position);
  bool forward(const std::array<float, 3>& angles, Vector3& position);

  // Обратная кинематика: позиция -> углы
  Result inverse(const Vector3& position);

  // Обратная кинематика с проверкой безопасности
  Result inverseSafe(const Vector3& position);

  // Проверка достижимости точки
  bool isReachable(const Vector3& position);

  // Проверка безопасности точки
  bool isSafe(const Vector3& position) const;

  // Проверка безопасности углов
  bool areAnglesSafe(const std::array<float, 3>& angles) const;

  // Преобразование скорости в пространстве задач в скорости шарниров
  bool velocityMapping(const Vector3& position, const Vector3& task_velocity,
                       std::array<float, 3>& joint_velocities);

  // Получение границ рабочего пространства
  void getWorkspaceBounds(float& min_radius, float& max_radius,
                          float& min_z, float& max_z) const;

  // Получение матрицы Якобиана
  bool getJacobian(const Vector3& position, float jacobian[3][3]);

  // Получение определителя Якобиана (для проверки сингулярностей)
  float getJacobianDeterminant(const Vector3& position);

  // Проверка на сингулярность
  bool isSingular(const Vector3& position, float threshold = 0.01f);

  // Получение текущей конфигурации
  const DeltaSolver::DeltaConfig& getConfig() const;

  // Вспомогательные методы
  Vector3 getJointPositions(float angle, uint8_t joint_index) const;
  float getJointDistance(const Vector3& position, uint8_t joint_index) const;

  // Конвертация единиц измерения
  static float radiansToDegrees(float rad) { return rad * 57.2957795131f; }
  static float degreesToRadians(float deg) { return deg * 0.01745329252f; }

private:
  DeltaSolver solver_;

  // Проверка границ рабочего пространства
  bool checkWorkspaceBounds(const Vector3& position) const;

  // Проверка угловых ограничений
  bool checkJointLimits(const std::array<float, 3>& angles) const;

  // Проверка на коллизии (упрощенная)
  bool checkCollisions(const Vector3& position,
                       const std::array<float, 3>& angles) const;
};
