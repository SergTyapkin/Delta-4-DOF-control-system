#pragma once

#include <Arduino.h>
#include "Kinematics.h"
#include "../../../src/utils/Vector3.h"
#include "../../../src/utils/Vector6.h"
#include "../../../src/utils/MathUtils.h"
#include "../../../config/limits.h"

class Kinematics {
public:
  // Результат вычислений
  struct Solution {
    float joints_angles[RobotParams::MOTORS_COUNT];        // Углы поворота двигателей (радианы)
    float joints_positions_z[RobotParams::MOTORS_COUNT];     // Позиции начал рычагов по оси Z
    bool valid;             // Решение валидно
    uint8_t error_code;     // Код ошибки
    char error_message[48]; // Сообщение об ошибке

    Solution() : valid(false), error_code(0) {
      joints_angles[0] = joints_angles[1] = joints_angles[2] = joints_angles[3] = 0;
      error_message[0] = '\0';
    }
  };

  // Структура конфигурации робота
  struct DeltaConfig {
    float arm_length;  // Длина рычага (мм)
    Vector3 columns_positions[RobotParams::MOTORS_COUNT]; // Позиции стоек для моторов
    Vector3 column_joint_positions[RobotParams::MOTORS_COUNT]; // Позиции точек на стоек, от которых отходит рычаг
    Vector3 effector_joint_positions[RobotParams::MOTORS_COUNT]; // Позиции креплений рычагов к нижней базе относительно её центра
    float effector_height; // Толщина платформы эффектора от центра шарниров плеча до нижней точки (мм)

    DeltaConfig() :
        arm_length(RobotParams::ARM_LENGTH),
        columns_positions(RobotParams::COLUMNS_POSITIONS),
        column_joint_positions(RobotParams::COLUMN_JOINT_POSITIONS),
        effector_joint_positions(RobotParams::EFFECTOR_JOINT_POSITIONS) {
    }

    bool isValid() const {
      return arm_length > 0;
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
  Kinematics();

  // Инициализация с кастомной конфигурацией
  void init(const DeltaConfig& config);

  // -------------------
  // Обратная кинематика: позиция эффектора → углы
  Solution inverseKinematics(const Vector6& position);

  // Прямая кинематика: углы → позиция эффектора
  bool forwardKinematics(const float joints_angles[RobotParams::MOTORS_COUNT], Vector6& position);

  // Обратная кинематика скоростей: позиция эффектора и скорости → угловые скорости
  bool inverseKinematicsVelocity(
      const Vector6& position,
      const Vector6& velocity,
      float joints_velocities[RobotParams::MOTORS_COUNT]
  );

  // Вспомогательные вычисления
  void getEffectorJointsPositions(const Vector6& position, Vector3 (&result)[RobotParams::MOTORS_COUNT]) const;
  float convertPositionZToJointAngle(const float position_z);
  // -------------------

  // Проверка сингулярностей
  SingularityType checkSingularity(const Vector6& position, const float angles[RobotParams::MOTORS_COUNT]);

  // Получение текущей конфигурации
  const DeltaConfig& getConfig() const;

  // Вычисление матрицы Якобиана
  bool computeAnglesJacobian(const Vector6& position, float jacobian[RobotParams::MOTORS_COUNT][3]);

  // Вычисление позиций шарниров
  Vector6 getEffectorJointPosition(const Vector6& effector_pos, uint8_t arm_index) const;
private:
  // Конфигурация робота
  DeltaConfig config_;

  // Обратная кинематика: позиция эффектора → углы
  Solution solveInverseKinematics(const Vector6& position);
  float solveInverseKinematicsForArm(const Vector3& end_arm_joint_position, const uint8_t arm_index);

  // Прямая кинематика: углы → позиция эффектора
  bool solveForwardKinematics(const float joints_angles[RobotParams::MOTORS_COUNT], Vector6& position);

  // Проверка решения
  bool isSolutionPhysical(float joint_position_z);

  // Проверка границ рабочего пространства
  bool checkWorkspaceBounds(const Vector3& point) const;
  // Проверка границ поворота эффектора
  bool checkEffectorAnglesBounds(const Vector3& angles) const;

  // Проверка на коллизии
  bool checkCollisions(const Vector6& position, const float angles[RobotParams::MOTORS_COUNT]) const;
};
