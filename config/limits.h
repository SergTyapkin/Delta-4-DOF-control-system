#pragma once

#include <Arduino.h>
#include "robot_params.h"
#include "../src/utils/MathUtils.h"
#include "../src/utils/Vector3.h"
#include "../src/utils/Vector6.h"
#include "../src/DrivesController/Drive.h"


namespace Limits {
  // ----------------------------------------------------------------------------
  // Пределы скорости и ускорения
  // ----------------------------------------------------------------------------
  struct DriversLimits {
    // Линейная скорость по умолчанию (мм/с)
    float default_linear_velocity = 30.0f;
    // Линейное ускорение по умолчанию (мм/с²)
    float default_linear_acceleration = 15.0f;
    // Максимальная линейная скорость (мм/с)
    float max_linear_velocity = 100.0f;
    // Максимальная скорость поворота платформы (рад/с)
    float max_angular_velocity = 30.0f * MathUtils::DEG_TO_RAD; // TODO: использовать
    // Максимальное линейное ускорение (мм/с²)
    float max_linear_acceleration = 30.0f;  // TODO: использовать
    // Максимальное угловое ускорение платформы (мм/с²)
    float max_angular_acceleration = 500.0f;  // TODO: использовать

    // Максимальная угловая скорость рычагов (рад/с)
    float max_joint_velocity = 90.0f * MathUtils::DEG_TO_RAD; // TODO: использовать
    // Максимальное угловое ускорение (рад/с²)
    float max_joint_acceleration = 3.0f; // TODO: использовать
//    // Угловая скорость по умолчанию (рад/с)
//    float default_joint_velocity = 100.0f;
//    // Угловое ускорение по умолчанию (рад/с²)
//    float default_joint_acceleration = 30.0f * MathUtils::DEG_TO_RAD;

    // Максимальная скорость при homing (мм/с)
    float homing_velocity = 20.0f;
    // Максимальное ускорении при homing (мм/с)
    float homing_acceleration = 30.0f;
    // Направление homing
    Drive::HomingDirection homing_direction = Drive::HOMING_POSITIVE;

    // Максимальный момент (рывок) (мм/с³)
    float max_jerk = 5000.0f;  // TODO: использовать

    // Максимальный ток на двигатель (А)
    float max_motor_current = 2.0f; // TODO: проверка
    // Ток удержания (А)
    float hold_current = 0.5f; // TODO: используется?

    // Порог перегрузки по току (А)
    float overcurrent_threshold = 2.5f; // TODO: проверка
    // Время до срабатывания защиты (мс)
    uint32_t overcurrent_timeout = 100; // TODO: проверка

    // Компенсация люфта (рад)
    float backlash_compensation = 0.0001f * MathUtils::DEG_TO_RAD;
  };
  constexpr DriversLimits DRIVERS;

  // ----------------------------------------------------------------------------
  // Температурные пределы
  // ----------------------------------------------------------------------------
  struct TemperatureLimits {
    // Максимальная температура двигателей (°C)
    float max_motor_temp = 80.0f; // TODO: проверка
    // Максимальная температура драйверов (°C)
    float max_driver_temp = 70.0f; // TODO: проверка
    // Максимальная температура контроллера (°C)
    float max_controller_temp = 60.0f; // TODO: проверка

    // На сколько отстоит температура предупреждений от критической
    float warning_temp_offset = 10.0f;

    // Температура предупреждения (ниже максимальной на 10°C)
    float getWarningTemp(const float temp) { return temp - warning_temp_offset; } // TODO: использовать
  };
  constexpr TemperatureLimits TEMPERATURE;

  // ----------------------------------------------------------------------------
  // Временные пределы и таймауты
  // ----------------------------------------------------------------------------
  struct TimeLimits {
    // Таймаут homing (мс)
    uint32_t homing_timeout = 30000; // 30 секунд

    // Таймаут движения к точке (мс)
    uint32_t move_timeout = 60000; // 60 секунд
    // TODO: использовать

    // Таймаут синхронизации с контроллером (мс)
    uint32_t controller_sync_timeout = 5000; // 5 секунд

    // Интервал watchdog (мс)
    uint32_t watchdog_interval = 1000;
    // TODO: использовать

    // Максимальное время выполнения задачи (мс)
    uint32_t max_task_time = 10000;

    // Время ожидания перед отключением двигателей при бездействии (мс)
    uint32_t idle_disable_time = 60000; // 60 секунд
    // TODO: использовать
  };

  constexpr TimeLimits TIME;

// ----------------------------------------------------------------------------
// Пределы точности позиционирования
// ----------------------------------------------------------------------------
  struct ToleranceLimits {
    // Допуск по позиции (мм)
    float position_tolerance = 0.1f;
    // Допуск по наклону платформы (рад)
    float angular_tolerance = 1.0f * MathUtils::DEG_TO_RAD;

    // Допуск по углу двигателя (рад)
    float joint_angle_tolerance = 0.2f * MathUtils::DEG_TO_RAD;

    // Проверка достижения целевой позиции
    bool isPositionReached(float target, float actual) const {
      return fabs(target - actual) < position_tolerance;
    }
    // Проверка достижения целевого угла
    bool isAngleReached(float target, float actual) const {
      return fabs(target - actual) < angular_tolerance;
    }
    // Проверка достижения целевого угла двигателя
    bool isJointAngleReached(float target, float actual) const {
      return fabs(target - actual) < joint_angle_tolerance;
    }
  };

  constexpr ToleranceLimits TOLERANCE;

  // ----------------------------------------------------------------------------
  // Пределы углов шарниров (радианы)
  // ----------------------------------------------------------------------------
  struct JointLimits {
    // Предельные смещения по стойке (мм)
    float min_distance = 0.0f;
    float max_distance = 100.0f;

    // Запас безопасности от предельных положений (мм)
    float safety_margin = 5.0f;

    // Проверка смещения по стойке на допустимость
    bool isPositionZValid(const float position_z) const {
      return (position_z <= -min_distance - safety_margin && position_z >= -max_distance + safety_margin);
    }
  };

  // Пределы для каждого из трех шарниров
  constexpr JointLimits JOINT_LIMITS[4] = {
  JointLimits{},
  JointLimits{},
  JointLimits{},
  JointLimits{}
  };

  // ============================================================================
  // ПРЕДЕЛЫ ДВИЖЕНИЯ И БЕЗОПАСНОСТИ
  // ============================================================================
  struct WorkspaceLimits {
    float min_x = -RobotParams::WORKSPACE_RADIUS;
    float max_x = RobotParams::WORKSPACE_RADIUS;
    float min_y = -RobotParams::WORKSPACE_RADIUS;
    float max_y = RobotParams::WORKSPACE_RADIUS;
    float min_z = RobotParams::WORKSPACE_MIN_Z;
    float max_z = RobotParams::WORKSPACE_MAX_Z;

    // Углы Эйлера наклона эффектора
    float min_angle_x = -0.0f * MathUtils::DEG_TO_RAD;
    float max_angle_x = 0.0f * MathUtils::DEG_TO_RAD;
    float min_angle_y = -40.0f * MathUtils::DEG_TO_RAD;
    float max_angle_y = 40.0f * MathUtils::DEG_TO_RAD;
    float min_angle_z = -0.0f * MathUtils::DEG_TO_RAD;
    float max_angle_z = 0.0f * MathUtils::DEG_TO_RAD;


    // Проверка точки на нахождение в рабочей зоне
    bool isPointValid(const float x, const float y, const float z) const {
      return (
          x >= min_x && x <= max_x &&
          y >= min_y && y <= max_y &&
          z >= min_z && z <= max_z
      );
    }
    // Проверка углов на нахождение в границах
    bool isAnglesValid(const float ax, const float ay, const float az) const {
      return (
          ax >= min_angle_x && ax <= max_angle_x &&
          ay >= min_angle_y && ay <= max_angle_y &&
          az >= min_angle_z && az <= max_angle_z
      );
    }
  };

  constexpr WorkspaceLimits WORKSPACE;

  // ----------------------------------------------------------------------------
  // Функции проверки безопасности
  // ----------------------------------------------------------------------------
  namespace SafetyCheck {
    // Проверка рабочей точки на безопасность
    static bool isWorkspacePointSafe(const Vector3& point) {
      return WORKSPACE.isPointValid(point.x, point.y, point.z);
    }
    // Проверка поворота эффектора на безопасность
    static bool isEffectorAnglesSafe(const Vector3& angles) {
      return WORKSPACE.isAnglesValid(angles.x, angles.y, angles.z);
    }

    // Проверка углов шарниров на безопасность
    static bool arePositionsZSafe(const float angles[3]) {
      for (int i = 0; i < RobotParams::MOTORS_COUNT; i++) {
        if (!JOINT_LIMITS[i].isPositionZValid(angles[i])) {
          return false;
        }
      }
      return true;
    }

    // TODO: использовать
    // Проверка скорости движения
    static bool isVelocitySafe(const float velocity) {
      return (velocity >= 0 && velocity <= DRIVERS.max_linear_velocity);
    }

    // TODO: использовать
    // Проверка ускорения
    static bool isAccelerationSafe(const float acceleration) {
      return (acceleration >= 0 && acceleration <= DRIVERS.max_linear_acceleration);
    }
  }
} // namespace Limits
