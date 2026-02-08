#pragma once

#include "robot_params.h"
#include <cstdint>

namespace Limits {

// ============================================================================
// ПРЕДЕЛЫ ДВИЖЕНИЯ И БЕЗОПАСНОСТИ
// ============================================================================

// ----------------------------------------------------------------------------
// Пределы по осям в рабочем пространстве (мм)
// ----------------------------------------------------------------------------
  struct WorkspaceLimits {
    float min_x = -RobotParams::WORKSPACE_RADIUS;
    float max_x =  RobotParams::WORKSPACE_RADIUS;
    float min_y = -RobotParams::WORKSPACE_RADIUS;
    float max_y =  RobotParams::WORKSPACE_RADIUS;
    float min_z =  RobotParams::WORKSPACE_MIN_Z;
    float max_z =  RobotParams::WORKSPACE_MAX_Z;

    // Проверка точки на нахождение в рабочей зоне
    bool isPointValid(float x, float y, float z) const {
      return (x >= min_x && x <= max_x &&
              y >= min_y && y <= max_y &&
              z >= min_z && z <= max_z);
    }
  };

// Глобальный экземпляр пределов рабочей зоны
constexpr WorkspaceLimits WORKSPACE;

// ----------------------------------------------------------------------------
// Пределы углов шарниров (радианы)
// ----------------------------------------------------------------------------
struct JointLimits {
  float min_angle = RobotParams::JOINT_MIN_ANGLE * MathUtils::DEG_TO_RAD;
  float max_angle = RobotParams::JOINT_MAX_ANGLE * MathUtils::DEG_TO_RAD;

  // Запас безопасности от предельных положений (радианы)
  float safety_margin = 0.087f; // ~5 градусов

  // Получить безопасные пределы
  float getSafeMin() const { return min_angle + safety_margin; }
  float getSafeMax() const { return max_angle - safety_margin; }

  // Проверка угла на допустимость
  bool isAngleValid(float angle) const {
    return (angle >= getSafeMin() && angle <= getSafeMax());
  }

  // Ограничение угла безопасными пределами
  float clampAngle(float angle) const {
    if (angle < getSafeMin()) return getSafeMin();
    if (angle > getSafeMax()) return getSafeMax();
    return angle;
  }
};

// Пределы для каждого из трех шарниров
  constexpr JointLimits JOINT_LIMITS[3] = {
  JointLimits{},
JointLimits{},
JointLimits{}
};

// ----------------------------------------------------------------------------
// Пределы скорости и ускорения
// ----------------------------------------------------------------------------
struct VelocityLimits {
  // Максимальная линейная скорость (мм/с)
  float max_linear_velocity = RobotParams::MAX_VELOCITY;

  // Максимальная угловая скорость рычагов (рад/с)
  float max_joint_velocity = 1.5f; // ~86 градусов/с

  // Максимальное линейное ускорение (мм/с²)
  float max_linear_acceleration = RobotParams::MAX_ACCELERATION;

  // Максимальное угловое ускорение (рад/с²)
  float max_joint_acceleration = 3.0f;

  // Максимальная скорость при homing (мм/с)
  float homing_velocity = 50.0f;

  // Максимальное рывок (jerk) (мм/с³)
  float max_jerk = 5000.0f;
};

constexpr VelocityLimits VELOCITY;

// ----------------------------------------------------------------------------
// Пределы по току и мощности
// ----------------------------------------------------------------------------
struct CurrentLimits {
  // Максимальный ток на двигатель (А)
  float max_motor_current = 2.0f;

  // Ток удержания (А)
  float hold_current = 0.5f;

  // Ток при homing (А)
  float homing_current = 1.0f;

  // Порог перегрузки по току (А)
  float overcurrent_threshold = 2.5f;

  // Время до срабатывания защиты (мс)
  uint32_t overcurrent_timeout = 100;
};

constexpr CurrentLimits CURRENT;

// ----------------------------------------------------------------------------
// Температурные пределы
// ----------------------------------------------------------------------------
struct TemperatureLimits {
  // Максимальная температура двигателей (°C)
  float max_motor_temp = 80.0f;

  // Максимальная температура драйверов (°C)
  float max_driver_temp = 70.0f;

  // Максимальная температура контроллера (°C)
  float max_controller_temp = 60.0f;

  // Температура предупреждения (ниже максимальной на 10°C)
  float getWarningTemp(float max_temp) const { return max_temp - 10.0f; }
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

  // Интервал watchdog (мс)
  uint32_t watchdog_interval = 1000;

  // Максимальное время выполнения задачи (мс)
  uint32_t max_task_time = 10;

  // Время ожидания перед отключением двигателей при бездействии (мс)
  uint32_t idle_disable_time = 60000; // 60 секунд
};

constexpr TimeLimits TIME;

// ----------------------------------------------------------------------------
// Пределы точности позиционирования
// ----------------------------------------------------------------------------
struct ToleranceLimits {
  // Допуск по позиции (мм)
  float position_tolerance = 0.1f;

  // Допуск по углу (радианы)
  float angle_tolerance = 0.0017f; // ~0.1 градуса

  // Допуск для завершения homing (мм)
  float homing_tolerance = 0.05f;

  // Допуск по скорости (мм/с)
  float velocity_tolerance = 0.5f;

  // Проверка достижения целевой позиции
  bool isPositionReached(float target, float actual) const {
    return fabs(target - actual) < position_tolerance;
  }

  // Проверка достижения целевого угла
  bool isAngleReached(float target, float actual) const {
    return fabs(target - actual) < angle_tolerance;
  }
};

constexpr ToleranceLimits TOLERANCE;

// ----------------------------------------------------------------------------
// Функции проверки безопасности
// ----------------------------------------------------------------------------
namespace SafetyCheck {

  // Проверка рабочей точки на безопасность
  static bool isWorkspacePointSafe(float x, float y, float z) {
    // Основная проверка пределов
    if (!WORKSPACE.isPointValid(x, y, z)) {
      return false;
    }

    // Дополнительные проверки (например, избегание столкновений)
    // Минимальная высота от основания
    const float MIN_HEIGHT_FROM_BASE = 50.0f;
    if (z > -MIN_HEIGHT_FROM_BASE) {
      return false;
    }

    return true;
  }

  // Проверка углов шарниров на безопасность
  static bool areJointAnglesSafe(float angles[3]) {
    for (int i = 0; i < 3; i++) {
      if (!JOINT_LIMITS[i].isAngleValid(angles[i])) {
        return false;
      }
    }

    // Проверка на сингулярности или коллизии
    // (добавить при необходимости)

    return true;
  }

  // Проверка скорости движения
  static bool isVelocitySafe(float velocity) {
    return (velocity >= 0 && velocity <= VELOCITY.max_linear_velocity);
  }

  // Проверка ускорения
  static bool isAccelerationSafe(float acceleration) {
    return (acceleration >= 0 && acceleration <= VELOCITY.max_linear_acceleration);
  }
}

} // namespace Limits
