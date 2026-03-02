#pragma once

#include <Arduino.h>
#include "../../src/utils/Vector6.h"
#include "../../config/robot_params.h"

struct RobotState {
  // Статус робота
  enum Status {
    STATUS_IDLE,
    STATUS_HOMING,
    STATUS_MOVING,
    STATUS_PAUSED,
    STATUS_ERROR,
    STATUS_EMERGENCY_STOP,
    STATUS_CALIBRATING
  };

  // Текущий статус
  Status status;

  // Позиция и наклон эффектора
  Vector6 effector_position;

  // Позиции и скорости шарниров (радианы, рад/с)
  float joint_positions[RobotParams::MOTORS_COUNT];
  float joint_velocities[RobotParams::MOTORS_COUNT];

  // Целевые значения
  Vector6 target_position;

  // Информация о движении
  float current_velocity;     // Текущая скорость (мм/с)
  float target_velocity;      // Целевая скорость (мм/с)
  float movement_progress;    // Прогресс движения (0-1)

  // Ошибки
  uint32_t error_code;
  char error_message[48];

  // Временные метки
  uint32_t timestamp;

  // Флаги состояния
  bool is_homed;
  bool is_moving;
  bool is_paused;
  bool is_emergency;

  // Конструктор
  RobotState();

  // Методы
  String serialize() const;
  void print() const;
};
