#pragma once

#include <Arduino.h>
#include "../../src/utils/Vector3.h"

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

  // Позиция эффектора (мм)
  Vector3 effector_position;

  // Позиции и скорости шарниров (радианы, рад/с)
  float joint_positions[3];
  float joint_velocities[3];

  // Целевые значения
  Vector3 target_position;
  float target_joints[3];

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
