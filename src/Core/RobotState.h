#pragma once

#include "../../src/utils/Vector3.h"
#include <array>
#include <cstdint>
#include <string>


struct RobotState {
  // Статус робота
  enum class Status {
    IDLE,               // Ожидание
    HOMING,             // Поиск нуля
    MOVING,             // В движении
    PAUSED,             // На паузе
    ERROR,              // Ошибка
    EMERGENCY_STOP,     // Аварийная остановка
    CALIBRATING         // Калибровка
  };

  // Текущий статус
  Status status;

  // Позиция и ориентация эффектора (мм)
  Vector3 effector_position;
  Vector3 effector_orientation; // Для будущего расширения (углы Эйлера)

  // Позиции и скорости шарниров (радианы, рад/с)
  std::array<float, 3> joint_positions;
  std::array<float, 3> joint_velocities;
  std::array<float, 3> joint_torques;    // Для будущего расширения

  // Целевые значения
  Vector3 target_position;
  std::array<float, 3> target_joints;

  // Информация о движении
  float current_velocity;     // Текущая скорость (мм/с)
  float target_velocity;      // Целевая скорость (мм/с)
  float movement_progress;    // Прогресс движения (0-1)

  // Ошибки
  uint32_t error_code;
  String error_message;

  // Временные метки
  uint32_t timestamp;         // Время последнего обновления (мс)
  uint32_t movement_start_time;
  uint32_t movement_duration;

  // Флаги состояния
  bool is_homed;
  bool is_moving;
  bool is_paused;
  bool is_emergency;

  // Конструктор по умолчанию
  RobotState() :
      status(Status::IDLE),
      effector_position(0, 0, 0),
      effector_orientation(0, 0, 0),
      joint_positions({0, 0, 0}),
      joint_velocities({0, 0, 0}),
      joint_torques({0, 0, 0}),
      target_position(0, 0, 0),
      target_joints({0, 0, 0}),
      current_velocity(0),
      target_velocity(0),
      movement_progress(0),
      error_code(0),
      timestamp(0),
      movement_start_time(0),
      movement_duration(0),
      is_homed(false),
      is_moving(false),
      is_paused(false),
      is_emergency(false) {}

  // Метод для сериализации состояния (для передачи по Serial/Web)
  String serialize() const {
    String json = "{";

    json += "\"status\":" + String(static_cast<int>(status)) + ",";
    json += "\"position\":{\"x\":" + String(effector_position.x, 2) +
            ",\"y\":" + String(effector_position.y, 2) +
            ",\"z\":" + String(effector_position.z, 2) + "},";

    json += "\"joints\":[" +
            String(joint_positions[0], 4) + "," +
            String(joint_positions[1], 4) + "," +
            String(joint_positions[2], 4) + "],";

    json += "\"velocities\":[" +
            String(joint_velocities[0], 4) + "," +
            String(joint_velocities[1], 4) + "," +
            String(joint_velocities[2], 4) + "],";

    json += "\"target\":{\"x\":" + String(target_position.x, 2) +
            ",\"y\":" + String(target_position.y, 2) +
            ",\"z\":" + String(target_position.z, 2) + "},";

    json += "\"is_homed\":" + String(is_homed ? "true" : "false") + ",";
    json += "\"is_moving\":" + String(is_moving ? "true" : "false") + ",";
    json += "\"is_paused\":" + String(is_paused ? "true" : "false") + ",";
    json += "\"velocity\":" + String(current_velocity, 2) + ",";
    json += "\"progress\":" + String(movement_progress, 3) + ",";
    json += "\"error_code\":" + String(error_code) + ",";
    json += "\"timestamp\":" + String(timestamp);

    json += "}";
    return json;
  }

  // Метод для вывода в лог
  void print() const {
    Serial.println("=== Robot State ===");
    Serial.print("Status: ");
    switch (status) {
      case Status::IDLE: Serial.println("IDLE"); break;
      case Status::HOMING: Serial.println("HOMING"); break;
      case Status::MOVING: Serial.println("MOVING"); break;
      case Status::PAUSED: Serial.println("PAUSED"); break;
      case Status::ERROR: Serial.println("ERROR"); break;
      case Status::EMERGENCY_STOP: Serial.println("EMERGENCY_STOP"); break;
      case Status::CALIBRATING: Serial.println("CALIBRATING"); break;
    }

    Serial.print("Position: (");
    Serial.print(effector_position.x, 1);
    Serial.print(", ");
    Serial.print(effector_position.y, 1);
    Serial.print(", ");
    Serial.print(effector_position.z, 1);
    Serial.println(") mm");
    Serial.print("Joints: (");
    Serial.print(joint_positions[0] * 57.2958f, 2);
    Serial.print(", ");
    Serial.print(joint_positions[1] * 57.2958f, 2);
    Serial.print(", ");
    Serial.print(joint_positions[2] * 57.2958f, 2);
    Serial.println(") deg");

    Serial.print("Velocity: ");
    Serial.print(current_velocity, 1);
    Serial.print(" mm/s, Progress: ");
    Serial.print(movement_progress * 100, 1);
    Serial.println("%");

    Serial.print("Homed: ");
    Serial.print(is_homed ? "YES" : "NO");
    Serial.print(", Moving: ");
    Serial.print(is_moving ? "YES" : "NO");
    Serial.print(", Paused: ");
    Serial.println(is_paused ? "YES" : "NO");

    if (error_code != 0) {
      Serial.print("Error: ");
      Serial.print(error_code);
      Serial.print(" - ");
      Serial.println(error_message);
    }
  }
};
