#pragma once

#include "../src/utils/MathUtils.h"
#include "../src/utils/Vector3.h"
#include "../src/utils/Vector6.h"

namespace RobotParams {
  constexpr uint8_t MOTORS_COUNT = 4;    // Количество моторов

  // Геометрические параметры дельта-робота (в мм)
  //  constexpr float BRIDGE_LENGTH = 70.0f;          // Длина короткой перемычки каждого параллелограмма (мм)
  constexpr float ARM_LENGTH = 337.0f;            // Длина плеча (мм)
  constexpr float EFFECTOR_HEIGHT = 4.0f;         // Толщина платформы эффектора от центра шарниров плеча до нижней точки (мм)
  const Vector3 COLUMNS_POSITIONS[RobotParams::MOTORS_COUNT] = {
    Vector3( 235.0f, 000.0f, 0.0f),
    Vector3( 035.0f, 235.0f, 0.0f),
    Vector3(-035.0f, 235.0f, 0.0f),
    Vector3(-235.0f, 000.0f, 0.0f)
  }; // Позиции стоек для моторов
  const Vector3 COLUMN_JOINT_POSITIONS[RobotParams::MOTORS_COUNT] = {
    Vector3( 198.5f, 000.0f, 0.0f),
    Vector3( 035.0f, 198.5f, 0.0f),
    Vector3(-035.5f, 198.5f, 0.0f),
    Vector3(-198.5f, 000.0f, 0.0f)
  }; // Позиции точек на стоек, от которых отходит рычаг
  const Vector3 EFFECTOR_JOINT_POSITIONS[RobotParams::MOTORS_COUNT] = {
    Vector3( 045.0f, 000.0f, 0.0f),
    Vector3( 035.0f, 045.0f, 0.0f),
    Vector3(-035.0f, 045.0f, 0.0f),
    Vector3(-045.0f, 000.0f, 0.0f),
  }; // Позиции креплений рычагов к нижней базе относительно её центра

  // Пределы движения (в мм)
  constexpr float WORKSPACE_MIN_Z = -700.0f;
  constexpr float WORKSPACE_MAX_Z = -400.0f;
  constexpr float WORKSPACE_RADIUS = 200.0f;

  // Параметры приводов
  constexpr float STEPS_PER_REVOLUTION = 200.0f;      // Шагов на оборот
  constexpr float MICROSTEPS_PER_REVOLUTION = 800.0f; // Микрошаги
  constexpr float MM_PER_REVOLUTION = 2.0f;           // Передаточное отношение винтовой передачи на штангу
  constexpr float GEAR_RATIO = 1.0f;                  // Передаточное число редуктора двигателя

  // Безопасная начальная точка
  const Vector6 SAFE_START_POSITION = Vector6(0, 0, -200, 0, 0, 0);
};
