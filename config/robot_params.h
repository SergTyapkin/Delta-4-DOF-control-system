#pragma once

namespace RobotParams {
  // Геометрические параметры дельта-робота (в мм)
  constexpr float BASE_RADIUS = 150.0f;    // Радиус основания
  constexpr float EFFECTOR_RADIUS = 50.0f; // Радиус эффектора
  constexpr float ARM_LENGTH = 300.0f;     // Длина плеча
  constexpr float FOREARM_LENGTH = 600.0f; // Длина предплечья

  // Углы расположения рычагов на основании (в градусах)
  constexpr float BASE_ANGLES[3] = {0.0f, 120.0f, 240.0f};

  // Пределы движения (в мм)
  constexpr float WORKSPACE_MIN_Z = -700.0f;
  constexpr float WORKSPACE_MAX_Z = -400.0f;
  constexpr float WORKSPACE_RADIUS = 200.0f;

  // Пределы углов рычагов (в градусах)
  constexpr float JOINT_MIN_ANGLE = 0.0f;
  constexpr float JOINT_MAX_ANGLE = 90.0f;

  // Параметры приводов
  constexpr float STEPS_PER_REVOLUTION = 200.0f;  // Шагов на оборот
  constexpr float MICROSTEPS = 16.0f;             // Микрошаги
  constexpr float GEAR_RATIO = 5.0f;              // Передаточное отношение

  // Расчетные значения
  constexpr float STEPS_PER_RADIAN =
      (STEPS_PER_REVOLUTION * MICROSTEPS * GEAR_RATIO) / (2.0f * 3.1415926535f);

  // Параметры управления
  constexpr float MAX_VELOCITY = 100.0f;    // мм/с
  constexpr float MAX_ACCELERATION = 500.0f; // мм/с²
  constexpr float CONTROL_FREQUENCY = 500.0f; // Гц
};
