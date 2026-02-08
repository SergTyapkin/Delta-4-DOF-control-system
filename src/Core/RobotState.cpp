#include "RobotState.h"
#include "../../src/utils/Logger.h"
#include <Arduino.h>

// Методы RobotState уже определены inline в заголовочном файле,
// поэтому здесь оставляем файл минимальным или добавляем дополнительные методы

// Если нужны не-inline методы, они будут здесь. Например:

/*
void RobotState::updateTimestamp() {
    timestamp = millis();
}

bool RobotState::isValid() const {
    // Проверка корректности состояния
    if (joint_positions[0] != joint_positions[0] || // Проверка NaN
        joint_positions[1] != joint_positions[1] ||
        joint_positions[2] != joint_positions[2]) {
        return false;
    }

    // Дополнительные проверки...
    return true;
}

void RobotState::reset() {
    status = Status::IDLE;
    effector_position = Vector3(0, 0, 0);
    effector_orientation = Vector3(0, 0, 0);

    joint_positions.fill(0);
    joint_velocities.fill(0);
    joint_torques.fill(0);

    target_position = Vector3(0, 0, 0);
    target_joints.fill(0);

    current_velocity = 0;
    target_velocity = 0;
    movement_progress = 0;

    error_code = 0;
    error_message = "";

    timestamp = millis();
    movement_start_time = 0;
    movement_duration = 0;

    is_homed = false;
    is_moving = false;
    is_paused = false;
    is_emergency = false;
}
*/
