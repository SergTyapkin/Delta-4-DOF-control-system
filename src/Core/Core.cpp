#include <Arduino.h>
#include "Core.h"
#include "../../src/utils/Logger.h"
#include "../../src/Core/TrajectoryGenerator.h"

Core::Core() :
    mode_(MODE_IDLE),
    previous_mode_(MODE_IDLE),
    is_initialized_(false),
    is_homed_(false),
    current_command_status_(STATUS_PENDING),
    current_command_id_(0),
    current_command_start_time_(0),
    command_history_index_(0),
    trajectory_points_count_(0),
    current_trajectory_point_(0),
    trajectory_in_progress_(false),
    last_update_time_(0),
    last_state_update_time_(0),
    trajectory_update_interval_(10),
    state_update_callback_(nullptr),
    state_update_context_(nullptr),
    command_complete_callback_(nullptr),
    command_complete_context_(nullptr),
    mode_change_callback_(nullptr),
    mode_change_context_(nullptr),
    error_callback_(nullptr),
    error_context_(nullptr) {
}

bool Core::init(const Config& config) {
  config_ = config;

  Logger::info("Initializing Core system...");

  // Инициализация кинематики
  kinematics_.init(config_.kinematics_config);

  // Инициализация контроллера приводов
  if (!drives_controller_.init(config_.drives_config)) {
    Logger::error("Failed to initialize drives controller");
    return false;
  }

  // Настройка callback'ов контроллера приводов
  drives_controller_.setStateChangeCallback(
      [](DrivesController::State old_state, DrivesController::State new_state, void* ctx) {
        Logger::debug("Drives controller state changed: %d -> %d", old_state, new_state);
      },
      this
  );

  drives_controller_.setHomingCompleteCallback(
      [](bool success, void* ctx) {
        ((Core*)ctx)->onDrivesHomingComplete(success);
      },
      this
  );

  drives_controller_.setCommandCompleteCallback(
      [](uint32_t commands_executed, void* ctx) {
        ((Core*)ctx)->onDrivesCommandComplete(commands_executed);
      },
      this
  );

  drives_controller_.setDriveStateCallback(
      [](uint8_t index, Drive::State old_state, Drive::State new_state, void* ctx) {
        ((Core*)ctx)->onDriveStateChanged(index, old_state, new_state);
      },
      this
  );

  // Инициализация генератора траекторий
  trajectory_generator_.setMaxVelocity(config_.default_velocity);
  trajectory_generator_.setMaxAcceleration(config_.default_acceleration);
  trajectory_update_interval_ = (uint32_t)(1000.0f / config_.trajectory_update_rate);

  // Инициализация состояния робота
  robot_state_.status = RobotState::STATUS_IDLE;
  robot_state_.effector_position = Vector3(0, 0, 0);
  robot_state_.joint_positions[0] = 0;
  robot_state_.joint_positions[1] = 0;
  robot_state_.joint_positions[2] = 0;
  robot_state_.joint_velocities[0] = 0;
  robot_state_.joint_velocities[1] = 0;
  robot_state_.joint_velocities[2] = 0;
  robot_state_.error_code = 0;
  robot_state_.error_message[0] = '\0';

  is_initialized_ = true;

  Logger::info("Core system initialized successfully");
  Logger::info("Default velocity: %.1f mm/s", config_.default_velocity);
  Logger::info("Default acceleration: %.1f mm/s²", config_.default_acceleration);
  Logger::info("Trajectory update rate: %.1f Hz", config_.trajectory_update_rate);

  return true;
}

void Core::update(uint32_t delta_time_ms) {
  if (!is_initialized_) return;

  uint32_t current_time = millis();

  // Сохраняем предыдущий режим для callback'ов
  previous_mode_ = mode_;

  // Обновляем контроллер приводов
  drives_controller_.update(delta_time_ms);

  // Обновляем траекторию, если она выполняется
  if (trajectory_in_progress_) {
    updateTrajectory();
  }

  // Обрабатываем очередь команд
  processCommandQueue();

  // Обновляем состояние робота
  updateState();

  // Отправляем обновление состояния (не чаще чем раз в 50 мс)
  if (current_time - last_state_update_time_ > 50) {
    if (state_update_callback_) {
      state_update_callback_(robot_state_, state_update_context_);
    }
    last_state_update_time_ = current_time;
  }

  // Вызываем callback при изменении режима
  if (mode_ != previous_mode_ && mode_change_callback_) {
    mode_change_callback_(previous_mode_, mode_, mode_change_context_);
  }

  last_update_time_ = current_time;
}

uint32_t Core::executeCommand(const Command& command) {
  if (!is_initialized_) {
    Logger::error("Core not initialized");
    return 0;
  }

  if (mode_ == MODE_ERROR) {
    Logger::warning("Cannot execute command: system in error state");
    return 0;
  }

  // Генерируем ID команды
  current_command_id_++;
  if (current_command_id_ == 0) current_command_id_ = 1;

  Command cmd_with_id = command;
  cmd_with_id.id = current_command_id_;

  // Добавляем команду в очередь
  if (!addCommandToQueue(cmd_with_id)) {
    Logger::error("Failed to add command to queue");
    return 0;
  }

  Logger::info("Command %d queued (type: %d)",
               current_command_id_, command.type);

  return current_command_id_;
}

bool Core::addCommandToQueue(const Command& command) {
  if (command_queue_.isFull()) {
    Logger::warning("Command queue is full");
    return false;
  }

  return command_queue_.push(command);
}

void Core::clearCommandQueue() {
  command_queue_.clear();
  Logger::info("Command queue cleared");
}

bool Core::moveToPoint(const Vector3& point, float velocity, TrajectoryGenerator::TrajectoryType traj_type) {
  Command cmd;
  cmd.type = Command::CMD_MOVE_TO_POINT;
  cmd.target_point = point;
  cmd.velocity = (velocity > 0) ? velocity : config_.default_velocity;
  cmd.trajectory_type = traj_type;

  return (executeCommand(cmd) != 0);
}

bool Core::moveJoints(const float angles[3], float velocity) {
  Command cmd;
  cmd.type = Command::CMD_MOVE_JOINTS;
  cmd.joint_angles[0] = angles[0];
  cmd.joint_angles[1] = angles[1];
  cmd.joint_angles[2] = angles[2];
  cmd.velocity = (velocity > 0) ? velocity : config_.default_velocity;

  return (executeCommand(cmd) != 0);
}

bool Core::setCartesianVelocity(const Vector3& velocity) {
  Logger::warning("Cartesian velocity control not yet implemented");
  return false;
}

bool Core::setJointVelocity(const float velocities[3]) {
  DrivesController::Command drive_cmd;
  drive_cmd.type = DrivesController::Command::SET_VELOCITY;
  drive_cmd.velocities[0] = velocities[0];
  drive_cmd.velocities[1] = velocities[1];
  drive_cmd.velocities[2] = velocities[2];

  return drives_controller_.executeCommand(drive_cmd);
}

bool Core::performHoming() {
  Command cmd;
  cmd.type = Command::CMD_CALIBRATE;

  return (executeCommand(cmd) != 0);
}

bool Core::calibrate() {
  return performHoming();
}

bool Core::setHomePosition(const Vector3& home_point) {
  Command cmd;
  cmd.type = Command::CMD_SET_HOME;
  cmd.target_point = home_point;

  return (executeCommand(cmd) != 0);
}

void Core::stop() {
  Command cmd;
  cmd.type = Command::CMD_STOP;
  executeCommand(cmd);
}

void Core::pause() {
  Command cmd;
  cmd.type = Command::CMD_PAUSE;
  executeCommand(cmd);
}

void Core::resume() {
  Command cmd;
  cmd.type = Command::CMD_RESUME;
  executeCommand(cmd);
}

void Core::emergencyStop() {
  Command cmd;
  cmd.type = Command::CMD_EMERGENCY_STOP;
  executeCommand(cmd);
}

bool Core::reset() {
  if (mode_ != MODE_ERROR) {
    Logger::warning("Reset only available in error mode");
    return false;
  }

  // Сбрасываем ошибки контроллера приводов
  if (!drives_controller_.resetErrors()) {
    Logger::error("Failed to reset drive errors");
    return false;
  }

  // Сбрасываем состояние ядра
  mode_ = MODE_IDLE;
  robot_state_.error_code = 0;
  robot_state_.error_message[0] = '\0';

  Logger::info("System reset successful");

  return true;
}

RobotState Core::getState() const {
  return robot_state_;
}

Core::CommandStatus Core::getCommandStatus(uint32_t command_id) const {
  for (uint8_t i = 0; i < COMMAND_HISTORY_SIZE; i++) {
    if (command_history_[i].command_id == command_id) {
      return command_history_[i].status;
    }
  }

  // Проверяем текущую команду
  if (current_command_.id == command_id) {
    return current_command_status_;
  }

  return STATUS_PENDING;
}

Vector3 Core::getCurrentPosition() const {
  return robot_state_.effector_position;
}

void Core::getCurrentJoints(float angles[3]) const {
  angles[0] = robot_state_.joint_positions[0];
  angles[1] = robot_state_.joint_positions[1];
  angles[2] = robot_state_.joint_positions[2];
}

bool Core::isMoving() const {
  return drives_controller_.getState() == DrivesController::MOVING ||
         drives_controller_.getState() == DrivesController::SYNCING;
}

bool Core::isHomed() const {
  return is_homed_ && drives_controller_.isAllHomed();
}

bool Core::isReady() const {
  return (is_initialized_ && is_homed_ && mode_ != MODE_ERROR);
}

bool Core::startTrajectory(const Vector3& start, const Vector3& end,
                           TrajectoryGenerator::TrajectoryType type, float velocity) {

  if (!checkPointSafety(start) || !checkPointSafety(end)) {
    Logger::error("Trajectory points are not safe");
    return false;
  }

  trajectory_generator_.setType(type);
  trajectory_generator_.setMaxVelocity((velocity > 0) ? velocity : config_.default_velocity);
  trajectory_generator_.setMaxAcceleration(config_.default_acceleration);

  if (!trajectory_generator_.generate(start, end)) {
    Logger::error("Failed to generate trajectory");
    return false;
  }

  trajectory_in_progress_ = true;
  current_trajectory_point_ = 0;
  mode_ = MODE_TRAJECTORY;

  Logger::info("Trajectory started: %s type, %.1f mm/s",
               (type == TrajectoryGenerator::TRAJ_LINEAR) ? "Linear" : "Spline",
               trajectory_generator_.getMaxVelocity());

  return true;
}

bool Core::addTrajectoryPoint(const Vector3& point) {
  if (!checkPointSafety(point)) {
    return false;
  }

  if (trajectory_points_count_ >= MAX_TRAJECTORY_POINTS) {
    Logger::warning("Max trajectory points reached");
    return false;
  }

  trajectory_points_[trajectory_points_count_] = point;
  trajectory_points_count_++;
  return true;
}

bool Core::clearTrajectory() {
  trajectory_points_count_ = 0;
  trajectory_in_progress_ = false;
  return true;
}

bool Core::teachPoint(const Vector3& point, uint32_t point_id) {
  if (!isReady()) {
    Logger::error("Cannot teach point: system not ready");
    return false;
  }

  if (teach_points_count_ >= MAX_TEACH_POINTS) {
    Logger::warning("Max teach points reached");
    return false;
  }

  teach_points_[teach_points_count_].id = point_id;
  teach_points_[teach_points_count_].point = point;
  teach_points_count_++;

  Logger::info("Point %d taught: (%.1f, %.1f, %.1f)",
               point_id, point.x, point.y, point.z);

  return true;
}

void Core::processCommandQueue() {
  // Если текущая команда еще выполняется, ждем ее завершения
  if (current_command_status_ == STATUS_EXECUTING) {
    // Проверяем таймаут команды
    if (millis() - current_command_start_time_ > current_command_.timeout) {
      Logger::warning("Command %d timeout after %d ms",
                      current_command_.id, current_command_.timeout);

      CommandResult result;
      result.command_id = current_command_.id;
      result.status = STATUS_FAILED;
      result.error_code = 1001;
      strcpy(result.error_message, "Command timeout");
      result.execution_time = millis() - current_command_start_time_;

      logCommand(current_command_, STATUS_FAILED, 1001, "Command timeout");

      if (command_complete_callback_) {
        command_complete_callback_(result, command_complete_context_);
      }

      command_history_[command_history_index_] = result;
      command_history_index_ = (command_history_index_ + 1) % COMMAND_HISTORY_SIZE;

      current_command_status_ = STATUS_FAILED;
    }
    return;
  }

  // Если команда завершена (успешно или с ошибкой), берем следующую
  if (current_command_status_ == STATUS_COMPLETED ||
      current_command_status_ == STATUS_FAILED ||
      current_command_status_ == STATUS_CANCELLED) {

    current_command_status_ = STATUS_PENDING;
  }

  // Берем следующую команду из очереди
  if (current_command_status_ == STATUS_PENDING && !command_queue_.isEmpty()) {
    if (command_queue_.pop(current_command_)) {
      current_command_status_ = STATUS_EXECUTING;
      current_command_start_time_ = millis();

      Logger::info("Executing command %d (type: %d)",
                   current_command_.id, current_command_.type);

      executeCurrentCommand();
    }
  }
}

void Core::executeCurrentCommand() {
  bool success = false;
  uint16_t error_code = 0;
  const char* error_message = "";

  switch (current_command_.type) {
    case Command::CMD_MOVE_TO_POINT:
      success = handleMoveToPoint(current_command_);
      break;

    case Command::CMD_MOVE_JOINTS:
      success = handleMoveJoints(current_command_);
      break;

    case Command::CMD_EXECUTE_TRAJECTORY:
      success = handleExecuteTrajectory(current_command_);
      break;

    case Command::CMD_TEACH_POINT:
      success = handleTeachPoint(current_command_);
      break;

    case Command::CMD_STOP:
      success = handleStop(current_command_);
      break;

    case Command::CMD_PAUSE:
      success = handlePause(current_command_);
      break;

    case Command::CMD_RESUME:
      success = handleResume(current_command_);
      break;

    case Command::CMD_EMERGENCY_STOP:
      success = handleEmergencyStop(current_command_);
      break;

    case Command::CMD_SET_HOME:
      success = handleSetHome(current_command_);
      break;

    case Command::CMD_CALIBRATE:
      success = handleCalibrate(current_command_);
      break;

    default:
      error_code = 1002;
      error_message = "Unknown command type";
      Logger::error("Unknown command type: %d", current_command_.type);
      break;
  }

  // Если команда выполняется асинхронно (например, движение),
  // она установит current_command_status_ сама когда завершится
  if (!success && current_command_status_ == STATUS_EXECUTING) {
    current_command_status_ = STATUS_FAILED;

    CommandResult result;
    result.command_id = current_command_.id;
    result.status = STATUS_FAILED;
    result.error_code = error_code;
    strcpy(result.error_message, error_message);
    result.execution_time = millis() - current_command_start_time_;

    logCommand(current_command_, STATUS_FAILED, error_code, error_message);

    if (command_complete_callback_) {
      command_complete_callback_(result, command_complete_context_);
    }

    command_history_[command_history_index_] = result;
    command_history_index_ = (command_history_index_ + 1) % COMMAND_HISTORY_SIZE;
  }
}

void Core::updateState() {
  // Обновляем углы шарниров от контроллера приводов
  float positions[3], velocities[3];
  drives_controller_.getPositions(positions);
  drives_controller_.getVelocities(velocities);

  robot_state_.joint_positions[0] = positions[0];
  robot_state_.joint_positions[1] = positions[1];
  robot_state_.joint_positions[2] = positions[2];

  robot_state_.joint_velocities[0] = velocities[0];
  robot_state_.joint_velocities[1] = velocities[1];
  robot_state_.joint_velocities[2] = velocities[2];

  // Обновляем позицию эффектора через прямую кинематику
  Vector3 effector_position;
  if (kinematics_.forwardKinematics(positions, effector_position)) {
    robot_state_.effector_position = effector_position;
  } else {
    Logger::warning("Failed to compute forward kinematics");
  }

  // Обновляем статус робота
  if (mode_ == MODE_ERROR) {
    robot_state_.status = RobotState::STATUS_ERROR;
  } else if (isMoving()) {
    robot_state_.status = RobotState::STATUS_MOVING;
  } else if (mode_ == MODE_HOMING) {
    robot_state_.status = RobotState::STATUS_HOMING;
//  } else if (isPaused()) {
//    robot_state_.status = RobotState::STATUS_PAUSED;
  } else {
    robot_state_.status = RobotState::STATUS_IDLE;
  }
}

void Core::updateTrajectory() {
  static uint32_t last_trajectory_update = 0;
  uint32_t current_time = millis();

  if (current_time - last_trajectory_update < trajectory_update_interval_) {
    return;
  }

  Vector3 next_point;
  if (trajectory_generator_.getNextPoint(next_point, trajectory_update_interval_)) {
    // Двигаемся к следующей точке траектории
    float joint_angles[3];
    if (convertToJointAngles(next_point, joint_angles)) {
      // Используем прямой контроль шарниров для точности
      DrivesController::Command cmd;
      cmd.type = DrivesController::Command::MOVE_TO_POSITION_SYNC;
      cmd.positions[0] = joint_angles[0];
      cmd.positions[1] = joint_angles[1];
      cmd.positions[2] = joint_angles[2];
      cmd.velocities[0] = config_.default_velocity;
      cmd.velocities[1] = config_.default_velocity;
      cmd.velocities[2] = config_.default_velocity;

      drives_controller_.executeCommand(cmd);
    }
  } else {
    // Траектория завершена
    trajectory_in_progress_ = false;
    mode_ = MODE_IDLE;
    Logger::info("Trajectory completed");
  }

  last_trajectory_update = current_time;
}

bool Core::handleMoveToPoint(const Command& cmd) {
  if (!checkPointSafety(cmd.target_point)) {
    logCommand(cmd, STATUS_FAILED, 2001, "Point not safe");
    return false;
  }

  float joint_angles[3];
  if (!convertToJointAngles(cmd.target_point, joint_angles)) {
    logCommand(cmd, STATUS_FAILED, 2002, "IK solution not found");
    return false;
  }

  if (!checkJointSafety(joint_angles)) {
    logCommand(cmd, STATUS_FAILED, 2003, "Joint angles not safe");
    return false;
  }

  // Выполняем движение
  mode_ = MODE_CARTESIAN;

  DrivesController::Command drive_cmd;
  drive_cmd.type = DrivesController::Command::MOVE_TO_POSITION_SYNC;
  drive_cmd.positions[0] = joint_angles[0];
  drive_cmd.positions[1] = joint_angles[1];
  drive_cmd.positions[2] = joint_angles[2];
  drive_cmd.velocities[0] = cmd.velocity > 0 ? cmd.velocity : config_.default_velocity;
  drive_cmd.velocities[1] = cmd.velocity > 0 ? cmd.velocity : config_.default_velocity;
  drive_cmd.velocities[2] = cmd.velocity > 0 ? cmd.velocity : config_.default_velocity;

  bool success = drives_controller_.executeCommand(drive_cmd);

  if (success) {
    logCommand(cmd, STATUS_EXECUTING);
  } else {
    logCommand(cmd, STATUS_FAILED, 2004, "Drive command failed");
  }

  return success;
}

bool Core::handleMoveJoints(const Command& cmd) {
  if (!checkJointSafety(cmd.joint_angles)) {
    logCommand(cmd, STATUS_FAILED, 2101, "Joint angles not safe");
    return false;
  }

  mode_ = MODE_DIRECT_JOINT;

  DrivesController::Command drive_cmd;
  drive_cmd.type = DrivesController::Command::MOVE_TO_POSITION_SYNC;
  drive_cmd.positions[0] = cmd.joint_angles[0];
  drive_cmd.positions[1] = cmd.joint_angles[1];
  drive_cmd.positions[2] = cmd.joint_angles[2];
  drive_cmd.velocities[0] = cmd.velocity > 0 ? cmd.velocity : config_.default_velocity;
  drive_cmd.velocities[1] = cmd.velocity > 0 ? cmd.velocity : config_.default_velocity;
  drive_cmd.velocities[2] = cmd.velocity > 0 ? cmd.velocity : config_.default_velocity;

  bool success = drives_controller_.executeCommand(drive_cmd);

  if (success) {
    logCommand(cmd, STATUS_EXECUTING);
  } else {
    logCommand(cmd, STATUS_FAILED, 2102, "Drive command failed");
  }

  return success;
}

bool Core::handleExecuteTrajectory(const Command& cmd) {
  // Для простоты используем прямолинейную траекторию от текущей точки
  Vector3 current_pos = getCurrentPosition();

  if (!startTrajectory(current_pos, cmd.target_point,
                       cmd.trajectory_type, cmd.velocity)) {
    logCommand(cmd, STATUS_FAILED, 2201, "Failed to start trajectory");
    return false;
  }

  logCommand(cmd, STATUS_EXECUTING);
  return true;
}

bool Core::handleTeachPoint(const Command& cmd) {
  if (!teachPoint(cmd.target_point, cmd.id)) {
    logCommand(cmd, STATUS_FAILED, 2301, "Failed to teach point");
    return false;
  }

  // Команда обучения выполняется синхронно
  CommandResult result;
  result.command_id = cmd.id;
  result.status = STATUS_COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, STATUS_COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result, command_complete_context_);
  }

  command_history_[command_history_index_] = result;
  command_history_index_ = (command_history_index_ + 1) % COMMAND_HISTORY_SIZE;

  current_command_status_ = STATUS_COMPLETED;

  return true;
}

bool Core::handleStop(const Command& cmd) {
  drives_controller_.softStop();
  trajectory_in_progress_ = false;
  mode_ = MODE_IDLE;

  // Команда остановки выполняется синхронно
  CommandResult result;
  result.command_id = cmd.id;
  result.status = STATUS_COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, STATUS_COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result, command_complete_context_);
  }

  current_command_status_ = STATUS_COMPLETED;

  return true;
}

bool Core::handlePause(const Command& cmd) {
  drives_controller_.softStop();

  CommandResult result;
  result.command_id = cmd.id;
  result.status = STATUS_COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, STATUS_COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result, command_complete_context_);
  }

  current_command_status_ = STATUS_COMPLETED;

  return true;
}

bool Core::handleResume(const Command& cmd) {
  CommandResult result;
  result.command_id = cmd.id;
  result.status = STATUS_COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, STATUS_COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result, command_complete_context_);
  }

  current_command_status_ = STATUS_COMPLETED;

  return true;
}

bool Core::handleEmergencyStop(const Command& cmd) {
  drives_controller_.emergencyStop();
  trajectory_in_progress_ = false;
  mode_ = MODE_ERROR;
  robot_state_.error_code = 9000;
  strcpy(robot_state_.error_message, "Emergency stop");

  CommandResult result;
  result.command_id = cmd.id;
  result.status = STATUS_COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, STATUS_COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result, command_complete_context_);
  }

  current_command_status_ = STATUS_COMPLETED;

  return true;
}

bool Core::handleSetHome(const Command& cmd) {
  Logger::warning("Set home command not fully implemented");

  CommandResult result;
  result.command_id = cmd.id;
  result.status = STATUS_COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, STATUS_COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result, command_complete_context_);
  }

  current_command_status_ = STATUS_COMPLETED;

  return true;
}

bool Core::handleCalibrate(const Command& cmd) {
  mode_ = MODE_HOMING;
  is_homed_ = false;

  bool success = drives_controller_.homeAll();

  if (success) {
    logCommand(cmd, STATUS_EXECUTING);
  } else {
    logCommand(cmd, STATUS_FAILED, 2501, "Failed to start homing");
    mode_ = MODE_ERROR;
  }

  return success;
}

bool Core::convertToJointAngles(const Vector3& point, float angles[3]) {
  DeltaSolver::Solution solution = kinematics_.inverseKinematicsSafe(point);

  if (!solution.valid) {
    Logger::error("IK failed for point (%.1f, %.1f, %.1f)",
                  point.x, point.y, point.z);
    return false;
  }

  angles[0] = solution.angles[0];
  angles[1] = solution.angles[1];
  angles[2] = solution.angles[2];

  return true;
}

bool Core::checkPointSafety(const Vector3& point) {
  // Проверка рабочего пространства
  if (!Limits::SafetyCheck::isWorkspacePointSafe(point.x, point.y, point.z)) {
    Logger::warning("Point (%.1f, %.1f, %.1f) outside workspace",
                    point.x, point.y, point.z);
    return false;
  }

  // Проверка через обратную кинематику
  DeltaSolver::Solution solution = kinematics_.inverseKinematicsSafe(point);
  if (!solution.valid) {
    Logger::warning("Point (%.1f, %.1f, %.1f) not reachable",
                    point.x, point.y, point.z);
    return false;
  }

  return true;
}

bool Core::checkJointSafety(const float angles[3]) {
  return Limits::SafetyCheck::areJointAnglesSafe(angles);
}

void Core::logCommand(const Command& cmd, CommandStatus status,
                      uint16_t error_code, const char* error_msg) {

  const char* type_str = "UNKNOWN";
  switch (cmd.type) {
    case Command::CMD_MOVE_TO_POINT: type_str = "MOVE_TO_POINT"; break;
    case Command::CMD_MOVE_JOINTS: type_str = "MOVE_JOINTS"; break;
    case Command::CMD_EXECUTE_TRAJECTORY: type_str = "EXECUTE_TRAJECTORY"; break;
    case Command::CMD_TEACH_POINT: type_str = "TEACH_POINT"; break;
    case Command::CMD_RUN_PROGRAM: type_str = "RUN_PROGRAM"; break;
    case Command::CMD_STOP: type_str = "STOP"; break;
    case Command::CMD_PAUSE: type_str = "PAUSE"; break;
    case Command::CMD_RESUME: type_str = "RESUME"; break;
    case Command::CMD_EMERGENCY_STOP: type_str = "EMERGENCY_STOP"; break;
    case Command::CMD_SET_HOME: type_str = "SET_HOME"; break;
    case Command::CMD_CALIBRATE: type_str = "CALIBRATE"; break;
  }

  const char* status_str = "UNKNOWN";
  switch (status) {
    case STATUS_PENDING: status_str = "PENDING"; break;
    case STATUS_EXECUTING: status_str = "EXECUTING"; break;
    case STATUS_COMPLETED: status_str = "COMPLETED"; break;
    case STATUS_FAILED: status_str = "FAILED"; break;
    case STATUS_CANCELLED: status_str = "CANCELLED"; break;
  }

  if (status == STATUS_FAILED) {
    Logger::error("Command %d [%s] -> %s (error %d: %s)",
                  cmd.id, type_str, status_str, error_code, error_msg);
  } else {
    Logger::debug("Command %d [%s] -> %s", cmd.id, type_str, status_str);
  }
}

void Core::onDriveStateChanged(uint8_t index, Drive::State old_state,
  Drive::State new_state) {
  // Обработка изменений состояния отдельных приводов
  if (new_state == Drive::STATE_ERROR) {
  Logger::error("Drive %d error detected", index);

    // Если система в режиме движения, переходим в состояние ошибки
    if (mode_ == MODE_CARTESIAN || mode_ == MODE_DIRECT_JOINT || mode_ == MODE_TRAJECTORY) {
      mode_ = MODE_ERROR;
      robot_state_.error_code = 5000 + index;
      sprintf(robot_state_.error_message, "Drive %d error", index);

      if (error_callback_) {
        error_callback_(robot_state_.error_code, robot_state_.error_message, error_context_);
      }
    }
  }
}

void Core::onDrivesHomingComplete(bool success) {
  if (current_command_.type == Command::CMD_CALIBRATE) {
    // Завершаем команду калибровки
    CommandResult result;
    result.command_id = current_command_.id;
    result.execution_time = millis() - current_command_start_time_;

    if (success) {
      is_homed_ = true;
      mode_ = MODE_IDLE;
      result.status = STATUS_COMPLETED;

      logCommand(current_command_, STATUS_COMPLETED);
      Logger::info("Homing completed successfully");
    } else {
      mode_ = MODE_ERROR;
      result.status = STATUS_FAILED;
      result.error_code = 2502;
      strcpy(result.error_message, "Homing failed");

      logCommand(current_command_, STATUS_FAILED, 2502, "Homing failed");
      Logger::error("Homing failed");
    }

    if (command_complete_callback_) {
      command_complete_callback_(result, command_complete_context_);
    }

    command_history_[command_history_index_] = result;
    command_history_index_ = (command_history_index_ + 1) % COMMAND_HISTORY_SIZE;

    current_command_status_ = success ? STATUS_COMPLETED : STATUS_FAILED;
  }
}

void Core::onDrivesCommandComplete(uint32_t commands_executed) {
  // Обработка завершения команд движения
  if (
      current_command_.type == Command::CMD_MOVE_TO_POINT ||
      current_command_.type == Command::CMD_MOVE_JOINTS
  ) {
    // Проверяем, действительно ли движение завершено
    if (!drives_controller_.isMoving()) {
      CommandResult result;
      result.command_id = current_command_.id;
      result.status = STATUS_COMPLETED;
      result.execution_time = millis() - current_command_start_time_;

      logCommand(current_command_, STATUS_COMPLETED);

      if (command_complete_callback_) {
        command_complete_callback_(result, command_complete_context_);
      }

      command_history_[command_history_index_] = result;
      command_history_index_ = (command_history_index_ + 1) % COMMAND_HISTORY_SIZE;

      current_command_status_ = STATUS_COMPLETED;
      mode_ = MODE_IDLE;
    }
  }
}

// Callback setters с контекстом
void Core::setStateUpdateCallback(StateUpdateCallback callback, void* context) {
  state_update_callback_ = callback;
  state_update_context_ = context;
}

void Core::setCommandCompleteCallback(CommandCompleteCallback callback, void* context) {
  command_complete_callback_ = callback;
  command_complete_context_ = context;
}

void Core::setModeChangeCallback(ModeChangeCallback callback, void* context) {
  mode_change_callback_ = callback;
  mode_change_context_ = context;
}

void Core::setErrorCallback(ErrorCallback callback, void* context) {
  error_callback_ = callback;
  error_context_ = context;
}

void Core::printStatus() const {
  Logger::info("=== Core System Status ===");

  const char* mode_str = "UNKNOWN";
  switch (mode_) {
    case MODE_IDLE: mode_str = "IDLE"; break;
    case MODE_DIRECT_JOINT: mode_str = "DIRECT_JOINT"; break;
    case MODE_CARTESIAN: mode_str = "CARTESIAN"; break;
    case MODE_TRAJECTORY: mode_str = "TRAJECTORY"; break;
    case MODE_HOMING: mode_str = "HOMING"; break;
    case MODE_TEACHING: mode_str = "TEACHING"; break;
    case MODE_ERROR: mode_str = "ERROR"; break;
  }

  Logger::info("Mode: %s", mode_str);
  Logger::info("Initialized: %s, Homed: %s",
               is_initialized_ ? "YES" : "NO",
               is_homed_ ? "YES" : "NO");

  Logger::info("Command queue: %d/%d", command_queue_.size(), command_queue_.capacity());
  Logger::info("Current command: ID=%d, Status=%d",
               current_command_.id, current_command_status_);

  Vector3 pos = getCurrentPosition();
  float joints[3];
  getCurrentJoints(joints);

  Logger::info("Position: (%.1f, %.1f, %.1f) mm", pos.x, pos.y, pos.z);
  Logger::info("Joints: (%.2f, %.2f, %.2f) deg",
               joints[0] * MathUtils::RAD_TO_DEG,
               joints[1] * MathUtils::RAD_TO_DEG,
               joints[2] * MathUtils::RAD_TO_DEG);
}

void Core::printKinematicsInfo() const {
  Logger::info("=== Kinematics Info ===");

  const auto& config = kinematics_.getConfig();
  Logger::info("Base radius: %.1f mm", config.base_radius);
  Logger::info("Effector radius: %.1f mm", config.effector_radius);
  Logger::info("Arm length: %.1f mm", config.arm_length);
  Logger::info("Forearm length: %.1f mm", config.forearm_length);

  float min_r, max_r, min_z, max_z;
  kinematics_.getWorkspaceBounds(min_r, max_r, min_z, max_z);

  Logger::info("Workspace: R=%.1f-%.1f mm, Z=%.1f-%.1f mm",
               min_r, max_r, min_z, max_z);
}

void Core::printDriveInfo() const {
  drives_controller_.printStatus();
}
