#include <Arduino.h>
#include "Core.h"
#include "../../src/utils/Logger.h"

Core::Core() :
    mode_(Mode::IDLE),
    previous_mode_(Mode::IDLE),
    is_initialized_(false),
    is_homed_(false),
    is_moving_(false),
    is_paused_(false),
    current_command_status_(CommandStatus::PENDING),
    current_command_id_(0),
    current_command_start_time_(0),
    command_history_index_(0),
    current_trajectory_point_(0),
    trajectory_in_progress_(false),
    last_update_time_(0),
    last_state_update_time_(0),
    trajectory_update_interval_(10) { // 10 мс = 100 Гц

  command_history_.fill(CommandResult());
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
      [this](DrivesController::State old_state, DrivesController::State new_state) {
        // Обработка изменений состояния контроллера приводов
        Logger::debug("Drives controller state changed: %d -> %d",
                      static_cast<int>(old_state), static_cast<int>(new_state));
      }
  );

  drives_controller_.setHomingCompleteCallback(
      [this](bool success) {
        onDrivesHomingComplete(success);
      }
  );

  drives_controller_.setCommandCompleteCallback(
      [this](uint32_t commands_executed) {
        onDrivesCommandComplete(commands_executed);
      }
  );

  drives_controller_.setDriveStateCallback(
      [this](uint8_t index, Drive::State old_state, Drive::State new_state) {
        onDriveStateChanged(index, old_state, new_state);
      }
  );

  // Инициализация генератора траекторий
  trajectory_generator_.setMaxVelocity(config_.default_velocity);
  trajectory_generator_.setMaxAcceleration(config_.default_acceleration);

  // Инициализация состояния робота
  robot_state_.status = RobotState::Status::IDLE;
  robot_state_.effector_position = Vector3(0, 0, 0);
  robot_state_.joint_positions.fill(0);
  robot_state_.joint_velocities.fill(0);
  robot_state_.error_code = 0;

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
  if (trajectory_in_progress_ && !is_paused_) {
    updateTrajectory();
  }

  // Обрабатываем очередь команд
  processCommandQueue();

  // Обновляем состояние робота
  updateState();

  // Отправляем обновление состояния (не чаще чем раз в 50 мс)
  if (current_time - last_state_update_time_ > 50) {
    if (state_update_callback_) {
      state_update_callback_(robot_state_);
    }
    last_state_update_time_ = current_time;
  }

  // Вызываем callback при изменении режима
  if (mode_ != previous_mode_ && mode_change_callback_) {
    mode_change_callback_(previous_mode_, mode_);
  }

  last_update_time_ = current_time;
}

uint32_t Core::executeCommand(const Command& command) {
  if (!is_initialized_) {
    Logger::error("Core not initialized");
    return 0;
  }

  if (mode_ == Mode::ERROR) {
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
               current_command_id_, static_cast<int>(command.type));

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

bool Core::moveToPoint(const Vector3& point, float velocity, TrajectoryType traj_type) {
  Command cmd;
  cmd.type = Command::Type::MOVE_TO_POINT;
  cmd.target_point = point;
  cmd.velocity = (velocity > 0) ? velocity : config_.default_velocity;
  cmd.trajectory_type = traj_type;

  return (executeCommand(cmd) != 0);
}

bool Core::moveJoints(const std::array<float, 3>& angles, float velocity) {
  Command cmd;
  cmd.type = Command::Type::MOVE_JOINTS;
  cmd.joint_angles = angles;
  cmd.velocity = (velocity > 0) ? velocity : config_.default_velocity;

  return (executeCommand(cmd) != 0);
}

bool Core::setCartesianVelocity(const Vector3& velocity) {
  // Для управления скоростью в декартовых координатах
  // нужна более сложная реализация через Jacobian
  Logger::warning("Cartesian velocity control not yet implemented");
  return false;
}

bool Core::setJointVelocity(const std::array<float, 3>& velocities) {
  DrivesController::Command drive_cmd;
  drive_cmd.type = DrivesController::Command::Type::SET_VELOCITY;
  drive_cmd.velocities = velocities;

  return drives_controller_.executeCommand(drive_cmd);
}

bool Core::performHoming() {
  Command cmd;
  cmd.type = Command::Type::CALIBRATE;

  return (executeCommand(cmd) != 0);
}

bool Core::calibrate() {
  // Калибровка может включать homing и дополнительные процедуры
  return performHoming();
}

bool Core::setHomePosition(const Vector3& home_point) {
  Command cmd;
  cmd.type = Command::Type::SET_HOME;
  cmd.target_point = home_point;

  return (executeCommand(cmd) != 0);
}

void Core::stop() {
  Command cmd;
  cmd.type = Command::Type::STOP;
  executeCommand(cmd);
}

void Core::pause() {
  Command cmd;
  cmd.type = Command::Type::PAUSE;
  executeCommand(cmd);
}

void Core::resume() {
  Command cmd;
  cmd.type = Command::Type::RESUME;
  executeCommand(cmd);
}

void Core::emergencyStop() {
  Command cmd;
  cmd.type = Command::Type::EMERGENCY_STOP;
  executeCommand(cmd);
}

bool Core::reset() {
  if (mode_ != Mode::ERROR) {
    Logger::warning("Reset only available in error mode");
    return false;
  }

  // Сбрасываем ошибки контроллера приводов
  if (!drives_controller_.resetErrors()) {
    Logger::error("Failed to reset drive errors");
    return false;
  }

  // Сбрасываем состояние ядра
  mode_ = Mode::IDLE;
  is_moving_ = false;
  is_paused_ = false;
  robot_state_.error_code = 0;

  Logger::info("System reset successful");

  return true;
}

RobotState Core::getState() const {
  return robot_state_;
}

Core::CommandStatus Core::getCommandStatus(uint32_t command_id) const {
  for (const auto& result : command_history_) {
    if (result.command_id == command_id) {
      return result.status;
    }
  }

  // Проверяем текущую команду
  if (current_command_.id == command_id) {
    return current_command_status_;
  }

  return CommandStatus::PENDING;
}

Vector3 Core::getCurrentPosition() const {
  return robot_state_.effector_position;
}

std::array<float, 3> Core::getCurrentJoints() const {
  return robot_state_.joint_positions;
}

bool Core::isMoving() const {
  return is_moving_;
}

bool Core::isHomed() const {
  return is_homed_ && drives_controller_.isAllHomed();
}

bool Core::isReady() const {
  return (is_initialized_ && is_homed_ && mode_ != Mode::ERROR);
}

bool Core::startTrajectory(const Vector3& start, const Vector3& end,
                           TrajectoryType type, float velocity) {

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
  mode_ = Mode::TRAJECTORY;

  Logger::info("Trajectory started: %s type, %.1f mm/s",
               (type == TrajectoryType::LINEAR) ? "Linear" : "Spline",
               trajectory_generator_.getMaxVelocity());

  return true;
}

bool Core::addTrajectoryPoint(const Vector3& point) {
  if (!checkPointSafety(point)) {
    return false;
  }

  trajectory_points_.push_back(point);
  return true;
}

bool Core::clearTrajectory() {
  trajectory_points_.clear();
  trajectory_in_progress_ = false;
  return true;
}

bool Core::teachPoint(const Vector3& point, uint32_t point_id) {
  if (!isReady()) {
    Logger::error("Cannot teach point: system not ready");
    return false;
  }

  teach_points_[point_id] = point;
  Logger::info("Point %d taught: (%.1f, %.1f, %.1f)",
               point_id, point.x, point.y, point.z);

  return true;
}

bool Core::saveTeachPoints() {
  // Здесь должна быть реализация сохранения точек в EEPROM или файл
  Logger::info("Saving %d teach points", teach_points_.size());
  // TODO: Реализовать сохранение
  return true;
}

bool Core::loadTeachPoints() {
  // Здесь должна быть реализация загрузки точек из EEPROM или файла
  Logger::info("Loading teach points");
  // TODO: Реализовать загрузку
  return true;
}

void Core::processCommandQueue() {
  // Если текущая команда еще выполняется, ждем ее завершения
  if (current_command_status_ == CommandStatus::EXECUTING) {
    // Проверяем таймаут команды
    if (millis() - current_command_start_time_ > current_command_.timeout) {
      Logger::warning("Command %d timeout after %d ms",
                      current_command_.id, current_command_.timeout);

      CommandResult result;
      result.command_id = current_command_.id;
      result.status = CommandStatus::FAILED;
      result.error_code = 1001; // Таймаут
      result.error_message = "Command timeout";
      result.execution_time = millis() - current_command_start_time_;

      logCommand(current_command_, CommandStatus::FAILED, 1001, "Timeout");

      if (command_complete_callback_) {
        command_complete_callback_(result);
      }

      // Сохраняем результат в историю
      command_history_[command_history_index_] = result;
      command_history_index_ = (command_history_index_ + 1) % command_history_.size();

      current_command_status_ = CommandStatus::FAILED;
    }
    return;
  }

  // Если команда завершена (успешно или с ошибкой), берем следующую
  if (current_command_status_ == CommandStatus::COMPLETED ||
      current_command_status_ == CommandStatus::FAILED ||
      current_command_status_ == CommandStatus::CANCELLED) {

    current_command_status_ = CommandStatus::PENDING;
  }

  // Берем следующую команду из очереди
  if (current_command_status_ == CommandStatus::PENDING &&
      !command_queue_.isEmpty()) {

    if (command_queue_.pop(current_command_)) {
      current_command_status_ = CommandStatus::EXECUTING;
      current_command_start_time_ = millis();

      Logger::info("Executing command %d (type: %d)",
                   current_command_.id,
                   static_cast<int>(current_command_.type));

      executeCurrentCommand();
    }
  }
}

void Core::executeCurrentCommand() {
  bool success = false;
  uint16_t error_code = 0;
  std::string error_message;

  switch (current_command_.type) {
    case Command::Type::MOVE_TO_POINT:
      success = handleMoveToPoint(current_command_);
      break;

    case Command::Type::MOVE_JOINTS:
      success = handleMoveJoints(current_command_);
      break;

    case Command::Type::EXECUTE_TRAJECTORY:
      success = handleExecuteTrajectory(current_command_);
      break;

    case Command::Type::TEACH_POINT:
      success = handleTeachPoint(current_command_);
      break;

    case Command::Type::STOP:
      success = handleStop(current_command_);
      break;

    case Command::Type::PAUSE:
      success = handlePause(current_command_);
      break;

    case Command::Type::RESUME:
      success = handleResume(current_command_);
      break;

    case Command::Type::EMERGENCY_STOP:
      success = handleEmergencyStop(current_command_);
      break;

    case Command::Type::SET_HOME:
      success = handleSetHome(current_command_);
      break;

    case Command::Type::CALIBRATE:
      success = handleCalibrate(current_command_);
      break;

    default:
      error_code = 1002;
      error_message = "Unknown command type";
      Logger::error("Unknown command type: %d",
                    static_cast<int>(current_command_.type));
      break;
  }

  // Если команда выполняется асинхронно (например, движение),
  // она установит current_command_status_ сама когда завершится
  if (!success && current_command_status_ == CommandStatus::EXECUTING) {
    current_command_status_ = CommandStatus::FAILED;

    CommandResult result;
    result.command_id = current_command_.id;
    result.status = CommandStatus::FAILED;
    result.error_code = error_code;
    result.error_message = error_message;
    result.execution_time = millis() - current_command_start_time_;

    logCommand(current_command_, CommandStatus::FAILED, error_code, error_message);

    if (command_complete_callback_) {
      command_complete_callback_(result);
    }

    command_history_[command_history_index_] = result;
    command_history_index_ = (command_history_index_ + 1) % command_history_.size();
  }
}

void Core::updateState() {
  // Обновляем углы шарниров от контроллера приводов
  auto joint_positions = drives_controller_.getPositions();
  auto joint_velocities = drives_controller_.getVelocities();

  robot_state_.joint_positions = joint_positions;
  robot_state_.joint_velocities = joint_velocities;

  // Обновляем позицию эффектора через прямую кинематику
  float angles_array[3] = {
      joint_positions[0],
      joint_positions[1],
      joint_positions[2]
  };

  Vector3 effector_position;
  if (kinematics_.forwardKinematics(angles_array, effector_position)) {
    robot_state_.effector_position = effector_position;
  } else {
    Logger::warning("Failed to compute forward kinematics");
  }

  // Обновляем статус робота
  if (mode_ == Mode::ERROR) {
    robot_state_.status = RobotState::Status::ERROR;
  } else if (is_moving_) {
    robot_state_.status = RobotState::Status::MOVING;
  } else if (mode_ == Mode::HOMING) {
    robot_state_.status = RobotState::Status::HOMING;
  } else if (is_paused_) {
    robot_state_.status = RobotState::Status::PAUSED;
  } else {
    robot_state_.status = RobotState::Status::IDLE;
  }

  // Обновляем флаги
  is_moving_ = drives_controller_.getState() == DrivesController::State::MOVING ||
               drives_controller_.getState() == DrivesController::State::SYNCING;
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
    std::array<float, 3> joint_angles;
    if (convertToJointAngles(next_point, joint_angles)) {
      // Используем прямой контроль шарниров для точности
      DrivesController::Command cmd;
      cmd.type = DrivesController::Command::Type::MOVE_TO_POSITION_SYNC;
      cmd.positions = joint_angles;
      cmd.velocities.fill(config_.default_velocity);

      drives_controller_.executeCommand(cmd);
    }
  } else {
    // Траектория завершена
    trajectory_in_progress_ = false;
    mode_ = Mode::IDLE;
    Logger::info("Trajectory completed");
  }

  last_trajectory_update = current_time;
}

bool Core::handleMoveToPoint(const Command& cmd) {
  if (!checkPointSafety(cmd.target_point)) {
    logCommand(cmd, CommandStatus::FAILED, 2001, "Point not safe");
    return false;
  }

  std::array<float, 3> joint_angles;
  if (!convertToJointAngles(cmd.target_point, joint_angles)) {
    logCommand(cmd, CommandStatus::FAILED, 2002, "IK solution not found");
    return false;
  }

  if (!checkJointSafety(joint_angles)) {
    logCommand(cmd, CommandStatus::FAILED, 2003, "Joint angles not safe");
    return false;
  }

  // Выполняем движение
  mode_ = Mode::CARTESIAN;

  DrivesController::Command drive_cmd;
  drive_cmd.type = DrivesController::Command::Type::MOVE_TO_POSITION_SYNC;
  drive_cmd.positions = joint_angles;
  drive_cmd.velocities.fill(cmd.velocity > 0 ? cmd.velocity : config_.default_velocity);

  bool success = drives_controller_.executeCommand(drive_cmd);

  if (success) {
    logCommand(cmd, CommandStatus::EXECUTING);
    // Команда будет завершена асинхронно через callback от drives_controller
  } else {
    logCommand(cmd, CommandStatus::FAILED, 2004, "Drive command failed");
  }

  return success;
}

bool Core::handleMoveJoints(const Command& cmd) {
  if (!checkJointSafety(cmd.joint_angles)) {
    logCommand(cmd, CommandStatus::FAILED, 2101, "Joint angles not safe");
    return false;
  }

  mode_ = Mode::DIRECT_JOINT;

  DrivesController::Command drive_cmd;
  drive_cmd.type = DrivesController::Command::Type::MOVE_TO_POSITION_SYNC;
  drive_cmd.positions = cmd.joint_angles;
  drive_cmd.velocities.fill(cmd.velocity > 0 ? cmd.velocity : config_.default_velocity);

  bool success = drives_controller_.executeCommand(drive_cmd);

  if (success) {
    logCommand(cmd, CommandStatus::EXECUTING);
  } else {
    logCommand(cmd, CommandStatus::FAILED, 2102, "Drive command failed");
  }

  return success;
}

bool Core::handleExecuteTrajectory(const Command& cmd) {
  // Для простоты используем прямолинейную траекторию от текущей точки
  Vector3 current_pos = getCurrentPosition();

  if (!startTrajectory(current_pos, cmd.target_point,
                       cmd.trajectory_type, cmd.velocity)) {
    logCommand(cmd, CommandStatus::FAILED, 2201, "Failed to start trajectory");
    return false;
  }

  logCommand(cmd, CommandStatus::EXECUTING);
  return true;
}

bool Core::handleTeachPoint(const Command& cmd) {
  if (!teachPoint(cmd.target_point, cmd.id)) {
    logCommand(cmd, CommandStatus::FAILED, 2301, "Failed to teach point");
    return false;
  }

  // Команда обучения выполняется синхронно
  CommandResult result;
  result.command_id = cmd.id;
  result.status = CommandStatus::COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, CommandStatus::COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result);
  }

  command_history_[command_history_index_] = result;
  command_history_index_ = (command_history_index_ + 1) % command_history_.size();

  current_command_status_ = CommandStatus::COMPLETED;

  return true;
}

bool Core::handleStop(const Command& cmd) {
  drives_controller_.softStop();
  trajectory_in_progress_ = false;
  is_moving_ = false;
  mode_ = Mode::IDLE;

  // Команда остановки выполняется синхронно
  CommandResult result;
  result.command_id = cmd.id;
  result.status = CommandStatus::COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, CommandStatus::COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result);
  }

  current_command_status_ = CommandStatus::COMPLETED;

  return true;
}

bool Core::handlePause(const Command& cmd) {
  is_paused_ = true;
  drives_controller_.softStop();

  CommandResult result;
  result.command_id = cmd.id;
  result.status = CommandStatus::COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, CommandStatus::COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result);
  }

  current_command_status_ = CommandStatus::COMPLETED;

  return true;
}

bool Core::handleResume(const Command& cmd) {
  is_paused_ = false;

  CommandResult result;
  result.command_id = cmd.id;
  result.status = CommandStatus::COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, CommandStatus::COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result);
  }

  current_command_status_ = CommandStatus::COMPLETED;

  return true;
}

bool Core::handleEmergencyStop(const Command& cmd) {
  drives_controller_.emergencyStop();
  trajectory_in_progress_ = false;
  is_moving_ = false;
  is_paused_ = false;
  mode_ = Mode::ERROR;
  robot_state_.error_code = 9000;

  CommandResult result;
  result.command_id = cmd.id;
  result.status = CommandStatus::COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, CommandStatus::COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result);
  }

  current_command_status_ = CommandStatus::COMPLETED;

  return true;
}

bool Core::handleSetHome(const Command& cmd) {
  // Установка домашней позиции требует реализации в драйверах
  Logger::warning("Set home command not fully implemented");

  CommandResult result;
  result.command_id = cmd.id;
  result.status = CommandStatus::COMPLETED;
  result.execution_time = millis() - current_command_start_time_;

  logCommand(cmd, CommandStatus::COMPLETED);

  if (command_complete_callback_) {
    command_complete_callback_(result);
  }

  current_command_status_ = CommandStatus::COMPLETED;

  return true;
}

bool Core::handleCalibrate(const Command& cmd) {
  mode_ = Mode::HOMING;
  is_homed_ = false;

  bool success = drives_controller_.homeAll();

  if (success) {
    logCommand(cmd, CommandStatus::EXECUTING);
    // Homing выполняется асинхронно, завершится через callback
  } else {
    logCommand(cmd, CommandStatus::FAILED, 2501, "Failed to start homing");
    mode_ = Mode::ERROR;
  }

  return success;
}

bool Core::convertToJointAngles(const Vector3& point, std::array<float, 3>& angles) {
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

bool Core::checkJointSafety(const std::array<float, 3>& angles) {
  return Limits::SafetyCheck::areJointAnglesSafe(angles.data());
}

void Core::logCommand(const Command& cmd, CommandStatus status,
                      uint16_t error_code, const std::string& error_msg) {

  const char* type_str = "UNKNOWN";
  switch (cmd.type) {
    case Command::Type::RUN_PROGRAM: type_str = "RUN_PROGRAM"; break;
    case Command::Type::MOVE_TO_POINT: type_str = "MOVE_TO_POINT"; break;
    case Command::Type::MOVE_JOINTS: type_str = "MOVE_JOINTS"; break;
    case Command::Type::EXECUTE_TRAJECTORY: type_str = "EXECUTE_TRAJECTORY"; break;
    case Command::Type::TEACH_POINT: type_str = "TEACH_POINT"; break;
    case Command::Type::STOP: type_str = "STOP"; break;
    case Command::Type::PAUSE: type_str = "PAUSE"; break;
    case Command::Type::RESUME: type_str = "RESUME"; break;
    case Command::Type::EMERGENCY_STOP: type_str = "EMERGENCY_STOP"; break;
    case Command::Type::SET_HOME: type_str = "SET_HOME"; break;
    case Command::Type::CALIBRATE: type_str = "CALIBRATE"; break;
  }

  const char* status_str = "UNKNOWN";
  switch (status) {
    case CommandStatus::PENDING: status_str = "PENDING"; break;
    case CommandStatus::EXECUTING: status_str = "EXECUTING"; break;
    case CommandStatus::COMPLETED: status_str = "COMPLETED"; break;
    case CommandStatus::FAILED: status_str = "FAILED"; break;
    case CommandStatus::CANCELLED: status_str = "CANCELLED"; break;
  }

  if (status == CommandStatus::FAILED) {
    Logger::error("Command %d [%s] -> %s (error %d: %s)",
                  cmd.id, type_str, status_str, error_code, error_msg.c_str());
  } else {
    Logger::debug("Command %d [%s] -> %s", cmd.id, type_str, status_str);
  }
}

void Core::onDriveStateChanged(uint8_t index, Drive::State old_state,
                               Drive::State new_state) {
  // Обработка изменений состояния отдельных приводов
  if (new_state == Drive::State::ERROR) {
    Logger::error("Drive %d error detected", index);

    // Если система в режиме движения, переходим в состояние ошибки
    if (mode_ == Mode::CARTESIAN || mode_ == Mode::DIRECT_JOINT ||
        mode_ == Mode::TRAJECTORY) {
      mode_ = Mode::ERROR;
      robot_state_.error_code = 5000 + index;

      if (error_callback_) {
        error_callback_(robot_state_.error_code,
                        std::string("Drive ") + Utils::toString(index) + " error");
      }
    }
  }
}

void Core::onDrivesHomingComplete(bool success) {
  if (current_command_.type == Command::Type::CALIBRATE) {
    // Завершаем команду калибровки
    CommandResult result;
    result.command_id = current_command_.id;
    result.execution_time = millis() - current_command_start_time_;

    if (success) {
      is_homed_ = true;
      mode_ = Mode::IDLE;
      result.status = CommandStatus::COMPLETED;

      logCommand(current_command_, CommandStatus::COMPLETED);
      Logger::info("Homing completed successfully");
    } else {
      mode_ = Mode::ERROR;
      result.status = CommandStatus::FAILED;
      result.error_code = 2502;
      result.error_message = "Homing failed";

      logCommand(current_command_, CommandStatus::FAILED, 2502, "Homing failed");
      Logger::error("Homing failed");
    }

    if (command_complete_callback_) {
      command_complete_callback_(result);
    }

    command_history_[command_history_index_] = result;
    command_history_index_ = (command_history_index_ + 1) % command_history_.size();

    current_command_status_ = success ? CommandStatus::COMPLETED : CommandStatus::FAILED;
  }
}

void Core::onDrivesCommandComplete(uint32_t commands_executed) {
  // Обработка завершения команд движения
  if (current_command_.type == Command::Type::MOVE_TO_POINT ||
      current_command_.type == Command::Type::MOVE_JOINTS) {

    // Проверяем, действительно ли движение завершено
    if (!drives_controller_.isMoving()) {
      CommandResult result;
      result.command_id = current_command_.id;
      result.status = CommandStatus::COMPLETED;
      result.execution_time = millis() - current_command_start_time_;

      logCommand(current_command_, CommandStatus::COMPLETED);

      if (command_complete_callback_) {
        command_complete_callback_(result);
      }

      command_history_[command_history_index_] = result;
      command_history_index_ = (command_history_index_ + 1) % command_history_.size();

      current_command_status_ = CommandStatus::COMPLETED;
      mode_ = Mode::IDLE;
    }
  }
}

// Callback setters
void Core::setStateUpdateCallback(StateUpdateCallback callback) {
  state_update_callback_ = callback;
}

void Core::setCommandCompleteCallback(CommandCompleteCallback callback) {
  command_complete_callback_ = callback;
}

void Core::setModeChangeCallback(ModeChangeCallback callback) {
  mode_change_callback_ = callback;
}

void Core::setErrorCallback(ErrorCallback callback) {
  error_callback_ = callback;
}

void Core::printStatus() const {
  Logger::info("=== Core System Status ===");

  const char* mode_str = "UNKNOWN";
  switch (mode_) {
    case Mode::IDLE: mode_str = "IDLE"; break;
    case Mode::DIRECT_JOINT: mode_str = "DIRECT_JOINT"; break;
    case Mode::CARTESIAN: mode_str = "CARTESIAN"; break;
    case Mode::TRAJECTORY: mode_str = "TRAJECTORY"; break;
    case Mode::HOMING: mode_str = "HOMING"; break;
    case Mode::TEACHING: mode_str = "TEACHING"; break;
    case Mode::ERROR: mode_str = "ERROR"; break;
  }

  Logger::info("Mode: %s", mode_str);
  Logger::info("Initialized: %s, Homed: %s, Moving: %s, Paused: %s",
               is_initialized_ ? "YES" : "NO",
               is_homed_ ? "YES" : "NO",
               is_moving_ ? "YES" : "NO",
               is_paused_ ? "YES" : "NO");

  Logger::info("Command queue: %d/%d", command_queue_.size(), command_queue_.capacity());
  Logger::info("Current command: ID=%d, Status=%d",
               current_command_.id, static_cast<int>(current_command_status_));

  Vector3 pos = getCurrentPosition();
  auto joints = getCurrentJoints();

  Logger::info("Position: (%.1f, %.1f, %.1f) mm", pos.x, pos.y, pos.z);
  Logger::info("Joints: (%.2f, %.2f, %.2f) deg",
               MathUtils::toDegrees(joints[0]),
               MathUtils::toDegrees(joints[1]),
               MathUtils::toDegrees(joints[2]));
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
