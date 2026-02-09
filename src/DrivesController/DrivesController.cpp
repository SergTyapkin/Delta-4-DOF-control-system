#include <Arduino.h>
#include "DrivesController.h"
#include "../../src/utils/Logger.h"

DrivesController::DrivesController() :
    state_(State::IDLE),
    previous_state_(State::IDLE),
    mode_(Mode::INDEPENDENT),
    command_in_progress_(false),
    command_start_time_(0),
    sync_in_progress_(false),
    sync_start_time_(0),
    commands_executed_(0),
    commands_failed_(0),
    last_update_time_(0) {

  sync_drives_done_.fill(false);
}

bool DrivesController::init(const Config& config) {
  config_ = config;

  Logger::info("Initializing DrivesController...");

  // Инициализация каждого привода
  bool all_initialized = true;
  for (uint8_t i = 0; i < 3; i++) {
    if (!drives_[i].init(config_.drive_configs[i], i + 1)) {
      Logger::error("Failed to initialize drive %d", i);
      all_initialized = false;
    } else {
      // Устанавливаем callback'и
      drives_[i].setStateChangeCallback(
          [this, i](Drive::State old_state, Drive::State new_state) {
            onDriveStateChanged(i, old_state, new_state);
          }
      );

      drives_[i].setPositionUpdateCallback(
          [this, i](float position, float velocity) {
            onDrivePositionUpdated(i, position, velocity);
          }
      );

      drives_[i].setHomingCompleteCallback(
          [this, i](bool success) {
            onDriveHomingComplete(i, success);
          }
      );
    }
  }

  if (!all_initialized) {
    Logger::error("DrivesController initialization failed");
    return false;
  }

  // Изначально отключаем все приводы
  disableAll();

  Logger::info("DrivesController initialized successfully");
  Logger::info("Mode: %s, Sync tolerance: %.3f rad",
               config_.enable_sync_move ? "SYNCHRONIZED" : "INDEPENDENT",
               config_.sync_tolerance);

  return true;
}

void DrivesController::update(uint32_t delta_time_ms) {
  // Сохраняем предыдущее состояние
  previous_state_ = state_;

  // Обновляем каждый привод
  for (auto& drive : drives_) {
    drive.update(delta_time_ms);
  }

  // Обрабатываем синхронное движение
  if (sync_in_progress_) {
    updateSyncMove();
  }

  // Обрабатываем очередь команд
  processCommandQueue();

  // Обновляем состояние контроллера
  updateControllerState();

  // Вызываем callback при изменении состояния
  if (state_ != previous_state_ && state_change_callback_) {
    state_change_callback_(previous_state_, state_);
  }

  last_update_time_ = millis();
}

bool DrivesController::executeCommand(const Command& command) {
  Logger::debug("Executing command type: %d", static_cast<int>(command.type));

  bool success = false;
  command_start_time_ = millis();

  switch (command.type) {
    case Command::Type::MOVE_TO_POSITION:
      success = moveToPosition(command.positions,
                               command.velocities[0],
                               command.drive_mask);
      break;

    case Command::Type::MOVE_TO_POSITION_SYNC:
      success = moveToPositionSync(command.positions,
                                   command.velocities[0]);
      break;

    case Command::Type::SET_VELOCITY:
      success = setVelocities(command.velocities);
      break;

    case Command::Type::HOME:
      if (command.drive_mask == 0b111) {
        success = homeAll();
      } else {
        // Homing отдельных приводов
        for (uint8_t i = 0; i < 3; i++) {
          if (command.drive_mask & (1 << i)) {
            success = homeSingle(i) && success;
          }
        }
      }
      break;

    case Command::Type::ENABLE:
      enableAll();
      success = true;
      break;

    case Command::Type::DISABLE:
      disableAll();
      success = true;
      break;

    case Command::Type::STOP:
      softStop();
      success = true;
      break;

    case Command::Type::EMERGENCY_STOP:
      emergencyStop();
      success = true;
      break;

    default:
      Logger::warning("Unknown command type: %d",
                      static_cast<int>(command.type));
      success = false;
  }

  // Логирование результата
  logCommand(command, success);

  if (success) {
    commands_executed_++;

    if (command_complete_callback_) {
      command_complete_callback_(commands_executed_);
    }
  } else {
    commands_failed_++;
  }

  return success;
}

bool DrivesController::addCommandToQueue(const Command& command) {
  if (command_queue_.isFull()) {
    Logger::warning("Command queue is full");
    return false;
  }

  bool success = command_queue_.push(command);

  if (success) {
    Logger::debug("Command added to queue. Queue size: %d",
                  command_queue_.size());
  }

  return success;
}

void DrivesController::clearCommandQueue() {
  command_queue_.clear();
  Logger::info("Command queue cleared");
}

bool DrivesController::moveToPosition(const std::array<float, 3>& positions,
                                      float velocity,
                                      uint8_t drive_mask) {

  if (!checkDrivesReady()) {
    Logger::warning("Cannot move: drives not ready");
    return false;
  }

  if (!checkPositionsValid(positions)) {
    Logger::warning("Invalid target positions");
    return false;
  }

  bool success = true;

  for (uint8_t i = 0; i < 3; i++) {
    if (drive_mask & (1 << i)) {
      if (!drives_[i].moveToPosition(positions[i], velocity)) {
        success = false;
        Logger::error("Drive %d failed to move to position", i);

        if (config_.stop_on_single_error) {
          // Останавливаем все приводы при ошибке одного
          emergencyStop();
          break;
        }
      }
    }
  }

  if (success) {
    state_ = State::MOVING;
    Logger::info("Moving to positions: [%.2f, %.2f, %.2f] deg",
                 positions[0] * 57.2958f,
                 positions[1] * 57.2958f,
                 positions[2] * 57.2958f);
  }

  return success;
}

bool DrivesController::moveToPositionSync(const std::array<float, 3>& positions,
                                          float velocity) {

  if (!config_.enable_sync_move) {
    Logger::warning("Sync move is disabled");
    return moveToPosition(positions, velocity);
  }

  if (!checkDrivesReady()) {
    return false;
  }

  if (!checkPositionsValid(positions)) {
    return false;
  }

  // Устанавливаем целевые позиции
  bool all_success = true;
  for (uint8_t i = 0; i < 3; i++) {
    if (!drives_[i].moveToPosition(positions[i], velocity)) {
      all_success = false;
      Logger::error("Drive %d failed to set target position", i);
    }
  }

  if (!all_success) {
    return false;
  }

  // Инициализируем синхронное движение
  sync_in_progress_ = true;
  sync_start_time_ = millis();
  sync_drives_done_.fill(false);
  state_ = State::SYNCING;

  Logger::info("Starting synchronized move");

  return true;
}

bool DrivesController::setVelocities(const std::array<float, 3>& velocities) {
  if (!checkDrivesReady()) {
    return false;
  }

  if (!checkVelocitiesValid(velocities)) {
    return false;
  }

  bool success = true;

  for (uint8_t i = 0; i < 3; i++) {
    if (!drives_[i].setVelocity(velocities[i])) {
      success = false;
      Logger::error("Drive %d failed to set velocity", i);
    }
  }

  if (success) {
    state_ = State::MOVING;
    Logger::info("Velocities set: [%.2f, %.2f, %.2f] rad/s",
                 velocities[0], velocities[1], velocities[2]);
  }

  return success;
}

bool DrivesController::homeAll() {
  if (state_ != State::IDLE && state_ != State::ERROR) {
    Logger::warning("Cannot start homing: controller not idle");
    return false;
  }

  Logger::info("Starting homing sequence for all drives");

  state_ = State::HOMING;
  bool all_success = true;

  for (uint8_t i = 0; i < 3; i++) {
    if (!drives_[i].startHoming()) {
      all_success = false;
      Logger::error("Drive %d failed to start homing", i);

      if (config_.stop_on_single_error) {
        emergencyStop();
        break;
      }
    }
  }

  return all_success;
}

bool DrivesController::homeSingle(uint8_t drive_index) {
  if (drive_index >= 3) {
    Logger::error("Invalid drive index: %d", drive_index);
    return false;
  }

  if (state_ != State::IDLE && state_ != State::ERROR) {
    Logger::warning("Cannot start homing: controller not idle");
    return false;
  }

  Logger::info("Starting homing for drive %d", drive_index);

  state_ = State::HOMING;
  return drives_[drive_index].startHoming();
}

void DrivesController::enableAll() {
  for (auto& drive : drives_) {
    drive.enable();
  }

  Logger::info("All drives enabled");
}

void DrivesController::disableAll() {
  for (auto& drive : drives_) {
    drive.disable();
  }

  state_ = State::IDLE;
  Logger::info("All drives disabled");
}

void DrivesController::emergencyStop() {
  for (auto& drive : drives_) {
    drive.emergencyStop();
  }

  state_ = State::EMERGENCY_STOP;
  sync_in_progress_ = false;
  command_in_progress_ = false;

  Logger::critical("EMERGENCY STOP: All drives stopped");
}

void DrivesController::softStop() {
  for (auto& drive : drives_) {
    // Плавная остановка через установку нулевой скорости
    drive.setVelocity(0);
  }

  state_ = State::IDLE;
  sync_in_progress_ = false;

  Logger::info("Soft stop executed");
}

bool DrivesController::resetErrors() {
  bool all_reset = true;

  for (auto& drive : drives_) {
    if (!drive.resetError()) {
      all_reset = false;
    }
  }

  if (all_reset && state_ == State::ERROR) {
    state_ = State::IDLE;
    Logger::info("All errors reset");
  }

  return all_reset;
}

std::array<float, 3> DrivesController::getPositions() const {
  std::array<float, 3> positions;
  for (uint8_t i = 0; i < 3; i++) {
    positions[i] = drives_[i].getPosition();
  }
  return positions;
}

std::array<float, 3> DrivesController::getVelocities() const {
  std::array<float, 3> velocities;
  for (uint8_t i = 0; i < 3; i++) {
    velocities[i] = drives_[i].getVelocity();
  }
  return velocities;
}

std::array<float, 3> DrivesController::getTargetPositions() const {
  std::array<float, 3> targets;
  for (uint8_t i = 0; i < 3; i++) {
    targets[i] = drives_[i].getTargetPosition();
  }
  return targets;
}

Drive::State DrivesController::getDriveState(uint8_t index) const {
  if (index >= 3) return Drive::State::ERROR;
  return drives_[index].getState();
}

bool DrivesController::isDriveEnabled(uint8_t index) const {
  if (index >= 3) return false;
  return drives_[index].isEnabled();
}

bool DrivesController::isDriveHomed(uint8_t index) const {
  if (index >= 3) return false;
  return drives_[index].isHomed();
}

bool DrivesController::isDriveMoving(uint8_t index) const {
  if (index >= 3) return false;
  return drives_[index].isMoving();
}

bool DrivesController::isMoving() const {
  for (const auto& drive : drives_) {
    if (drive.isMoving()) {
      return true;
    }
  }
  return false;
}

bool DrivesController::isHomingComplete() const {
  return (state_ != State::HOMING);
}

bool DrivesController::isAllHomed() const {
  for (const auto& drive : drives_) {
    if (!drive.isHomed()) {
      return false;
    }
  }
  return true;
}

void DrivesController::processCommandQueue() {
  if (command_in_progress_) {
    // Проверяем завершение текущей команды
    if (state_ == State::IDLE || state_ == State::ERROR) {
      command_in_progress_ = false;

      Logger::debug("Command completed, state: %d",
                    static_cast<int>(state_));
    } else if (millis() - command_start_time_ > current_command_.timeout &&
               current_command_.timeout > 0) {
      // Таймаут команды
      Logger::warning("Command timeout after %d ms",
                      current_command_.timeout);
      command_in_progress_ = false;
      emergencyStop();
    }
  }

  // Если нет текущей команды и есть команды в очереди
  if (!command_in_progress_ && !command_queue_.isEmpty()) {
    Command next_command;
    if (command_queue_.pop(next_command)) {
      current_command_ = next_command;
      command_in_progress_ = true;
      command_start_time_ = millis();

      // Выполняем команду
      if (!executeCommand(current_command_)) {
        command_in_progress_ = false;
        Logger::warning("Command execution failed");
      }
    }
  }
}

void DrivesController::updateSyncMove() {
  if (!sync_in_progress_) return;

  // Проверяем таймаут синхронизации
  if (millis() - sync_start_time_ > config_.sync_timeout) {
    Logger::error("Sync move timeout after %d ms", config_.sync_timeout);
    sync_in_progress_ = false;
    emergencyStop();
    return;
  }

  // Проверяем завершение движения каждого привода
  for (uint8_t i = 0; i < 3; i++) {
    if (!sync_drives_done_[i] && !drives_[i].isMoving()) {
      sync_drives_done_[i] = true;
      Logger::debug("Drive %d sync completed", i);
    }
  }

  // Проверяем завершение всех приводов
  checkSyncCompletion();
}

void DrivesController::checkSyncCompletion() {
  bool all_done = true;
  for (bool done : sync_drives_done_) {
    if (!done) {
      all_done = false;
      break;
    }
  }

  if (all_done) {
    sync_in_progress_ = false;
    state_ = State::IDLE;

    Logger::info("Synchronized move completed successfully");

    // Проверяем точность синхронизации
    auto positions = getPositions();
    auto targets = getTargetPositions();

    float max_error = 0;
    for (uint8_t i = 0; i < 3; i++) {
      float error = fabs(positions[i] - targets[i]);
      if (error > max_error) {
        max_error = error;
      }
    }

    if (max_error > config_.sync_tolerance) {
      Logger::warning("Sync move tolerance exceeded: %.4f rad > %.4f rad",
                      max_error, config_.sync_tolerance);
    }
  }
}

void DrivesController::onDriveStateChanged(uint8_t index,
                                           Drive::State old_state,
                                           Drive::State new_state) {
  Logger::debug("Drive %d state changed: %d -> %d",
                index, static_cast<int>(old_state), static_cast<int>(new_state));

  if (drive_state_callback_) {
    drive_state_callback_(index, old_state, new_state);
  }

  // Если привод перешел в состояние ошибки
  if (new_state == Drive::State::ERROR && config_.stop_on_single_error) {
    Logger::error("Drive %d error -> stopping all drives", index);
    emergencyStop();
  }
}

void DrivesController::onDrivePositionUpdated(uint8_t index,
                                              float position,
                                              float velocity) {
  // Можно использовать для мониторинга в реальном времени
  // Например, для визуализации или обратной связи
}

void DrivesController::onDriveHomingComplete(uint8_t index, bool success) {
  Logger::info("Drive %d homing %s", index, success ? "successful" : "failed");

  // Проверяем завершение homing всех приводов
  bool all_homed = true;
  bool any_failed = false;

  for (uint8_t i = 0; i < 3; i++) {
    if (!drives_[i].isHomed()) {
      all_homed = false;
    }
    if (drives_[i].getState() == Drive::State::ERROR) {
      any_failed = true;
    }
  }

  if (all_homed) {
    state_ = State::IDLE;
    Logger::info("All drives homed successfully");

    if (homing_complete_callback_) {
      homing_complete_callback_(true);
    }
  } else if (any_failed) {
    state_ = State::ERROR;
    Logger::error("Homing failed for one or more drives");

    if (homing_complete_callback_) {
      homing_complete_callback_(false);
    }
  }
}

bool DrivesController::checkDrivesReady() const {
  for (const auto& drive : drives_) {
    Drive::State state = drive.getState();
    if (state == Drive::State::ERROR ||
        state == Drive::State::LIMIT_TRIGGERED) {
      Logger::warning("Drive not ready (state: %d)",
                      static_cast<int>(state));
      return false;
    }
  }

  if (!isAllHomed()) {
    Logger::warning("Drives not homed");
    return false;
  }

  return true;
}

bool DrivesController::checkPositionsValid(const std::array<float, 3>& positions) const {
  for (uint8_t i = 0; i < 3; i++) {
    // Проверка на NaN и бесконечность
    if (std::isnan(positions[i]) || std::isinf(positions[i])) {
      Logger::error("Invalid position for drive %d: %f", i, positions[i]);
      return false;
    }

    // Дополнительные проверки могут быть добавлены здесь
    // (например, проверка пределов для конкретного привода)
  }

  return true;
}

bool DrivesController::checkVelocitiesValid(const std::array<float, 3>& velocities) const {
  for (uint8_t i = 0; i < 3; i++) {
    if (std::isnan(velocities[i]) || std::isinf(velocities[i])) {
      Logger::error("Invalid velocity for drive %d: %f", i, velocities[i]);
      return false;
    }
  }

  return true;
}

void DrivesController::updateControllerState() {
  // Определяем состояние контроллера на основе состояния приводов
  if (state_ == State::EMERGENCY_STOP || state_ == State::ERROR) {
    return; // Эти состояния устанавливаются явно
  }

  // Проверяем, есть ли приводы в состоянии ошибки
  for (const auto& drive : drives_) {
    if (drive.getState() == Drive::State::ERROR) {
      state_ = State::ERROR;
      return;
    }
  }

  // Проверяем, движется ли хотя бы один привод
  bool any_moving = false;
  for (const auto& drive : drives_) {
    if (drive.isMoving()) {
      any_moving = true;
      break;
    }
  }

  if (any_moving) {
    if (sync_in_progress_) {
      state_ = State::SYNCING;
    } else {
      state_ = State::MOVING;
    }
  } else {
    // Проверяем, выполняется ли homing
    bool any_homing = false;
    for (const auto& drive : drives_) {
      if (drive.getState() == Drive::State::HOMING_IN_PROGRESS) {
        any_homing = true;
        break;
      }
    }

    if (any_homing) {
      state_ = State::HOMING;
    } else {
      state_ = State::IDLE;
    }
  }
}

void DrivesController::logCommand(const Command& cmd, bool success) const {
  const char* type_str = "UNKNOWN";
  switch (cmd.type) {
    case Command::Type::MOVE_TO_POSITION: type_str = "MOVE_TO_POSITION"; break;
    case Command::Type::MOVE_TO_POSITION_SYNC: type_str = "MOVE_TO_POSITION_SYNC"; break;
    case Command::Type::SET_VELOCITY: type_str = "SET_VELOCITY"; break;
    case Command::Type::HOME: type_str = "HOME"; break;
    case Command::Type::ENABLE: type_str = "ENABLE"; break;
    case Command::Type::DISABLE: type_str = "DISABLE"; break;
    case Command::Type::STOP: type_str = "STOP"; break;
    case Command::Type::EMERGENCY_STOP: type_str = "EMERGENCY_STOP"; break;
  }

  Logger::debug("Command %s: %s", type_str, success ? "SUCCESS" : "FAILED");
}

// Callback setters
void DrivesController::setStateChangeCallback(StateChangeCallback callback) {
  state_change_callback_ = callback;
}

void DrivesController::setHomingCompleteCallback(HomingCompleteCallback callback) {
  homing_complete_callback_ = callback;
}

void DrivesController::setCommandCompleteCallback(CommandCompleteCallback callback) {
  command_complete_callback_ = callback;
}

void DrivesController::setDriveStateCallback(DriveStateCallback callback) {
  drive_state_callback_ = callback;
}

void DrivesController::printStatus() const {
  Logger::info("=== DrivesController Status ===");
  Logger::info("State: %d, Mode: %d", static_cast<int>(state_), static_cast<int>(mode_));
  Logger::info("Commands: %d executed, %d failed", commands_executed_, commands_failed_);
  Logger::info("Queue size: %d", command_queue_.size());
  Logger::info("Sync in progress: %s", sync_in_progress_ ? "YES" : "NO");
  Logger::info("All homed: %s", isAllHomed() ? "YES" : "NO");

  auto positions = getPositions();
  auto velocities = getVelocities();

  Logger::info("Positions (deg): [%.2f, %.2f, %.2f]",
               positions[0] * 57.2958f,
               positions[1] * 57.2958f,
               positions[2] * 57.2958f);

  Logger::info("Velocities (rad/s): [%.3f, %.3f, %.3f]",
               velocities[0], velocities[1], velocities[2]);
}

void DrivesController::printDriveInfo(uint8_t index) const {
  if (index >= 3) {
    Logger::error("Invalid drive index: %d", index);
    return;
  }

  drives_[index].printDebugInfo();
}
