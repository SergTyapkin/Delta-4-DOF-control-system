#include <Arduino.h>
#include "DrivesController.h"
#include "../../src/utils/Logger.h"

DrivesController::DrivesController() :
    state_(IDLE),
    previous_state_(IDLE),
    mode_(INDEPENDENT),
    command_in_progress_(false),
    command_start_time_(0),
    sync_in_progress_(false),
    sync_start_time_(0),
    commands_executed_(0),
    commands_failed_(0),
    last_update_time_(0),
    state_change_callback_(nullptr),
    homing_complete_callback_(nullptr),
    command_complete_callback_(nullptr),
    drive_state_callback_(nullptr) {

  for (int i = 0; i < 3; i++) {
    sync_drives_done_[i] = false;
  }
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
      drives_[i].setStateChangeCallback(
          [](uint8_t idx, Drive::State old_state, Drive::State new_state, void* ctx) {
            ((DrivesController*)ctx)->onDriveStateChanged(idx, old_state, new_state);
          },
          this
      );

      drives_[i].setPositionUpdateCallback(
          [](uint8_t idx, float position, float velocity, void* ctx) {
            ((DrivesController*)ctx)->onDrivePositionUpdated(idx, position, velocity);
          },
          this
      );

      drives_[i].setHomingCompleteCallback(
          [](uint8_t idx, bool success, void* ctx) {
            ((DrivesController*)ctx)->onDriveHomingComplete(idx, success);
          },
          this
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
  for (uint8_t i = 0; i < 3; i++) {
    drives_[i].update(delta_time_ms);
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
    state_change_callback_(previous_state_, state_, state_change_callback_context_);
  }

  last_update_time_ = millis();
}

bool DrivesController::executeCommand(const Command& command) {
  Logger::debug("Executing command type: %d", command.type);

  bool success = false;
  command_start_time_ = millis();

  switch (command.type) {
    case Command::MOVE_TO_POSITION:
      success = moveToPosition(command.positions, command.velocities[0]);
      break;

    case Command::MOVE_TO_POSITION_SYNC:
      success = moveToPositionSync(command.positions, command.velocities[0]);
      break;

    case Command::SET_VELOCITY:
      success = setVelocities(command.velocities);
      break;

    case Command::HOME:
      success = homeAll();
      break;

    case Command::ENABLE:
      enableAll();
      success = true;
      break;

    case Command::DISABLE:
      disableAll();
      success = true;
      break;

    case Command::STOP:
      softStop();
      success = true;
      break;

    case Command::EMERGENCY_STOP:
      emergencyStop();
      success = true;
      break;

    default:
      Logger::warning("Unknown command type: %d", command.type);
      success = false;
  }

  if (success) {
    commands_executed_++;

    if (command_complete_callback_) {
      command_complete_callback_(commands_executed_, command_complete_callback_context_);
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
    Logger::debug("Command added to queue. Queue size: %d", command_queue_.size());
  }

  return success;
}

void DrivesController::clearCommandQueue() {
  command_queue_.clear();
  Logger::info("Command queue cleared");
}

bool DrivesController::moveToPosition(const float positions[3], float velocity) {
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

  if (success) {
    state_ = MOVING;
    Logger::info("Moving to positions: [%.2f, %.2f, %.2f] deg",
                 positions[0] * 57.2958f,
                 positions[1] * 57.2958f,
                 positions[2] * 57.2958f);
  }

  return success;
}

bool DrivesController::moveToPositionSync(const float positions[3], float velocity) {
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
  for (int i = 0; i < 3; i++) {
    sync_drives_done_[i] = false;
  }
  state_ = SYNCING;

  Logger::info("Starting synchronized move");

  return true;
}

bool DrivesController::setVelocities(const float velocities[3]) {
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
    state_ = MOVING;
    Logger::info("Velocities set: [%.2f, %.2f, %.2f] rad/s",
                 velocities[0], velocities[1], velocities[2]);
  }

  return success;
}

bool DrivesController::homeAll() {
  if (state_ != IDLE && state_ != ERROR) {
    Logger::warning("Cannot start homing: controller not idle");
    return false;
  }

  Logger::info("Starting homing sequence for all drives");

  state_ = HOMING;
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

  if (state_ != IDLE && state_ != ERROR) {
  Logger::warning("Cannot start homing: controller not idle");
  return false;
  }

  Logger::info("Starting homing for drive %d", drive_index);

  state_ = HOMING;
  return drives_[drive_index].startHoming();
}

void DrivesController::enableAll() {
  for (uint8_t i = 0; i < 3; i++) {
    drives_[i].enable();
  }

  Logger::info("All drives enabled");
}

void DrivesController::disableAll() {
  for (uint8_t i = 0; i < 3; i++) {
    drives_[i].disable();
  }

  state_ = IDLE;
  Logger::info("All drives disabled");
}

void DrivesController::emergencyStop() {
  for (uint8_t i = 0; i < 3; i++) {
    drives_[i].emergencyStop();
  }

  state_ = EMERGENCY_STOP;
  sync_in_progress_ = false;
  command_in_progress_ = false;

  Logger::critical("EMERGENCY STOP: All drives stopped");
}

void DrivesController::softStop() {
  for (uint8_t i = 0; i < 3; i++) {
    // Плавная остановка через установку нулевой скорости
    drives_[i].setVelocity(0);
  }

  state_ = IDLE;
  sync_in_progress_ = false;

  Logger::info("Soft stop executed");
}

bool DrivesController::resetErrors() {
  bool all_reset = true;

  for (uint8_t i = 0; i < 3; i++) {
    if (!drives_[i].resetError()) {
      all_reset = false;
    }
  }

  if (all_reset && state_ == ERROR) {
    state_ = IDLE;
    Logger::info("All errors reset");
  }

  return all_reset;
}

bool DrivesController::isHomingComplete() const {
  return (state_ != HOMING);
}

bool DrivesController::isAllHomed() const {
  for (uint8_t i = 0; i < 3; i++) {
    if (!drives_[i].isHomed()) {
      return false;
    }
  }
  return true;
}

void DrivesController::getPositions(float positions[3]) const {
  for (uint8_t i = 0; i < 3; i++) {
    positions[i] = drives_[i].getPosition();
  }
}

void DrivesController::getVelocities(float velocities[3]) const {
  for (uint8_t i = 0; i < 3; i++) {
    velocities[i] = drives_[i].getVelocity();
  }
}

void DrivesController::getTargetPositions(float targets[3]) const {
  for (uint8_t i = 0; i < 3; i++) {
    targets[i] = drives_[i].getTargetPosition();
  }
}

Drive::State DrivesController::getDriveState(uint8_t index) const {
if (index >= 3) return Drive::STATE_ERROR;
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
  for (uint8_t i = 0; i < 3; i++) {
    if (drives_[i].isMoving()) {
      return true;
    }
  }
  return false;
}

void DrivesController::processCommandQueue() {
  if (command_in_progress_) {
    // Проверяем завершение текущей команды
    if (state_ == IDLE || state_ == ERROR) {
      command_in_progress_ = false;

      Logger::debug("Command completed, state: %d", state_);
    } else if (current_command_.timeout > 0 &&
               millis() - command_start_time_ > current_command_.timeout) {
      // Таймаут команды
      Logger::warning("Command timeout after %d ms", current_command_.timeout);
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
  for (uint8_t i = 0; i < 3; i++) {
    if (!sync_drives_done_[i]) {
      all_done = false;
      break;
    }
  }

  if (all_done) {
    sync_in_progress_ = false;
    state_ = IDLE;

    Logger::info("Synchronized move completed successfully");

    // Проверяем точность синхронизации
    float positions[3], targets[3];
    getPositions(positions);
    getTargetPositions(targets);

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
index, old_state, new_state);

if (drive_state_callback_) {
drive_state_callback_(index, old_state, new_state, drive_state_callback_context_);
}

// Если привод перешел в состояние ошибки
if (new_state == Drive::STATE_ERROR && config_.stop_on_single_error) {
Logger::error("Drive %d error -> stopping all drives", index);
emergencyStop();
}
}

void DrivesController::onDrivePositionUpdated(uint8_t index,
float position,
float velocity) {
// Можно использовать для мониторинга в реальном времени
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
if (drives_[i].getState() == Drive::STATE_ERROR) {
any_failed = true;
}
}

if (all_homed) {
state_ = IDLE;
Logger::info("All drives homed successfully");

if (homing_complete_callback_) {
homing_complete_callback_(true, homing_complete_callback_context_);
}
} else if (any_failed) {
state_ = ERROR;
Logger::error("Homing failed for one or more drives");

if (homing_complete_callback_) {
homing_complete_callback_(false, homing_complete_callback_context_);
}
}
}

bool DrivesController::checkDrivesReady() const {
  for (uint8_t i = 0; i < 3; i++) {
    Drive::State state = drives_[i].getState();
    if (state == Drive::STATE_ERROR || state == Drive::STATE_LIMIT_TRIGGERED) {
      Logger::warning("Drive not ready (state: %d)", state);
      return false;
    }
  }

  if (!isAllHomed()) {
    Logger::warning("Drives not homed");
    return false;
  }

  return true;
}

bool DrivesController::checkPositionsValid(const float positions[3]) const {
  for (uint8_t i = 0; i < 3; i++) {
    // Проверка на NaN и бесконечность
    if (isnan(positions[i]) || isinf(positions[i])) {
      Logger::error("Invalid position for drive %d: %f", i, positions[i]);
      return false;
    }
  }

  return true;
}

bool DrivesController::checkVelocitiesValid(const float velocities[3]) const {
  for (uint8_t i = 0; i < 3; i++) {
    if (isnan(velocities[i]) || isinf(velocities[i])) {
      Logger::error("Invalid velocity for drive %d: %f", i, velocities[i]);
      return false;
    }
  }

  return true;
}

void DrivesController::updateControllerState() {
  // Определяем состояние контроллера на основе состояния приводов
  if (state_ == EMERGENCY_STOP || state_ == ERROR) {
    return; // Эти состояния устанавливаются явно
  }

  // Проверяем, есть ли приводы в состоянии ошибки
  for (uint8_t i = 0; i < 3; i++) {
    if (drives_[i].getState() == Drive::STATE_ERROR) {
      state_ = ERROR;
      return;
    }
  }

  // Проверяем, движется ли хотя бы один привод
  bool any_moving = false;
  for (uint8_t i = 0; i < 3; i++) {
    if (drives_[i].isMoving()) {
      any_moving = true;
      break;
    }
  }

  if (any_moving) {
    if (sync_in_progress_) {
      state_ = SYNCING;
    } else {
      state_ = MOVING;
    }
  } else {
    // Проверяем, выполняется ли homing
    bool any_homing = false;
    for (uint8_t i = 0; i < 3; i++) {
      if (drives_[i].getState() == Drive::STATE_HOMING_IN_PROGRESS) {
        any_homing = true;
        break;
      }
    }

    if (any_homing) {
      state_ = HOMING;
    } else {
      state_ = IDLE;
    }
  }
}

// Callback setters
void DrivesController::setStateChangeCallback(StateChangeCallback callback, void* context) {
  state_change_callback_ = callback;
  state_change_callback_context_ = context;
}

void DrivesController::setHomingCompleteCallback(HomingCompleteCallback callback, void* context) {
  homing_complete_callback_ = callback;
  homing_complete_callback_context_ = context;
}

void DrivesController::setCommandCompleteCallback(CommandCompleteCallback callback, void* context) {
  command_complete_callback_ = callback;
  command_complete_callback_context_ = context;
}

void DrivesController::setDriveStateCallback(DriveStateCallback callback, void* context) {
  drive_state_callback_ = callback;
  drive_state_callback_context_ = context;
}

void DrivesController::printStatus() const {
  float positions[3], velocities[3];
  getPositions(positions);
  getVelocities(velocities);

  Logger::info("=== DrivesController Status ===");
  Logger::info("State: %d, Mode: %d", state_, mode_);
  Logger::info("Commands: %d executed, %d failed", commands_executed_, commands_failed_);
  Logger::info("Queue size: %d", command_queue_.size());
  Logger::info("Sync in progress: %s", sync_in_progress_ ? "YES" : "NO");
  Logger::info("All homed: %s", isAllHomed() ? "YES" : "NO");

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
