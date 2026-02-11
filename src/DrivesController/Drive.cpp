#include <Arduino.h>
#include "Drive.h"
#include "../../src/utils/Logger.h"

// Макросы для быстрого управления пинами
#define SET_PIN(pin, value) digitalWrite(pin, value)
#define READ_PIN(pin) digitalRead(pin)
#define PIN_MODE(pin, mode) pinMode(pin, mode)

Drive::Drive() :
    drive_id_(0),
    state_(STATE_IDLE),
    previous_state_(STATE_IDLE),
    mode_(MODE_DISABLED),
    current_position_(0),
    current_velocity_(0),
    target_position_(0),
    is_homing_(false),
    is_homed_(false),
    homing_start_time_(0),
    min_position_(-3.1415926535f),  // -PI
    max_position_(3.1415926535f),   // PI
    enabled_(false),
    direction_(false),
    step_interval_(0),
    last_step_time_(0),
    steps_remaining_(0),
    profile_start_time_(0),
    state_change_callback_(nullptr),
    state_change_context_(nullptr),
    homing_complete_callback_(nullptr),
    homing_complete_context_(nullptr),
    position_update_callback_(nullptr),
    position_update_context_(nullptr) {

  current_profile_.acceleration_distance = 0;
  current_profile_.deceleration_distance = 0;
  current_profile_.cruise_distance = 0;
  current_profile_.max_reached_velocity = 0;
  current_profile_.acceleration_time = 0;
  current_profile_.deceleration_time = 0;
  current_profile_.cruise_time = 0;
  current_profile_.is_trapezoidal = false;
}

bool Drive::init(const Config& config, uint8_t drive_id) {
  config_ = config;
  drive_id_ = drive_id;

  // Проверка конфигурации
  if (config_.step_pin == 0 || config_.dir_pin == 0) {
    Logger::error("Drive %d: Invalid pin configuration", drive_id_);
    return false;
  }

  // Настройка пинов
  PIN_MODE(config_.step_pin, OUTPUT);
  PIN_MODE(config_.dir_pin, OUTPUT);
  PIN_MODE(config_.enable_pin, OUTPUT);
  PIN_MODE(config_.limit_switch_pin, INPUT_PULLUP);

  if (config_.fault_pin != 0) {
    PIN_MODE(config_.fault_pin, INPUT_PULLUP);
  }

  // Изначально отключаем двигатель
  disable();

  // Устанавливаем направление по умолчанию
  SET_PIN(config_.dir_pin, LOW);
  SET_PIN(config_.step_pin, LOW);

  Logger::info("Drive %d initialized", drive_id_);
  return true;
}

void Drive::update(uint32_t delta_time_ms) {
  // Сохраняем предыдущее состояние для callback'ов
  previous_state_ = state_;

  // Проверка ошибок драйвера
  if (config_.fault_pin != 0 && readFaultPin()) {
    if (state_ != STATE_ERROR) {
      Logger::error("Drive %d: Driver fault detected", drive_id_);
      state_ = STATE_ERROR;
      disable();
    }
    return;
  }

  // Проверка концевиков
  if (readLimitSwitch()) {
    if (state_ != STATE_LIMIT_TRIGGERED) {
      Logger::warning("Drive %d: Limit switch triggered", drive_id_);
      state_ = STATE_LIMIT_TRIGGERED;

      if (!is_homing_) {
        emergencyStop();
      }
    }
  } else if (state_ == STATE_LIMIT_TRIGGERED) {
    state_ = previous_state_;
  }

  // Обработка в зависимости от режима
  switch (mode_) {
    case MODE_DISABLED:
      state_ = STATE_IDLE;
      break;

    case MODE_HOMING:
      handleHoming(delta_time_ms);
      break;

    case MODE_POSITION:
      handlePositionControl(delta_time_ms);
      break;

    case MODE_VELOCITY:
      handleVelocityControl(delta_time_ms);
      break;

    default:
      break;
  }

  // Вызов callback'а при изменении состояния
  if (state_ != previous_state_ && state_change_callback_) {
    state_change_callback_(drive_id_, previous_state_, state_, state_change_context_);
  }

  // Обновление генератора шагов
  if (state_ == STATE_MOVING || (is_homing_ && state_ != STATE_LIMIT_TRIGGERED)) {
    updateStepGenerator();
  }
}

void Drive::enable() {
  if (!enabled_) {
    SET_PIN(config_.enable_pin, LOW);
    enabled_ = true;
    Logger::debug("Drive %d: Enabled", drive_id_);
  }
}

void Drive::disable() {
  if (enabled_) {
    SET_PIN(config_.enable_pin, HIGH);
    enabled_ = false;
    current_velocity_ = 0;
    state_ = STATE_IDLE;
    Logger::debug("Drive %d: Disabled", drive_id_);
  }
}

void Drive::setMode(Mode mode) {
  if (mode_ == mode) return;
  mode_ = mode;

  switch (mode_) {
    case MODE_DISABLED:
      disable();
      break;

    case MODE_HOMING:
    case MODE_POSITION:
    case MODE_VELOCITY:
      enable();
      break;

    default:
      break;
  }
}

bool Drive::moveToPosition(float position_rad, float velocity) {
  if (mode_ != MODE_POSITION) {
    setMode(MODE_POSITION);
  }

  // Применяем компенсацию люфта
  position_rad = applyBacklashCompensation(position_rad);

  // Проверка пределов
  if (!checkLimits(position_rad)) {
    Logger::warning("Drive %d: Target position out of limits", drive_id_);
    return false;
  }

  // Устанавливаем параметры движения
  motion_params_.target_position = position_rad;
  motion_params_.target_velocity = (velocity > 0) ?
                                   (velocity < config_.max_velocity ? velocity : config_.max_velocity) :
                                   config_.max_velocity;
  motion_params_.acceleration = config_.max_acceleration;

  // Рассчитываем профиль скорости
  float distance = position_rad - current_position_;
  calculateTrapezoidalProfile(
      fabs(distance),
      motion_params_.target_velocity,
      motion_params_.acceleration,
      motion_params_.acceleration
  );

  // Устанавливаем направление
  direction_ = (distance >= 0);
  SET_PIN(config_.dir_pin, direction_ ^ config_.invert_direction);

  // Сбрасываем счетчики шагов
  steps_remaining_ = radiansToSteps(fabs(distance));
  profile_start_time_ = micros();

  state_ = STATE_MOVING;
  return true;
}

bool Drive::setVelocity(float velocity_rad_s) {
  if (mode_ != MODE_VELOCITY) {
    setMode(MODE_VELOCITY);
  }

  if (!checkVelocity(velocity_rad_s)) {
    return false;
  }

  motion_params_.target_velocity = velocity_rad_s;
  current_velocity_ = velocity_rad_s;

  direction_ = (velocity_rad_s >= 0);
  SET_PIN(config_.dir_pin, direction_ ^ config_.invert_direction);

  if (fabs(velocity_rad_s) > 0.001f) {
    step_interval_ = calculateStepInterval(fabs(velocity_rad_s));
    state_ = STATE_MOVING;
  } else {
    step_interval_ = 0;
    state_ = STATE_IDLE;
    current_velocity_ = 0;
  }

  return true;
}

bool Drive::startHoming() {
  if (is_homing_) {
    return false;
  }

  setMode(MODE_HOMING);

  is_homed_ = false;
  is_homing_ = true;
  homing_start_time_ = millis();
  state_ = STATE_HOMING_IN_PROGRESS;

  float homing_velocity = (config_.homing_velocity > 0) ?
                          config_.homing_velocity : config_.max_velocity * 0.3f;

  switch (config_.homing_direction) {
    case HOMING_POSITIVE:
      direction_ = true;
      motion_params_.target_velocity = homing_velocity;
      break;

    case HOMING_NEGATIVE:
    case HOMING_TO_LIMIT:
      direction_ = false;
      motion_params_.target_velocity = -homing_velocity;
      break;
  }

  SET_PIN(config_.dir_pin, direction_ ^ config_.invert_direction);
  step_interval_ = calculateStepInterval(fabs(homing_velocity));

  Logger::info("Drive %d: Homing started", drive_id_);
  return true;
}

void Drive::handleHoming(uint32_t delta_time_ms) {
  static uint32_t debounce_time = 0;
  static bool limit_was_triggered = false;

  if (millis() - homing_start_time_ > 30000) {
    Logger::error("Drive %d: Homing timeout", drive_id_);
    is_homing_ = false;
    emergencyStop();

    if (homing_complete_callback_) {
      homing_complete_callback_(drive_id_, false, homing_complete_context_);
    }
    return;
  }

  if (readLimitSwitch()) {
    if (!limit_was_triggered) {
      limit_was_triggered = true;
      debounce_time = millis();
    }

    if (millis() - debounce_time > 50) {
      emergencyStop();

      current_position_ = 0;
      target_position_ = 0;

      is_homing_ = false;
      is_homed_ = true;
      state_ = STATE_IDLE;

      Logger::info("Drive %d: Homing complete", drive_id_);

      if (homing_complete_callback_) {
        homing_complete_callback_(drive_id_, true, homing_complete_context_);
      }
    }
  } else {
    limit_was_triggered = false;
  }
}

void Drive::handlePositionControl(uint32_t delta_time_ms) {
  if (state_ != STATE_MOVING) return;

  if (steps_remaining_ <= 0) {
    current_position_ = target_position_;
    current_velocity_ = 0;
    state_ = STATE_IDLE;

    if (position_update_callback_) {
      position_update_callback_(drive_id_, current_position_, current_velocity_,
                                position_update_context_);
    }
    return;
  }

  updateVelocityProfile();

  if (position_update_callback_) {
    position_update_callback_(drive_id_, current_position_, current_velocity_,
                              position_update_context_);
  }
}

void Drive::handleVelocityControl(uint32_t delta_time_ms) {
  float delta_time_s = delta_time_ms / 1000.0f;
  float delta_position = current_velocity_ * delta_time_s;

  current_position_ += delta_position;

  if (!checkLimits(current_position_)) {
    emergencyStop();
  }

  if (position_update_callback_) {
    position_update_callback_(drive_id_, current_position_, current_velocity_,
                              position_update_context_);
  }
}

void Drive::updateStepGenerator() {
  if (step_interval_ == 0) return;

  uint32_t current_time = micros();
  uint32_t elapsed = current_time - last_step_time_;

  if (elapsed >= step_interval_) {
    generateStep();
    last_step_time_ = current_time;
  }
}

void Drive::generateStep() {
  SET_PIN(config_.step_pin, HIGH);
  delayMicroseconds(2);
  SET_PIN(config_.step_pin, LOW);

  steps_remaining_--;
  float step_size = stepsToRadians(1) * (direction_ ? 1.0f : -1.0f);
  current_position_ += step_size;
  target_position_ = motion_params_.target_position;
}

void Drive::calculateTrapezoidalProfile(float distance, float max_velocity,
                                        float acceleration, float deceleration) {
  float accel_dist = (max_velocity * max_velocity) / (2.0f * acceleration);
  float decel_dist = (max_velocity * max_velocity) / (2.0f * deceleration);

  current_profile_.acceleration_distance = accel_dist;
  current_profile_.deceleration_distance = decel_dist;

  if (accel_dist + decel_dist <= distance) {
    current_profile_.cruise_distance = distance - accel_dist - decel_dist;
    current_profile_.is_trapezoidal = true;
    current_profile_.max_reached_velocity = max_velocity;

    current_profile_.acceleration_time = (uint32_t)((max_velocity / acceleration) * 1000000.0f);
    current_profile_.deceleration_time = (uint32_t)((max_velocity / deceleration) * 1000000.0f);
    current_profile_.cruise_time = (uint32_t)((current_profile_.cruise_distance / max_velocity) * 1000000.0f);
  } else {
    current_profile_.is_trapezoidal = false;

    float max_reachable = sqrtf((2.0f * acceleration * deceleration * distance) /
                                (acceleration + deceleration));

    current_profile_.max_reached_velocity = max_reachable;
    current_profile_.acceleration_distance = (max_reachable * max_reachable) / (2.0f * acceleration);
    current_profile_.deceleration_distance = (max_reachable * max_reachable) / (2.0f * deceleration);
    current_profile_.cruise_distance = 0;

    current_profile_.acceleration_time = (uint32_t)((max_reachable / acceleration) * 1000000.0f);
    current_profile_.deceleration_time = (uint32_t)((max_reachable / deceleration) * 1000000.0f);
    current_profile_.cruise_time = 0;
  }

  step_interval_ = calculateStepInterval(0.001f);
}

void Drive::updateVelocityProfile() {
  if (steps_remaining_ <= 0) return;

  uint32_t current_time = micros();
  uint32_t elapsed = current_time - profile_start_time_;

  float current_velocity = 0;

  if (elapsed < current_profile_.acceleration_time) {
    float t = elapsed / 1000000.0f;
    current_velocity = motion_params_.acceleration * t;
  }
  else if (elapsed < current_profile_.acceleration_time + current_profile_.cruise_time) {
    current_velocity = current_profile_.max_reached_velocity;
  }
  else if (elapsed < current_profile_.acceleration_time +
                     current_profile_.cruise_time +
                     current_profile_.deceleration_time) {
    float t = (elapsed - current_profile_.acceleration_time - current_profile_.cruise_time) / 1000000.0f;
    current_velocity = current_profile_.max_reached_velocity - (motion_params_.acceleration * t);
  }

  if (current_velocity < 0.001f) current_velocity = 0.001f;

  step_interval_ = calculateStepInterval(current_velocity);
  current_velocity_ = current_velocity * (direction_ ? 1.0f : -1.0f);
}

float Drive::stepsToRadians(int32_t steps) const {
  float steps_per_rad = (config_.steps_per_revolution * config_.microsteps *
                         config_.gear_ratio) / 6.283185307f;  // 2*PI
  return steps / steps_per_rad;
}

int32_t Drive::radiansToSteps(float radians) const {
  float steps_per_rad = (config_.steps_per_revolution * config_.microsteps *
                         config_.gear_ratio) / 6.283185307f;
  return (int32_t)(radians * steps_per_rad + 0.5f);
}

uint32_t Drive::calculateStepInterval(float velocity_rad_s) const {
  if (velocity_rad_s <= 0.001f) return 0;

  float steps_per_sec = velocity_rad_s *
                        (config_.steps_per_revolution * config_.microsteps * config_.gear_ratio) /
                        6.283185307f;

  return (steps_per_sec > 0) ? (uint32_t)(1000000.0f / steps_per_sec) : 0;
}

bool Drive::checkLimits(float position) const {
  return (position >= min_position_ && position <= max_position_);
}

bool Drive::checkVelocity(float velocity) const {
  return (fabs(velocity) <= config_.max_velocity);
}

bool Drive::checkAcceleration(float acceleration) const {
  return (acceleration >= 0 && acceleration <= config_.max_acceleration);
}

bool Drive::readLimitSwitch() const {
  if (config_.limit_switch_pin == 0) return false;
  return (READ_PIN(config_.limit_switch_pin) == LOW);
}

bool Drive::readFaultPin() const {
  if (config_.fault_pin == 0) return false;
  return (READ_PIN(config_.fault_pin) == LOW);
}

float Drive::applyBacklashCompensation(float position) {
  if (config_.backlash_compensation <= 0.0001f) return position;

  static float last_target = 0;
  static bool last_dir = true;

  bool current_dir = (position >= last_target);

  if (current_dir != last_dir) {
    position += current_dir ? config_.backlash_compensation : -config_.backlash_compensation;
  }

  last_target = position;
  last_dir = current_dir;
  return position;
}

void Drive::emergencyStop() {
  step_interval_ = 0;
  current_velocity_ = 0;
  state_ = STATE_IDLE;
  is_homing_ = false;
  SET_PIN(config_.step_pin, LOW);
  Logger::warning("Drive %d: Emergency stop", drive_id_);
}

bool Drive::resetError() {
  if (state_ == STATE_ERROR) {
    state_ = STATE_IDLE;
    Logger::info("Drive %d: Error reset", drive_id_);
    return true;
  }
  return false;
}

bool Drive::isHomingComplete() const {
  return !is_homing_ && is_homed_;
}

bool Drive::isLimitTriggered() {
  return readLimitSwitch();
}

void Drive::setLimits(float min_position, float max_position) {
  min_position_ = min_position;
  max_position_ = max_position;
}

void Drive::setCurrent(float run_current, float hold_current) {
  config_.run_current = run_current;
  config_.hold_current = hold_current;
}

void Drive::setBacklashCompensation(float backlash) {
  config_.backlash_compensation = backlash;
}

void Drive::setStateChangeCallback(StateChangeCallback callback, void* context) {
  state_change_callback_ = callback;
  state_change_context_ = context;
}

void Drive::setHomingCompleteCallback(HomingCompleteCallback callback, void* context) {
  homing_complete_callback_ = callback;
  homing_complete_context_ = context;
}

void Drive::setPositionUpdateCallback(PositionUpdateCallback callback, void* context) {
  position_update_callback_ = callback;
  position_update_context_ = context;
}

void Drive::printDebugInfo() const {
  Logger::info("=== Drive %d Debug Info ===", drive_id_);
  Logger::info("State: %d, Mode: %d", state_, mode_);
  Logger::info("Position: %.3f rad", current_position_);
  Logger::info("Velocity: %.3f rad/s", current_velocity_);
  Logger::info("Target: %.3f rad", target_position_);
  Logger::info("Homed: %s, Enabled: %s", is_homed_ ? "YES" : "NO", enabled_ ? "YES" : "NO");
  Logger::info("Steps remaining: %d", steps_remaining_);
}
