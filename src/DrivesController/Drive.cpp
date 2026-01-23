#include "Drive.h"
#include <Arduino.h>
#include "utils/Logger.h"
#include "utils/MathUtils.h"
#include <cmath>

// Макросы для быстрого управления пинами
#define SET_PIN(pin, value) digitalWrite(pin, value)
#define READ_PIN(pin) digitalRead(pin)
#define PIN_MODE(pin, mode) pinMode(pin, mode)

Drive::Drive() :
    drive_id_(0),
    state_(State::IDLE),
    previous_state_(State::IDLE),
    mode_(Mode::DISABLED),
    current_position_(0),
    current_velocity_(0),
    target_position_(0),
    is_homing_(false),
    is_homed_(false),
    homing_direction_(HomingDirection::NEGATIVE),
    homing_start_time_(0),
    min_position_(-MathUtils::PI),
    max_position_(MathUtils::PI),
    enabled_(false),
    direction_(false),
    step_interval_(0),
    last_step_time_(0),
    steps_to_execute_(0),
    steps_executed_(0) {

  current_profile_ = {0};
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

  // Сбрасываем шаговый пин
  SET_PIN(config_.step_pin, LOW);

  // Рассчитываем производные параметры
  float steps_per_radian = (config_.steps_per_revolution * config_.microsteps *
                            config_.gear_ratio) / (2.0f * MathUtils::PI);

  Logger::info("Drive %d initialized:", drive_id_);
  Logger::info("  Steps/rev: %.0f, Microsteps: %.0f, Gear ratio: %.2f",
               config_.steps_per_revolution, config_.microsteps, config_.gear_ratio);
  Logger::info("  Steps/rad: %.2f", steps_per_radian);
  Logger::info("  Max velocity: %.2f rad/s (%.1f deg/s)",
               config_.max_velocity, MathUtils::toDegrees(config_.max_velocity));

  return true;
}

void Drive::update(uint32_t delta_time_ms) {
  // Сохраняем предыдущее состояние для callback'ов
  previous_state_ = state_;

  // Проверка ошибок драйвера
  if (config_.fault_pin != 0 && readFaultPin()) {
    if (state_ != State::ERROR) {
      Logger::error("Drive %d: Driver fault detected", drive_id_);
      state_ = State::ERROR;
      disable();
    }
    return;
  }

  // Проверка концевиков
  if (readLimitSwitch()) {
    if (state_ != State::LIMIT_TRIGGERED) {
      Logger::warning("Drive %d: Limit switch triggered", drive_id_);
      state_ = State::LIMIT_TRIGGERED;

      // Если это не homing, останавливаем двигатель
      if (!is_homing_) {
        emergencyStop();
      }
    }
  } else if (state_ == State::LIMIT_TRIGGERED) {
    state_ = previous_state_;
  }

  // Обработка в зависимости от режима
  switch (mode_) {
    case Mode::DISABLED:
      state_ = State::IDLE;
      break;

    case Mode::HOMING:
      handleHoming(delta_time_ms);
      break;

    case Mode::POSITION:
      handlePositionControl(delta_time_ms);
      break;

    case Mode::VELOCITY:
      handleVelocityControl(delta_time_ms);
      break;

    case Mode::CALIBRATING:
      // Режим калибровки - специфичная логика
      break;

    case Mode::TORQUE:
      // Управление моментом - если поддерживается
      break;
  }

  // Вызов callback'а при изменении состояния
  if (state_ != previous_state_ && state_change_callback_) {
    state_change_callback_(previous_state_, state_);
  }

  // Обновление генератора шагов
  if (state_ == State::MOVING || (is_homing_ && state_ != State::LIMIT_TRIGGERED)) {
    updateStepGenerator();
  }
}

void Drive::enable() {
  if (!enabled_) {
    SET_PIN(config_.enable_pin, LOW); // Активный низкий уровень для большинства драйверов
    enabled_ = true;
    Logger::debug("Drive %d: Enabled", drive_id_);
  }
}

void Drive::disable() {
  if (enabled_) {
    SET_PIN(config_.enable_pin, HIGH);
    enabled_ = false;
    current_velocity_ = 0;
    state_ = State::IDLE;
    Logger::debug("Drive %d: Disabled", drive_id_);
  }
}

void Drive::setMode(Mode mode) {
  if (mode_ == mode) return;

  Mode old_mode = mode_;
  mode_ = mode;

  Logger::debug("Drive %d: Mode changed from %d to %d",
                drive_id_, static_cast<int>(old_mode), static_cast<int>(mode));

  switch (mode_) {
    case Mode::DISABLED:
      disable();
      break;

    case Mode::HOMING:
      enable();
      break;

    case Mode::POSITION:
    case Mode::VELOCITY:
      enable();
      break;

    default:
      break;
  }
}

bool Drive::moveToPosition(float position_rad, float velocity) {
  if (mode_ != Mode::POSITION) {
    setMode(Mode::POSITION);
  }

  // Применяем компенсацию люфта
  position_rad = applyBacklashCompensation(position_rad);

  // Проверка пределов
  if (!checkLimits(position_rad)) {
    Logger::warning("Drive %d: Target position %.3f rad out of limits",
                    drive_id_, position_rad);
    return false;
  }

  // Устанавливаем параметры движения
  motion_params_.target_position = position_rad;

  if (velocity > 0) {
    motion_params_.target_velocity = MathUtils::clamp(
        velocity, 0.0f, config_.max_velocity);
  } else {
    motion_params_.target_velocity = config_.max_velocity;
  }

  motion_params_.acceleration = config_.max_acceleration;
  motion_params_.jerk = config_.max_jerk;

  // Рассчитываем профиль скорости
  float distance = position_rad - current_position_;
  calculateTrapezoidalProfile(
      fabs(distance),
      motion_params_.target_velocity,
      motion_params_.acceleration,
      motion_params_.acceleration // Используем то же ускорение для замедления
  );

  // Устанавливаем направление
  direction_ = (distance >= 0);
  SET_PIN(config_.dir_pin, direction_ ^ config_.invert_direction);

  // Сбрасываем счетчики шагов
  steps_executed_ = 0;
  steps_to_execute_ = radiansToSteps(fabs(distance));

  // Переходим в состояние движения
  state_ = State::MOVING;

  Logger::debug("Drive %d: Moving to %.3f rad (%.1f deg), distance: %.3f rad",
                drive_id_, position_rad, MathUtils::toDegrees(position_rad), distance);

  return true;
}

bool Drive::moveToPositionRelative(float delta_rad, float velocity) {
  return moveToPosition(current_position_ + delta_rad, velocity);
}

bool Drive::setVelocity(float velocity_rad_s) {
  if (mode_ != Mode::VELOCITY) {
    setMode(Mode::VELOCITY);
  }

  // Проверка скорости
  if (!checkVelocity(velocity_rad_s)) {
    Logger::warning("Drive %d: Velocity %.3f rad/s out of limits",
                    drive_id_, velocity_rad_s);
    return false;
  }

  motion_params_.target_velocity = velocity_rad_s;
  current_velocity_ = velocity_rad_s;

  // Устанавливаем направление
  direction_ = (velocity_rad_s >= 0);
  SET_PIN(config_.dir_pin, direction_ ^ config_.invert_direction);

  // Рассчитываем интервал шагов
  if (fabs(velocity_rad_s) > 0.001f) {
    step_interval_ = calculateStepInterval(fabs(velocity_rad_s));
    state_ = State::MOVING;
  } else {
    step_interval_ = 0;
    state_ = State::IDLE;
    current_velocity_ = 0;
  }

  Logger::debug("Drive %d: Velocity set to %.3f rad/s", drive_id_, velocity_rad_s);

  return true;
}

bool Drive::startHoming() {
  if (is_homing_) {
    Logger::warning("Drive %d: Homing already in progress", drive_id_);
    return false;
  }

  setMode(Mode::HOMING);

  // Сбрасываем флаг homing
  is_homed_ = false;
  is_homing_ = true;
  homing_start_time_ = millis();
  state_ = State::HOMING_IN_PROGRESS;

  // Устанавливаем параметры homing
  float homing_velocity = (config_.homing_velocity > 0) ?
                          config_.homing_velocity : config_.max_velocity * 0.3f;

  // Двигаемся в направлении homing
  switch (config_.homing_direction) {
    case HomingDirection::POSITIVE:
      direction_ = true;
      motion_params_.target_velocity = homing_velocity;
      break;

    case HomingDirection::NEGATIVE:
      direction_ = false;
      motion_params_.target_velocity = -homing_velocity;
      break;

    case HomingDirection::TO_LIMIT:
      // Определяем направление к ближайшему концевику
      // Для простоты - двигаемся в отрицательном направлении
      direction_ = false;
      motion_params_.target_velocity = -homing_velocity;
      break;
  }

  SET_PIN(config_.dir_pin, direction_ ^ config_.invert_direction);
  step_interval_ = calculateStepInterval(fabs(homing_velocity));

  Logger::info("Drive %d: Homing started (direction: %s)",
               drive_id_, direction_ ? "POSITIVE" : "NEGATIVE");

  return true;
}

void Drive::handleHoming(uint32_t delta_time_ms) {
  static uint32_t debounce_time = 0;
  static bool limit_was_triggered = false;

  // Проверка таймаута homing
  if (millis() - homing_start_time_ > 30000) { // 30 секунд таймаут
    Logger::error("Drive %d: Homing timeout", drive_id_);
    is_homing_ = false;
    emergencyStop();

    if (homing_complete_callback_) {
      homing_complete_callback_(false);
    }
    return;
  }

  // Проверка концевика
  if (readLimitSwitch()) {
    if (!limit_was_triggered) {
      limit_was_triggered = true;
      debounce_time = millis();
    }

    // Дебаунс концевика
    if (millis() - debounce_time > 50) { // 50 мс дебаунс
      // Останавливаем двигатель
      emergencyStop();

      // Сбрасываем позицию в ноль
      current_position_ = 0;
      target_position_ = 0;

      // Отъезжаем от концевика
      float backoff_velocity = config_.homing_velocity * 0.5f;
      float backoff_distance = 0.1f; // 0.1 радиана

      moveToPositionRelative(direction_ ? -backoff_distance : backoff_distance,
                             backoff_velocity);

      // Ждем завершения отъезда
      // (в реальности нужно дождаться завершения движения)

      is_homing_ = false;
      is_homed_ = true;
      state_ = State::IDLE;

      Logger::info("Drive %d: Homing complete", drive_id_);

      if (homing_complete_callback_) {
        homing_complete_callback_(true);
      }
    }
  } else {
    limit_was_triggered = false;
  }
}

void Drive::handlePositionControl(uint32_t delta_time_ms) {
  if (state_ != State::MOVING) return;

  // Обновляем текущую позицию на основе пройденных шагов
  float distance_traveled = stepsToRadians(steps_executed_) * (direction_ ? 1.0f : -1.0f);
  float new_position = current_position_ + distance_traveled;

  // Проверка достижения цели
  if (steps_executed_ >= steps_to_execute_) {
    current_position_ = target_position_;
    current_velocity_ = 0;
    state_ = State::IDLE;
    steps_executed_ = 0;

    Logger::debug("Drive %d: Position reached: %.3f rad",
                  drive_id_, current_position_);

    if (position_update_callback_) {
      position_update_callback_(current_position_, current_velocity_);
    }
    return;
  }

  // Обновляем скорость по трапецеидальному профилю
  updateVelocityProfile();

  // Обновляем текущую позицию
  current_position_ = new_position;

  if (position_update_callback_) {
    position_update_callback_(current_position_, current_velocity_);
  }
}

void Drive::handleVelocityControl(uint32_t delta_time_ms) {
  // Для режима скорости просто поддерживаем установленную скорость
  // Позиция непрерывно обновляется
  float delta_time_s = delta_time_ms / 1000.0f;
  float delta_position = current_velocity_ * delta_time_s;

  current_position_ += delta_position;

  // Проверка пределов
  if (!checkLimits(current_position_)) {
    Logger::warning("Drive %d: Velocity control hit limit at %.3f rad",
                    drive_id_, current_position_);
    emergencyStop();
  }

  if (position_update_callback_) {
    position_update_callback_(current_position_, current_velocity_);
  }
}

void Drive::updateStepGenerator() {
  if (step_interval_ == 0) return;

  uint32_t current_time = micros();
  uint32_t elapsed = current_time - last_step_time_;

  if (elapsed >= step_interval_) {
    generateStep();
    last_step_time_ = current_time;

    // Коррекция для компенсации дрифта
    if (elapsed > step_interval_ * 2) {
      last_step_time_ = current_time - step_interval_;
    }
  }
}

void Drive::generateStep() {
  // Генерация шага (импульс на STEP пине)
  SET_PIN(config_.step_pin, HIGH);
  delayMicroseconds(2); // Минимальная длительность импульса
  SET_PIN(config_.step_pin, LOW);

  steps_executed_++;

  // Для отладки можно вести счетчик шагов
  if (steps_executed_ % 1000 == 0) {
    Logger::debug("Drive %d: Steps executed: %d", drive_id_, steps_executed_);
  }
}

void Drive::calculateTrapezoidalProfile(float distance, float max_velocity,
                                        float acceleration, float deceleration) {
  // Рассчитываем расстояние, необходимое для разгона
  float acceleration_distance = (max_velocity * max_velocity) / (2.0f * acceleration);
  float deceleration_distance = (max_velocity * max_velocity) / (2.0f * deceleration);

  current_profile_.acceleration_distance = acceleration_distance;
  current_profile_.deceleration_distance = deceleration_distance;

  // Проверяем, будет ли участок постоянной скорости
  if (acceleration_distance + deceleration_distance <= distance) {
    // Трапецеидальный профиль
    current_profile_.cruise_distance = distance - acceleration_distance - deceleration_distance;
    current_profile_.is_trapezoidal = true;
    current_profile_.max_reached_velocity = max_velocity;

    current_profile_.acceleration_time = static_cast<uint32_t>(
        (max_velocity / acceleration) * 1000000.0f); // в микросекундах
    current_profile_.deceleration_time = static_cast<uint32_t>(
        (max_velocity / deceleration) * 1000000.0f);
    current_profile_.cruise_time = static_cast<uint32_t>(
        (current_profile_.cruise_distance / max_velocity) * 1000000.0f);
  } else {
    // Треугольный профиль (не хватает расстояния для выхода на макс. скорость)
    current_profile_.is_trapezoidal = false;

    // Находим максимальную достижимую скорость
    float max_reachable_velocity = sqrtf(
        (2.0f * acceleration * deceleration * distance) /
        (acceleration + deceleration)
    );

    current_profile_.max_reached_velocity = max_reachable_velocity;
    current_profile_.acceleration_distance =
        (max_reachable_velocity * max_reachable_velocity) / (2.0f * acceleration);
    current_profile_.deceleration_distance =
        (max_reachable_velocity * max_reachable_velocity) / (2.0f * deceleration);
    current_profile_.cruise_distance = 0;

    current_profile_.acceleration_time = static_cast<uint32_t>(
        (max_reachable_velocity / acceleration) * 1000000.0f);
    current_profile_.deceleration_time = static_cast<uint32_t>(
        (max_reachable_velocity / deceleration) * 1000000.0f);
    current_profile_.cruise_time = 0;
  }

  // Устанавливаем начальную скорость
  current_velocity_ = 0;
  step_interval_ = calculateStepInterval(0.001f); // Очень медленно для старта
}

void Drive::updateVelocityProfile() {
  if (steps_to_execute_ == 0) return;

  float progress = static_cast<float>(steps_executed_) / steps_to_execute_;
  float current_distance = progress * fabs(target_position_ - current_position_);

  float current_velocity = 0;

  if (current_profile_.is_trapezoidal) {
    if (current_distance < current_profile_.acceleration_distance) {
      // Фаза разгона
      current_velocity = sqrtf(2.0f * motion_params_.acceleration * current_distance);
    } else if (current_distance < current_profile_.acceleration_distance +
                                  current_profile_.cruise_distance) {
      // Фаза постоянной скорости
      current_velocity = current_profile_.max_reached_velocity;
    } else {
      // Фаза замедления
      float decel_distance = current_distance -
                             (current_profile_.acceleration_distance +
                              current_profile_.cruise_distance);
      current_velocity = sqrtf(current_profile_.max_reached_velocity *
                               current_profile_.max_reached_velocity -
                               2.0f * motion_params_.acceleration * decel_distance);
    }
  } else {
    // Треугольный профиль
    if (current_distance < current_profile_.acceleration_distance) {
      // Фаза разгона
      current_velocity = sqrtf(2.0f * motion_params_.acceleration * current_distance);
    } else {
      // Фаза замедления
      float decel_distance = current_distance - current_profile_.acceleration_distance;
      current_velocity = sqrtf(current_profile_.max_reached_velocity *
                               current_profile_.max_reached_velocity -
                               2.0f * motion_params_.acceleration * decel_distance);
    }
  }

  // Ограничиваем минимальную скорость
  if (current_velocity < 0.001f) current_velocity = 0.001f;

  // Обновляем интервал шагов
  step_interval_ = calculateStepInterval(current_velocity);
  current_velocity_ = current_velocity * (direction_ ? 1.0f : -1.0f);
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

float Drive::stepsToRadians(int32_t steps) const {
  float steps_per_radian = (config_.steps_per_revolution * config_.microsteps *
                            config_.gear_ratio) / (2.0f * MathUtils::PI);
  return steps / steps_per_radian;
}

int32_t Drive::radiansToSteps(float radians) const {
  float steps_per_radian = (config_.steps_per_revolution * config_.microsteps *
                            config_.gear_ratio) / (2.0f * MathUtils::PI);
  return static_cast<int32_t>(roundf(radians * steps_per_radian));
}

uint32_t Drive::calculateStepInterval(float velocity_rad_s) const {
  if (velocity_rad_s <= 0.001f) {
    return 0; // Остановка
  }

  float steps_per_second = velocity_rad_s *
                           (config_.steps_per_revolution * config_.microsteps * config_.gear_ratio) /
                           (2.0f * MathUtils::PI);

  if (steps_per_second <= 0) {
    return 0;
  }

  // Интервал в микросекундах между шагами
  return static_cast<uint32_t>(1000000.0f / steps_per_second);
}

bool Drive::readLimitSwitch() const {
  if (config_.limit_switch_pin == 0) return false;

  // Предполагаем, что концевик активен низким уровнем
  return (READ_PIN(config_.limit_switch_pin) == LOW);
}

bool Drive::readFaultPin() const {
  if (config_.fault_pin == 0) return false;

  // Предполагаем, что ошибка активна низким уровнем
  return (READ_PIN(config_.fault_pin) == LOW);
}

float Drive::applyBacklashCompensation(float position) {
  if (config_.backlash_compensation <= 0.0001f) {
    return position;
  }

  static float last_target_position = 0;
  static bool last_direction = true;

  bool current_direction = (position >= last_target_position);

  // Если направление изменилось, добавляем компенсацию люфта
  if (current_direction != last_direction) {
    float compensation = config_.backlash_compensation;
    position += current_direction ? compensation : -compensation;
  }

  last_target_position = position;
  last_direction = current_direction;

  return position;
}

void Drive::emergencyStop() {
  step_interval_ = 0;
  current_velocity_ = 0;
  state_ = State::IDLE;
  is_homing_ = false;

  // Для быстрой остановки можно добавить дополнительное торможение
  SET_PIN(config_.step_pin, LOW);

  Logger::warning("Drive %d: Emergency stop", drive_id_);
}

bool Drive::resetError() {
  if (state_ == State::ERROR) {
    state_ = State::IDLE;
    Logger::info("Drive %d: Error reset", drive_id_);
    return true;
  }
  return false;
}

void Drive::printDebugInfo() const {
  Logger::info("=== Drive %d Debug Info ===", drive_id_);
  Logger::info("State: %d, Mode: %d", static_cast<int>(state_), static_cast<int>(mode_));
  Logger::info("Position: %.3f rad (%.1f deg)",
               current_position_, MathUtils::toDegrees(current_position_));
  Logger::info("Velocity: %.3f rad/s", current_velocity_);
  Logger::info("Target: %.3f rad", target_position_);
  Logger::info("Homed: %s, Enabled: %s", is_homed_ ? "YES" : "NO", enabled_ ? "YES" : "NO");
  Logger::info("Limit switch: %s", readLimitSwitch() ? "TRIGGERED" : "OK");
  Logger::info("Steps: %d/%d", steps_executed_, steps_to_execute_);
}

// Callback setters
void Drive::setStateChangeCallback(StateChangeCallback callback) {
  state_change_callback_ = callback;
}

void Drive::setHomingCompleteCallback(HomingCompleteCallback callback) {
  homing_complete_callback_ = callback;
}

void Drive::setPositionUpdateCallback(PositionUpdateCallback callback) {
  position_update_callback_ = callback;
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

  // Здесь должна быть реализация установки тока через драйвер
  // (через ШИМ или цифровой интерфейс)
  Logger::debug("Drive %d: Current set to run=%.2fA, hold=%.2fA",
                drive_id_, run_current, hold_current);
}

void Drive::setBacklashCompensation(float backlash) {
  config_.backlash_compensation = backlash;
}
