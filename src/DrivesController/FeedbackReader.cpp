#include <Arduino.h>
#include "FeedbackReader.h"
#include "../../src/utils/Logger.h"
#include "../../src/utils/MathUtils.h"

FeedbackReader::FeedbackReader() :
    reader_id_(0),
    encoder_count_(0),
    last_encoder_time_(0),
    last_encoder_state_(0),
    analog_raw_(0),
    velocity_filter_alpha_(0.1f),
    filtered_velocity_(0),
    data_update_callback_(nullptr),
    error_callback_(nullptr) {
}

bool FeedbackReader::init(const Config& config, uint8_t reader_id) {
  config_ = config;
  reader_id_ = reader_id;

  if (config_.type == NONE) {
    return true;
  }

  Logger::info("Initializing FeedbackReader %d (type: %d)", reader_id_, config_.type);

  switch (config_.type) {
    case ENCODER:
      if (config_.pin_a == 0 || config_.pin_b == 0) {
        return false;
      }

      pinMode(config_.pin_a, INPUT_PULLUP);
      pinMode(config_.pin_b, INPUT_PULLUP);

      last_encoder_state_ = (digitalRead(config_.pin_a) << 1) | digitalRead(config_.pin_b);
      break;

    case POTENTIOMETER:
      if (config_.pin_a == 0) {
        return false;
      }
      pinMode(config_.pin_a, INPUT_ANALOG);
      break;

    default:
      return false;
  }

  data_.valid = true;
  return true;
}

void FeedbackReader::update() {
  if (config_.type == NONE) {
    return;
  }

  previous_data_ = data_;

  switch (config_.type) {
    case ENCODER:
      updateEncoder();
      break;

    case POTENTIOMETER:
      updateAnalog();
      // Простое преобразование 0-4095 → 0-2π
      data_.position = (analog_raw_ / 4095.0f) * 6.283185307f;
      break;

    default:
      break;
  }

  updateVelocity();
  checkForErrors();
  data_.timestamp = millis();

  if (data_update_callback_ &&
      (data_.raw_count != previous_data_.raw_count ||
       fabs(data_.position - previous_data_.position) > 0.001f)) {
    data_update_callback_(data_);
  }
}

void FeedbackReader::handleInterrupt() {
  if (config_.type != ENCODER) return;

  uint32_t current_time = micros();
  uint32_t delta_time = current_time - last_encoder_time_;

  uint8_t state_a = digitalRead(config_.pin_a);
  uint8_t state_b = digitalRead(config_.pin_b);
  uint8_t current_state = (state_a << 1) | state_b;

  static const int8_t state_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  uint8_t state_change = (last_encoder_state_ << 2) | current_state;
  int8_t direction = state_table[state_change & 0x0F];

  if (direction != 0) {
    if (config_.invert_direction) {
      direction = -direction;
    }

    encoder_count_ += direction;
    data_.raw_count = encoder_count_;

    if (delta_time > 0 && delta_time < 1000000) {
      float dt_s = delta_time / 1000000.0f > 0.0001f ? delta_time / 1000000.0f : 0.0001f;
      float angle_per_tick = MathUtils::TWO_PI / config_.pulses_per_rev;
      float delta_angle = angle_per_tick * direction;
      float instant_velocity = delta_angle / dt_s;

      filtered_velocity_ = filtered_velocity_ * (1.0f - velocity_filter_alpha_) +
                           instant_velocity * velocity_filter_alpha_;
      data_.velocity = filtered_velocity_;
    }

    last_encoder_time_ = current_time;
  }

  last_encoder_state_ = current_state;
}

void FeedbackReader::updateEncoder() {
  data_.position = rawToPosition(encoder_count_);
  data_.valid = true;
}

void FeedbackReader::updateAnalog() {
  analog_raw_ = analogRead(config_.pin_a);
  data_.valid = true;
}

void FeedbackReader::updateVelocity() {
  if (data_.timestamp == 0 || previous_data_.timestamp == 0) {
    return;
  }

  uint32_t dt_ms = data_.timestamp - previous_data_.timestamp;
  if (dt_ms == 0) {
    return;
  }

  float dt_s = dt_ms / 1000.0f;
  float delta_position = data_.position - previous_data_.position;

  // Учитываем переход через 0 (для круговых энкодеров)
  if (fabs(delta_position) > MathUtils::PI) {
    if (delta_position > 0) {
      delta_position -= MathUtils::TWO_PI;  // 2*PI
    } else {
      delta_position += MathUtils::TWO_PI;
    }
  }

  float instant_velocity = delta_position / dt_s;

  // Применяем фильтр
  filtered_velocity_ = filtered_velocity_ * (1.0f - velocity_filter_alpha_) +
                       instant_velocity * velocity_filter_alpha_;

  data_.velocity = filtered_velocity_;
}

void FeedbackReader::calibrateZero() {
  switch (config_.type) {
    case ENCODER:
      encoder_count_ = 0;
      data_.raw_count = 0;
      data_.position = 0;
      break;

    case POTENTIOMETER:
      // Для потенциометра сброс не нужен
      break;

    default:
      break;
  }

  Logger::info("FeedbackReader %d: Zero calibrated", reader_id_);
}

void FeedbackReader::setZeroPosition(float position) {
  if (config_.type == ENCODER) {
    encoder_count_ = positionToRaw(position);
  }
}

void FeedbackReader::setVelocityFilter(float time_constant) {
  if (time_constant <= 0) {
    velocity_filter_alpha_ = 1.0f;
  } else {
    float dt = 0.01f;  // 100 Гц
    velocity_filter_alpha_ = dt / (dt + time_constant);
  }
}

float FeedbackReader::rawToPosition(int32_t raw) const {
  if (config_.type != ENCODER) {
    return 0.0f;
  }

  float revolutions = static_cast<float>(raw) / config_.pulses_per_rev;
  return revolutions * MathUtils::TWO_PI;
}

int32_t FeedbackReader::positionToRaw(float position) const {
  if (config_.type != ENCODER) {
    return 0;
  }

  float revolutions = position / MathUtils::TWO_PI;
  return static_cast<int32_t>(revolutions * config_.pulses_per_rev);
}

void FeedbackReader::checkForErrors() {
  uint8_t errors = 0;

  // Проверка таймаута обновления
  uint32_t time_since_update = millis() - data_.timestamp;
  if (time_since_update > 1000) {
    errors |= 0x01;  // Таймаут
  }

  data_.error_flags = errors;

  if (errors != 0 && error_callback_) {
    error_callback_(errors);
  }
}

void FeedbackReader::setDataUpdateCallback(DataUpdateCallback callback) {
  data_update_callback_ = callback;
}

void FeedbackReader::setErrorCallback(ErrorCallback callback) {
  error_callback_ = callback;
}

void FeedbackReader::printStatus() const {
  Logger::info("=== FeedbackReader %d Status ===", reader_id_);

  const char* type_str = "UNKNOWN";
  switch (config_.type) {
    case NONE: type_str = "NONE"; break;
    case ENCODER: type_str = "ENCODER"; break;
    case POTENTIOMETER: type_str = "POTENTIOMETER"; break;
    default: break;
  }

  Logger::info("Type: %s", type_str);
  Logger::info("Valid: %s", data_.valid ? "YES" : "NO");

  if (config_.type == ENCODER) {
    Logger::info("Raw count: %ld", data_.raw_count);
  }

  Logger::info("Position: %.3f rad", data_.position);
  Logger::info("Velocity: %.3f rad/s", data_.velocity);

  if (data_.error_flags != 0) {
    Logger::warning("Error flags: 0x%02X", data_.error_flags);
  }
}
