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
    analog_zero_(512),
    velocity_filter_alpha_(0.1f),
    filtered_velocity_(0) {

  data_ = FeedbackData();
  previous_data_ = FeedbackData();
}

bool FeedbackReader::init(const Config& config, uint8_t reader_id) {
  config_ = config;
  reader_id_ = reader_id;

  if (config_.type == FeedbackType::NONE) {
    Logger::info("FeedbackReader %d: No feedback configured", reader_id_);
    return true;
  }

  Logger::info("Initializing FeedbackReader %d (type: %d)",
               reader_id_, static_cast<int>(config_.type));

  switch (config_.type) {
    case FeedbackType::ENCODER:
      // Настройка пинов энкодера
      if (config_.pin_a == 0 || config_.pin_b == 0) {
        Logger::error("Encoder pins not configured");
        return false;
      }

      pinMode(config_.pin_a, INPUT_PULLUP);
      pinMode(config_.pin_b, INPUT_PULLUP);

      // Настройка прерываний
      attachInterrupt(digitalPinToInterrupt(config_.pin_a),
                      []() { /* Обработка в основном классе */ },
                      CHANGE);

      // Инициализация состояния
      last_encoder_state_ = (digitalRead(config_.pin_a) << 1) |
                            digitalRead(config_.pin_b);

      Logger::debug("Encoder initialized on pins %d, %d",
                    config_.pin_a, config_.pin_b);
      break;

    case FeedbackType::ABSOLUTE:
      // Абсолютные энкодеры обычно используют SPI/I2C
      // Здесь нужна специфичная реализация
      Logger::warning("Absolute encoder not fully implemented");
      break;

    case FeedbackType::POTENTIOMETER:
    case FeedbackType::CURRENT_SENSE:
      // Аналоговые входы
      if (config_.pin_a == 0) {
        Logger::error("Analog pin not configured");
        return false;
      }

      pinMode(config_.pin_a, INPUT_ANALOG);
      analog_zero_ = analogRead(config_.pin_a);

      Logger::debug("Analog feedback on pin %d", config_.pin_a);
      break;

    case FeedbackType::HALL:
      // Датчики Холла
      Logger::warning("Hall sensors not implemented");
      break;

    default:
      Logger::error("Unknown feedback type: %d",
                    static_cast<int>(config_.type));
      return false;
  }

  data_.valid = true;
  Logger::info("FeedbackReader %d initialized successfully", reader_id_);

  return true;
}

void FeedbackReader::update() {
  if (config_.type == FeedbackType::NONE) {
    return;
  }

  // Сохраняем предыдущие данные
  previous_data_ = data_;

  // Обновляем данные в зависимости от типа
  switch (config_.type) {
    case FeedbackType::ENCODER:
      updateEncoder();
      break;

    case FeedbackType::POTENTIOMETER:
      updateAnalog();
      data_.position = analogToPosition(analog_raw_);
      break;

    case FeedbackType::CURRENT_SENSE:
      updateAnalog();
      data_.current = analogToCurrent(analog_raw_);
      break;

    default:
      break;
  }

  // Обновляем скорость
  updateVelocity();

  // Проверяем ошибки
  checkForErrors();

  // Обновляем timestamp
  data_.timestamp = millis();

  // Вызываем callback если данные изменились
  if (data_update_callback_ &&
      (data_.raw_count != previous_data_.raw_count ||
       data_.position != previous_data_.position ||
       data_.velocity != previous_data_.velocity)) {
    data_update_callback_(data_);
  }
}

void FeedbackReader::handleInterrupt() {
  if (config_.type != FeedbackType::ENCODER) {
    return;
  }

  uint32_t current_time = micros();
  uint32_t delta_time = current_time - last_encoder_time_;

  // Читаем текущее состояние пинов
  uint8_t state_a = digitalRead(config_.pin_a);
  uint8_t state_b = digitalRead(config_.pin_b);
  uint8_t current_state = (state_a << 1) | state_b;

  // Определяем направление и обновляем счетчик
  // Таблица состояний для определения направления (квадратурный энкодер)
  static const int8_t state_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  uint8_t state_change = (last_encoder_state_ << 2) | current_state;
  int8_t direction = state_table[state_change & 0x0F];

  if (direction != 0) {
    if (config_.invert_direction) {
      direction = -direction;
    }

    encoder_count_ += direction;
    data_.raw_count = encoder_count_;

    // Обновляем время для расчета скорости
    if (delta_time > 0 && delta_time < 1000000) { // Защита от переполнения
      // Минимальная скорость (если тик был, но время очень большое)
      float min_dt = 0.0001f; // 0.1 мс
      float dt_s = std::max(delta_time / 1000000.0f, min_dt);

      // Угловое перемещение за тик
      float angle_per_tick = (2.0f * MathUtils::PI) / config_.pulses_per_rev;
      float delta_angle = angle_per_tick * direction;

      // Мгновенная скорость
      float instant_velocity = delta_angle / dt_s;

      // Применяем фильтр низких частот
      filtered_velocity_ = filtered_velocity_ * (1.0f - velocity_filter_alpha_) +
                           instant_velocity * velocity_filter_alpha_;

      data_.velocity = filtered_velocity_;
    }

    last_encoder_time_ = current_time;
  }

  last_encoder_state_ = current_state;
}

void FeedbackReader::updateEncoder() {
  // Для энкодера данные обновляются в прерываниях
  // Здесь только преобразуем raw count в позицию
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
      delta_position -= 2.0f * MathUtils::PI;
    } else {
      delta_position += 2.0f * MathUtils::PI;
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
    case FeedbackType::ENCODER:
      encoder_count_ = 0;
      data_.raw_count = 0;
      data_.position = 0;
      break;

    case FeedbackType::POTENTIOMETER:
      analog_zero_ = analog_raw_;
      break;

    default:
      break;
  }

  Logger::info("FeedbackReader %d: Zero calibrated", reader_id_);
}

void FeedbackReader::setZeroPosition(float position) {
  switch (config_.type) {
    case FeedbackType::ENCODER:
      encoder_count_ = positionToRaw(position);
      break;

    default:
      // Для других типов нужна специфичная реализация
      break;
  }
}

void FeedbackReader::setVelocityFilter(float time_constant) {
  if (time_constant <= 0) {
    velocity_filter_alpha_ = 1.0f; // Без фильтра
  } else {
    // alpha = dt / (dt + tau), где dt - период обновления
    // Предполагаем dt = 0.01 с (100 Гц)
    float dt = 0.01f;
    velocity_filter_alpha_ = dt / (dt + time_constant);
  }

  Logger::debug("Velocity filter alpha: %.3f", velocity_filter_alpha_);
}

void FeedbackReader::setCurrentLimit(float max_current) {
  // Для current sense
  // В реальности нужно конвертировать ток в аналоговое значение
  Logger::debug("Current limit set to %.2f A", max_current);
}

float FeedbackReader::rawToPosition(int32_t raw) const {
  if (config_.type != FeedbackType::ENCODER) {
    return 0.0f;
  }

  float revolutions = static_cast<float>(raw) / config_.pulses_per_rev;
  return revolutions * 2.0f * MathUtils::PI;
}

int32_t FeedbackReader::positionToRaw(float position) const {
  if (config_.type != FeedbackType::ENCODER) {
    return 0;
  }

  float revolutions = position / (2.0f * MathUtils::PI);
  return static_cast<int32_t>(revolutions * config_.pulses_per_rev);
}

float FeedbackReader::analogToPosition(uint16_t analog) const {
  if (config_.type != FeedbackType::POTENTIOMETER) {
    return 0.0f;
  }

  // Преобразование аналогового значения в угол
  // Предполагаем, что потенциометр покрывает 300 градусов
  float voltage = (analog / 4095.0f) * 3.3f; // Для 12-bit ADC
  float normalized = (voltage - config_.voltage_min) /
                     (config_.voltage_max - config_.voltage_min);

  normalized = MathUtils::clamp(normalized, 0.0f, 1.0f);

  return normalized * (300.0f * MathUtils::DEG_TO_RAD); // 300 градусов в радианы
}

float FeedbackReader::analogToCurrent(uint16_t analog) const {
  if (config_.type != FeedbackType::CURRENT_SENSE) {
    return 0.0f;
  }

  // Преобразование аналогового значения в ток
  // Предполагаем датчик тока с чувствительностью 100 мВ/А
  // и опорным напряжением 3.3В
  float voltage = (analog / 4095.0f) * 3.3f;

  // Смещение нуля (обычно половина опорного напряжения)
  float zero_offset = 1.65f; // 3.3В / 2

  // Преобразование напряжения в ток
  float current = (voltage - zero_offset) / 0.1f; // 100 мВ/А = 0.1 В/А

  return current;
}

void FeedbackReader::checkForErrors() {
  uint16_t errors = 0;

  if (!checkSignalQuality()) {
    errors |= 0x0001; // Плохое качество сигнала
  }

  if (config_.type == FeedbackType::CURRENT_SENSE &&
      !checkCurrentLimit()) {
    errors |= 0x0002; // Превышение тока
  }

  // Проверка таймаута обновления
  uint32_t time_since_update = millis() - data_.timestamp;
  if (time_since_update > 1000) { // 1 секунда
    errors |= 0x0004; // Таймаут
  }

  data_.error_flags = errors;

  if (errors != 0 && error_callback_) {
    error_callback_(errors);
  }
}

bool FeedbackReader::checkSignalQuality() const {
  // Проверка качества сигнала
  // Для энкодера: проверка что счетчик обновляется
  // Для аналоговых: проверка нахождения в допустимом диапазоне

  switch (config_.type) {
    case FeedbackType::ENCODER:
      // Проверяем, что счетчик меняется (если ожидается движение)
      // Упрощенная проверка
      return true;

    case FeedbackType::POTENTIOMETER:
    case FeedbackType::CURRENT_SENSE:
      // Проверяем аналоговое значение
      if (analog_raw_ == 0 || analog_raw_ == 4095) {
        return false; // Возможно обрыв или короткое замыкание
      }
      return true;

    default:
      return true;
  }
}

bool FeedbackReader::checkCurrentLimit() const {
  // Проверка превышения тока
  // В реальности нужно учитывать настройки лимита
  const float MAX_CURRENT = 2.0f; // А

  return fabs(data_.current) <= MAX_CURRENT;
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
    case FeedbackType::NONE: type_str = "NONE"; break;
    case FeedbackType::ENCODER: type_str = "ENCODER"; break;
    case FeedbackType::ABSOLUTE: type_str = "ABSOLUTE"; break;
    case FeedbackType::POTENTIOMETER: type_str = "POTENTIOMETER"; break;
    case FeedbackType::CURRENT_SENSE: type_str = "CURRENT_SENSE"; break;
    case FeedbackType::HALL: type_str = "HALL"; break;
  }

  Logger::info("Type: %s", type_str);
  Logger::info("Valid: %s", data_.valid ? "YES" : "NO");

  if (config_.type == FeedbackType::ENCODER) {
    Logger::info("Raw count: %ld", data_.raw_count);
  }

  Logger::info("Position: %.3f rad (%.1f deg)",
               data_.position, MathUtils::toDegrees(data_.position));
  Logger::info("Velocity: %.3f rad/s", data_.velocity);

  if (config_.type == FeedbackType::CURRENT_SENSE) {
    Logger::info("Current: %.2f A", data_.current);
  }

  if (data_.error_flags != 0) {
    Logger::warning("Error flags: 0x%04X", data_.error_flags);
  }
}
