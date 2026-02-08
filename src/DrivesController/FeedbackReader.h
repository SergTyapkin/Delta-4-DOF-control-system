#pragma once

#include <cstdint>
#include <functional>
#include "../../config/pins_config.h"

class FeedbackReader {
public:
  // Типы обратной связи
  enum class FeedbackType {
    NONE,           // Нет обратной связи
    ENCODER,        // Инкрементальный энкодер
    ABSOLUTE,       // Абсолютный энкодер
    HALL,           // Датчики Холла
    POTENTIOMETER,  // Потенциометр
    CURRENT_SENSE   // Измерение тока
  };

  // Конфигурация
  struct Config {
    FeedbackType type;
    uint8_t pin_a;          // Фаза A энкодера или аналоговый вход
    uint8_t pin_b;          // Фаза B энкодера
    uint8_t pin_index;      // Индексный импульс (опционально)
    uint32_t pulses_per_rev; // Импульсов на оборот
    float voltage_min;      // Минимальное напряжение (для аналоговых)
    float voltage_max;      // Максимальное напряжение (для аналоговых)
    bool invert_direction;  // Инвертировать направление

    Config() :
        type(FeedbackType::NONE),
        pin_a(0),
        pin_b(0),
        pin_index(0),
        pulses_per_rev(1024),
        voltage_min(0.0f),
        voltage_max(3.3f),
        invert_direction(false) {}
  };

  // Данные обратной связи
  struct FeedbackData {
    int32_t raw_count;      // Сырое значение счетчика
    float position;         // Позиция (радианы)
    float velocity;         // Скорость (рад/с)
    float current;          // Ток (А) - если измеряется
    uint32_t timestamp;     // Время последнего обновления
    bool valid;             // Данные валидны
    uint16_t error_flags;   // Флаги ошибок

    FeedbackData() :
        raw_count(0),
        position(0),
        velocity(0),
        current(0),
        timestamp(0),
        valid(false),
        error_flags(0) {}
  };

  // Конструктор
  FeedbackReader();

  // Инициализация
  bool init(const Config& config, uint8_t reader_id = 0);

  // Обновление состояния (вызывается периодически или по прерыванию)
  void update();

  // Обновление по прерыванию (для энкодеров)
  void handleInterrupt();

  // Получение данных
  const FeedbackData& getData() const { return data_; }
  float getPosition() const { return data_.position; }
  float getVelocity() const { return data_.velocity; }
  float getCurrent() const { return data_.current; }
  bool isValid() const { return data_.valid; }

  // Калибровка
  void calibrateZero();
  void setZeroPosition(float position);

  // Настройка фильтров
  void setVelocityFilter(float time_constant); // Постоянная времени фильтра (с)
  void setCurrentLimit(float max_current);     // Лимит тока для защиты

  // Callback'и
  typedef std::function<void(const FeedbackData&)> DataUpdateCallback;
  typedef std::function<void(uint16_t)> ErrorCallback;

  void setDataUpdateCallback(DataUpdateCallback callback);
  void setErrorCallback(ErrorCallback callback);

  // Диагностика
  void printStatus() const;

private:
  // Конфигурация
  Config config_;
  uint8_t reader_id_;

  // Состояние
  FeedbackData data_;
  FeedbackData previous_data_;

  // Для энкодеров
  volatile int32_t encoder_count_;
  volatile uint32_t last_encoder_time_;
  uint8_t last_encoder_state_;

  // Для аналоговых датчиков
  uint16_t analog_raw_;
  uint16_t analog_zero_;

  // Фильтры
  float velocity_filter_alpha_;
  float filtered_velocity_;

  // Callback'и
  DataUpdateCallback data_update_callback_;
  ErrorCallback error_callback_;

  // Приватные методы
  void updateEncoder();
  void updateAnalog();
  void updateVelocity();

  // Обработка прерываний для энкодера
  static void encoderISR(void* context);

  // Вспомогательные методы
  float rawToPosition(int32_t raw) const;
  int32_t positionToRaw(float position) const;
  float analogToPosition(uint16_t analog) const;
  float analogToCurrent(uint16_t analog) const;

  // Проверка ошибок
  void checkForErrors();
  bool checkSignalQuality() const;
  bool checkCurrentLimit() const;
};
