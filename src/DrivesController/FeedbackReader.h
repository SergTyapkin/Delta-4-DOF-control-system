#pragma once

#include <Arduino.h>  // вместо <cstdint> и <functional>

class FeedbackReader {
public:
  // Типы обратной связи (упрощаем enum class)
  enum FeedbackType {
    NONE,           // Нет обратной связи
    ENCODER,        // Инкрементальный энкодер
    ABSOLUTE,       // Абсолютный энкодер
    HALL,           // Датчики Холла
    POTENTIOMETER,  // Потенциометр
    CURRENT_SENSE   // Измерение тока
  };

  // Конфигурация (упрощаем)
  struct Config {
    FeedbackType type;
    uint8_t pin_a;          // Фаза A энкодера или аналоговый вход
    uint8_t pin_b;          // Фаза B энкодера (0 если не используется)
    uint32_t pulses_per_rev; // Импульсов на оборот
    bool invert_direction;  // Инвертировать направление

    Config() :
        type(NONE),
        pin_a(0),
        pin_b(0),
        pulses_per_rev(1024),
        invert_direction(false) {}
  };

  // Данные обратной связи (упрощаем)
  struct FeedbackData {
    int32_t raw_count;      // Сырое значение счетчика
    float position;         // Позиция (радианы)
    float velocity;         // Скорость (рад/с)
    uint32_t timestamp;     // Время последнего обновления
    bool valid;             // Данные валидны
    uint8_t error_flags;    // Флаги ошибок (уменьшаем с uint16_t)

    FeedbackData() :
        raw_count(0),
        position(0),
        velocity(0),
        timestamp(0),
        valid(false),
        error_flags(0) {}
  };

  // Указатели на функции вместо std::function
  typedef void (*DataUpdateCallback)(const FeedbackData&);
  typedef void (*ErrorCallback)(uint8_t);

  // Конструктор
  FeedbackReader();

  // Инициализация
  bool init(const Config& config, uint8_t reader_id = 0);

  // Обновление состояния
  void update();

  // Обновление по прерыванию (для энкодеров)
  void handleInterrupt();

  // Получение данных
  const FeedbackData& getData() const { return data_; }
  float getPosition() const { return data_.position; }
  float getVelocity() const { return data_.velocity; }
  bool isValid() const { return data_.valid; }

  // Калибровка
  void calibrateZero();
  void setZeroPosition(float position);

  // Настройка фильтров
  void setVelocityFilter(float time_constant);

  // Callback'и
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

  // Вспомогательные методы
  float rawToPosition(int32_t raw) const;
  int32_t positionToRaw(float position) const;

  // Проверка ошибок
  void checkForErrors();
};
