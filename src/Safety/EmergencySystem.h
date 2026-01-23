#pragma once

#include <cstdint>
#include "config/pins_config.h"

class EmergencySystem {
public:
  // Коды ошибок
  enum class ErrorCode : uint16_t {
    NONE                = 0x0000,
    ESTOP_BUTTON       = 0x0001,  // Нажата аварийная кнопка
    LIMIT_SWITCH_1     = 0x0002,  // Сработал концевик 1
    LIMIT_SWITCH_2     = 0x0004,  // Сработал концевик 2
    LIMIT_SWITCH_3     = 0x0008,  // Сработал концевик 3
    DRIVER_FAULT_1     = 0x0010,  // Ошибка драйвера 1
    DRIVER_FAULT_2     = 0x0020,  // Ошибка драйвера 2
    DRIVER_FAULT_3     = 0x0040,  // Ошибка драйвера 3
    OVERCURRENT_1      = 0x0080,  // Перегруз по току 1
    OVERCURRENT_2      = 0x0100,  // Перегруз по току 2
    OVERCURRENT_3      = 0x0200,  // Перегруз по току 3
    OVERTEMPERATURE    = 0x0400,  // Перегрев
    UNDERVOLTAGE       = 0x0800,  // Низкое напряжение
    OVERVOLTAGE        = 0x1000,  // Высокое напряжение
    KINEMATICS_ERROR   = 0x2000,  // Ошибка кинематики
    COMMUNICATION_LOST = 0x4000,  // Потеря связи
    WATCHDOG_TIMEOUT   = 0x8000   // Таймаут watchdog
  };

  // Состояния системы
  enum class State {
    NORMAL,           // Нормальная работа
    WARNING,          // Предупреждение
    SOFT_STOP,        // Программная остановка
    EMERGENCY_STOP,   // Аварийная остановка
    RESETTING         // Сброс после аварии
  };

  // Конфигурация концевиков
  struct LimitSwitchConfig {
    uint8_t pin;
    bool active_low;   // true если концевик замыкает на GND
    bool normally_open; // true для нормально-разомкнутых
  };

  // Инициализация системы
  void init();

  // Основной метод обновления (вызывается в высокоприоритетной задаче)
  void update();

  // Ручной вызов аварийной остановки
  void triggerEmergency(ErrorCode error);

  // Сброс аварийного состояния
  bool reset();

  // Проверка состояния
  bool isEmergencyActive() const { return state_ == State::EMERGENCY_STOP; }
  bool isNormal() const { return state_ == State::NORMAL; }
  State getState() const { return state_; }

  // Получение текущих ошибок
  uint16_t getErrorCode() const { return static_cast<uint16_t>(error_code_); }
  bool hasError(ErrorCode error) const {
    return (static_cast<uint16_t>(error_code_) & static_cast<uint16_t>(error)) != 0;
  }

  // Настройка пользовательского обработчика аварии
  typedef void (*EmergencyCallback)(ErrorCode, void*);
  void setEmergencyCallback(EmergencyCallback callback, void* context = nullptr);

  // Мониторинг напряжения
  float getSupplyVoltage() const { return supply_voltage_; }
  bool isVoltageInRange() const;

private:
  // Состояние системы
  State state_ = State::NORMAL;
  ErrorCode error_code_ = ErrorCode::NONE;

  // Конфигурация концевиков
  LimitSwitchConfig limit_switches_[3];

  // Пользовательский callback
  EmergencyCallback emergency_callback_ = nullptr;
  void* callback_context_ = nullptr;

  // Мониторинг напряжения
  float supply_voltage_ = 0.0f;
  uint32_t last_voltage_check_ = 0;

  // Время последней аварии
  uint32_t emergency_time_ = 0;

  // Приватные методы
  void checkEmergencyButton();
  void checkLimitSwitches();
  void checkDriverFaults();
  void checkVoltage();
  void checkTemperature();

  void enterEmergencyState(ErrorCode error);
  void updateStatusLEDs();
  void activateBuzzer(uint8_t pattern);

  // Аппаратно-зависимые методы
  float readAnalogVoltage(uint8_t pin) const;
  float readTemperature() const;
};
