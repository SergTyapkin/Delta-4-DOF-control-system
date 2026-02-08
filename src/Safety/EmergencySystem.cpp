#include "EmergencySystem.h"
#include <Arduino.h>
#include "../../src/utils/Logger.h"

// Константы
namespace {
  constexpr uint32_t VOLTAGE_CHECK_INTERVAL = 1000;  // мс
  constexpr uint32_t TEMP_CHECK_INTERVAL = 5000;     // мс
  constexpr uint32_t BUZZER_PATTERN_TIME = 200;      // мс

  constexpr float VOLTAGE_MIN = 10.5f;   // Минимальное напряжение 12В системы
  constexpr float VOLTAGE_MAX = 13.8f;   // Максимальное напряжение
  constexpr float TEMP_MAX = 60.0f;      // Максимальная температура (°C)

  // Коэффициент делителя напряжения (если используется)
  constexpr float VOLTAGE_DIVIDER_RATIO = 4.0f;  // R2/(R1+R2)
  constexpr float ADC_REFERENCE = 3.3f;          // Опорное напряжение АЦП
}

// Инициализация системы
void EmergencySystem::init() {
  Logger::info("Initializing Emergency System...");

  // Настройка пина аварийной кнопки
  pinMode(Pins::Safety::EMERGENCY_STOP_BUTTON, INPUT_PULLUP);

  // Настройка выхода аварийной цепи
  pinMode(Pins::Safety::ESTOP_OUTPUT, OUTPUT);
  digitalWrite(Pins::Safety::ESTOP_OUTPUT, HIGH); // Активный низкий уровень

  // Настройка зуммера
  pinMode(Pins::Safety::BUZZER, OUTPUT);
  digitalWrite(Pins::Safety::BUZZER, LOW);

  // Настройка светодиодов состояния
  pinMode(Pins::Status::LED_READY, OUTPUT);
  pinMode(Pins::Status::LED_MOVING, OUTPUT);
  pinMode(Pins::Status::LED_ERROR, OUTPUT);

  digitalWrite(Pins::Status::LED_READY, LOW);
  digitalWrite(Pins::Status::LED_MOVING, LOW);
  digitalWrite(Pins::Status::LED_ERROR, LOW);

  // Настройка концевиков
  limit_switches_[0] = {Pins::DRIVE_1.limit_switch_pin, true, true};
  limit_switches_[1] = {Pins::DRIVE_2.limit_switch_pin, true, true};
  limit_switches_[2] = {Pins::DRIVE_3.limit_switch_pin, true, true};

  for (auto& sw : limit_switches_) {
    pinMode(sw.pin, sw.active_low ? INPUT_PULLUP : INPUT_PULLDOWN);
  }

  // Настройка пинов ошибок драйверов
  pinMode(Pins::DRIVE_1.fault_pin, INPUT_PULLUP);
  pinMode(Pins::DRIVE_2.fault_pin, INPUT_PULLUP);
  pinMode(Pins::DRIVE_3.fault_pin, INPUT_PULLUP);

  Logger::info("Emergency System initialized");
}

// Основное обновление
void EmergencySystem::update() {
  // Быстрая проверка аварийной кнопки
  checkEmergencyButton();

  // Проверка концевиков
  checkLimitSwitches();

  // Проверка драйверов
  checkDriverFaults();

  // Медленные проверки (раз в определенный интервал)
  uint32_t current_time = millis();

  if (current_time - last_voltage_check_ > VOLTAGE_CHECK_INTERVAL) {
    checkVoltage();
    last_voltage_check_ = current_time;
  }

  static uint32_t last_temp_check = 0;
  if (current_time - last_temp_check > TEMP_CHECK_INTERVAL) {
    checkTemperature();
    last_temp_check = current_time;
  }

  // Обновление индикации
  updateStatusLEDs();
}

// Проверка аварийной кнопки
void EmergencySystem::checkEmergencyButton() {
  // Кнопка подключена с подтяжкой к VCC, нажатие = LOW
  bool estop_pressed = (digitalRead(Pins::Safety::EMERGENCY_STOP_BUTTON) == LOW);

  if (estop_pressed && state_ != State::EMERGENCY_STOP) {
    Logger::critical("Emergency stop button pressed!");
    enterEmergencyState(ErrorCode::ESTOP_BUTTON);
    activateBuzzer(3); // Три коротких сигнала
  }
}

// Проверка концевиков
void EmergencySystem::checkLimitSwitches() {
  for (int i = 0; i < 3; i++) {
    const auto& sw = limit_switches_[i];
    bool triggered = digitalRead(sw.pin) == (sw.active_low ? LOW : HIGH);

    if (triggered) {
      ErrorCode error;
      switch (i) {
        case 0: error = ErrorCode::LIMIT_SWITCH_1; break;
        case 1: error = ErrorCode::LIMIT_SWITCH_2; break;
        case 2: error = ErrorCode::LIMIT_SWITCH_3; break;
        default: return;
      }

      if (!hasError(error)) {
        Logger::warning("Limit switch %d triggered", i + 1);
        triggerEmergency(error);
      }
    }
  }
}

// Проверка ошибок драйверов
void EmergencySystem::checkDriverFaults() {
  bool fault1 = (digitalRead(Pins::DRIVE_1.fault_pin) == LOW);
  bool fault2 = (digitalRead(Pins::DRIVE_2.fault_pin) == LOW);
  bool fault3 = (digitalRead(Pins::DRIVE_3.fault_pin) == LOW);

  if (fault1 && !hasError(ErrorCode::DRIVER_FAULT_1)) {
    Logger::error("Driver 1 fault detected");
    triggerEmergency(ErrorCode::DRIVER_FAULT_1);
  }

  if (fault2 && !hasError(ErrorCode::DRIVER_FAULT_2)) {
    Logger::error("Driver 2 fault detected");
    triggerEmergency(ErrorCode::DRIVER_FAULT_2);
  }

  if (fault3 && !hasError(ErrorCode::DRIVER_FAULT_3)) {
    Logger::error("Driver 3 fault detected");
    triggerEmergency(ErrorCode::DRIVER_FAULT_3);
  }
}

// Проверка напряжения
void EmergencySystem::checkVoltage() {
  supply_voltage_ = readAnalogVoltage(Pins::Analog::VOLTAGE_12V);

  if (supply_voltage_ < VOLTAGE_MIN && !hasError(ErrorCode::UNDERVOLTAGE)) {
    Logger::error("Undervoltage detected: %.1fV", supply_voltage_);
    triggerEmergency(ErrorCode::UNDERVOLTAGE);
  }

  if (supply_voltage_ > VOLTAGE_MAX && !hasError(ErrorCode::OVERVOLTAGE)) {
    Logger::error("Overvoltage detected: %.1fV", supply_voltage_);
    triggerEmergency(ErrorCode::OVERVOLTAGE);
  }
}

// Проверка температуры
void EmergencySystem::checkTemperature() {
  float temp = readTemperature();

  if (temp > TEMP_MAX && !hasError(ErrorCode::OVERTEMPERATURE)) {
    Logger::error("Overtemperature detected: %.1fC", temp);
    triggerEmergency(ErrorCode::OVERTEMPERATURE);
  }
}

// Ручной вызов аварии
void EmergencySystem::triggerEmergency(ErrorCode error) {
  if (state_ != State::EMERGENCY_STOP) {
    enterEmergencyState(error);
  } else {
    // Добавляем ошибку к существующим
    error_code_ = static_cast<ErrorCode>(
        static_cast<uint16_t>(error_code_) | static_cast<uint16_t>(error)
    );
  }
}

// Вход в аварийное состояние
void EmergencySystem::enterEmergencyState(ErrorCode error) {
  state_ = State::EMERGENCY_STOP;
  error_code_ = static_cast<ErrorCode>(
      static_cast<uint16_t>(error_code_) | static_cast<uint16_t>(error)
  );
  emergency_time_ = millis();

  // Активируем аварийную цепь
  digitalWrite(Pins::Safety::ESTOP_OUTPUT, LOW);

  // Включаем красный светодиод
  digitalWrite(Pins::Status::LED_ERROR, HIGH);
  digitalWrite(Pins::Status::LED_READY, LOW);
  digitalWrite(Pins::Status::LED_MOVING, LOW);

  // Сигнал зуммером
  activateBuzzer(2);

  // Вызываем пользовательский callback если есть
  if (emergency_callback_) {
    emergency_callback_(error, callback_context_);
  }

  Logger::critical("EMERGENCY STOP: Error code 0x%04X", static_cast<uint16_t>(error));
}

// Сброс аварийного состояния
bool EmergencySystem::reset() {
  if (state_ != State::EMERGENCY_STOP) {
    return true; // Не в аварийном состоянии
  }

  // Проверяем, что все ошибки устранены
  bool estop_clear = (digitalRead(Pins::Safety::EMERGENCY_STOP_BUTTON) == HIGH);
  bool faults_clear = (digitalRead(Pins::DRIVE_1.fault_pin) == HIGH &&
                       digitalRead(Pins::DRIVE_2.fault_pin) == HIGH &&
                       digitalRead(Pins::DRIVE_3.fault_pin) == HIGH);
  bool voltage_ok = isVoltageInRange();

  if (estop_clear && faults_clear && voltage_ok) {
    state_ = State::RESETTING;
    Logger::info("Resetting emergency state...");

    // Сброс аварийной цепи
    digitalWrite(Pins::Safety::ESTOP_OUTPUT, HIGH);

    // Очистка ошибок (кроме аппаратных)
    error_code_ = ErrorCode::NONE;
    state_ = State::NORMAL;

    Logger::info("Emergency state reset successfully");
    return true;
  }

  Logger::warning("Cannot reset: conditions not met");
  return false;
}

// Обновление светодиодов состояния
void EmergencySystem::updateStatusLEDs() {
  switch (state_) {
    case State::NORMAL:
      digitalWrite(Pins::Status::LED_READY, HIGH);
      digitalWrite(Pins::Status::LED_ERROR, LOW);
      break;

    case State::WARNING:
      // Мигание зеленым
      digitalWrite(Pins::Status::LED_READY, (millis() % 500) < 250);
      break;

    case State::EMERGENCY_STOP:
      // Мигание красным быстро
      digitalWrite(Pins::Status::LED_ERROR, (millis() % 200) < 100);
      break;

    case State::RESETTING:
      // Мигание всеми светодиодами
      bool on = (millis() % 400) < 200;
      digitalWrite(Pins::Status::LED_READY, on);
      digitalWrite(Pins::Status::LED_ERROR, on);
      break;
  }
}

// Активация зуммера с паттерном
void EmergencySystem::activateBuzzer(uint8_t pattern) {
  for (int i = 0; i < pattern; i++) {
    digitalWrite(Pins::Safety::BUZZER, HIGH);
    delay(BUZZER_PATTERN_TIME);
    digitalWrite(Pins::Safety::BUZZER, LOW);
    if (i < pattern - 1) delay(BUZZER_PATTERN_TIME);
  }
}

// Чтение аналогового напряжения
float EmergencySystem::readAnalogVoltage(uint8_t pin) const {
  // Для Nucleo: 12-bit ADC, 0-3.3V
  int raw = analogRead(pin);
  float voltage = (raw * ADC_REFERENCE) / 4095.0f;

  // Учитываем делитель напряжения если используется
  return voltage * VOLTAGE_DIVIDER_RATIO;
}

// Чтение температуры (заглушка - нужно реализовать для конкретного датчика)
float EmergencySystem::readTemperature() const {
  // Заглушка - возвращаем комнатную температуру
  // В реальности: чтение с датчика через ADC или I2C
  return 25.0f;
}

// Проверка напряжения в допустимом диапазоне
bool EmergencySystem::isVoltageInRange() const {
  return (supply_voltage_ >= VOLTAGE_MIN && supply_voltage_ <= VOLTAGE_MAX);
}

// Установка callback для аварии
void EmergencySystem::setEmergencyCallback(EmergencyCallback callback, void* context) {
  emergency_callback_ = callback;
  callback_context_ = context;
}
