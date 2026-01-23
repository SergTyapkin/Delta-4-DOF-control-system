// config/pins_config.h
#pragma once

#include <cstdint>

// ============================================================================
// Конфигурация пинов для STM32 Nucleo-F401RE
// ============================================================================

namespace Pins {

// ----------------------------------------------------------------------------
// Драйверы шаговых двигателей (3 оси)
// ----------------------------------------------------------------------------
  struct DrivePins {
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    uint8_t limit_switch_pin;  // Концевик для homing
    uint8_t fault_pin;         // Сигнал ошибки драйвера (опционально)
  };

// Привод 1
  constexpr DrivePins DRIVE_1 = {
      .step_pin          = PB0,   // Шаг
      .dir_pin           = PB1,   // Направление
      .enable_pin        = PB2,   // Разрешение
      .limit_switch_pin  = PA0,   // Концевик
      .fault_pin         = PA3    // Ошибка драйвера
  };

// Привод 2
  constexpr DrivePins DRIVE_2 = {
      .step_pin          = PB6,
      .dir_pin           = PB7,
      .enable_pin        = PB8,
      .limit_switch_pin  = PA1,
      .fault_pin         = PA4
  };

// Привод 3
  constexpr DrivePins DRIVE_3 = {
      .step_pin          = PA8,
      .dir_pin           = PA9,
      .enable_pin        = PA10,
      .limit_switch_pin  = PA2,
      .fault_pin         = PA5
  };

// Массив всех приводов для удобства итераций
  constexpr DrivePins ALL_DRIVES[] = {DRIVE_1, DRIVE_2, DRIVE_3};
constexpr uint8_t NUM_DRIVES = 3;

// ----------------------------------------------------------------------------
// Система безопасности
// ----------------------------------------------------------------------------
namespace Safety {
  constexpr uint8_t EMERGENCY_STOP_BUTTON = PC13;  // Синяя пользовательская кнопка
  constexpr uint8_t ESTOP_OUTPUT          = PB5;   // Выход для внешней аварийной цепи
  constexpr uint8_t BUZZER                = PB9;   // Зуммер для предупреждений
}

// ----------------------------------------------------------------------------
// Индикация состояния
// ----------------------------------------------------------------------------
namespace Status {
  constexpr uint8_t LED_READY    = PA5;   // Зеленый LED на Nucleo (LD2)
  constexpr uint8_t LED_MOVING   = PB3;   // Дополнительный LED (синий)
  constexpr uint8_t LED_ERROR    = PB4;   // Дополнительный LED (красный)
}

// ----------------------------------------------------------------------------
// Пользовательский интерфейс
// ----------------------------------------------------------------------------
namespace UI {
  constexpr uint8_t USER_BUTTON_1 = PC0;  // Дополнительные кнопки
  constexpr uint8_t USER_BUTTON_2 = PC1;
  constexpr uint8_t POTENTIOMETER = PA6;  // Потенциометр для ручного управления
}

// ----------------------------------------------------------------------------
// Коммуникации
// ----------------------------------------------------------------------------
namespace Comms {
  // UART1 уже используется Serial (PA9-TX, PA10-RX)
  constexpr uint8_t UART2_TX = PA2;
  constexpr uint8_t UART2_RX = PA3;

  // I2C для дополнительных датчиков
  constexpr uint8_t I2C1_SDA = PB7;
  constexpr uint8_t I2C1_SCL = PB6;

  // SPI для энкодеров или других устройств
  constexpr uint8_t SPI1_MOSI = PA7;
  constexpr uint8_t SPI1_MISO = PA6;
  constexpr uint8_t SPI1_SCK  = PA5;
  constexpr uint8_t SPI1_CS   = PA4;
}

// ----------------------------------------------------------------------------
// Аналоговые входы
// ----------------------------------------------------------------------------
namespace Analog {
  constexpr uint8_t CURRENT_SENSE_1 = PA0;  // Ток двигателя 1
  constexpr uint8_t CURRENT_SENSE_2 = PA1;  // Ток двигателя 2
  constexpr uint8_t CURRENT_SENSE_3 = PA4;  // Ток двигателя 3
  constexpr uint8_t TEMPERATURE     = PC2;  // Датчик температуры
  constexpr uint8_t VOLTAGE_12V     = PC3;  // Мониторинг питания
}

} // namespace Pins
