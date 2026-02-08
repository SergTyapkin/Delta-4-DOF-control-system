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
      .step_pin          = 0, // PB0,   // Шаг
      .dir_pin           = 1, // PB1,   // Направление
      .enable_pin        = 2, // PB2,   // Разрешение
      .limit_switch_pin  = 0, // PA0,   // Концевик
      .fault_pin         = 3 // PA3    // Ошибка драйвера
  };

// Привод 2
  constexpr DrivePins DRIVE_2 = {
      .step_pin          = 6, // PB6,
      .dir_pin           = 7, // PB7,
      .enable_pin        = 8, // PB8,
      .limit_switch_pin  = 1, // PA1,
      .fault_pin         = 4 // PA4
  };

// Привод 3
  constexpr DrivePins DRIVE_3 = {
      .step_pin          = 8, // PA8,
      .dir_pin           = 9, // PA9,
      .enable_pin        = 10, // PA10,
      .limit_switch_pin  = 2, // PA2,
      .fault_pin         = 5 // PA5
  };

// Массив всех приводов для удобства итераций
  constexpr DrivePins ALL_DRIVES[] = {DRIVE_1, DRIVE_2, DRIVE_3};
constexpr uint8_t NUM_DRIVES = 3;

// ----------------------------------------------------------------------------
// Система безопасности
// ----------------------------------------------------------------------------
namespace Safety {
  constexpr uint8_t EMERGENCY_STOP_BUTTON = 13; // PC13;  // Синяя пользовательская кнопка
  constexpr uint8_t ESTOP_OUTPUT          = 5; // PB5;   // Выход для внешней аварийной цепи
  constexpr uint8_t BUZZER                = 9; // PB9;   // Зуммер для предупреждений
}

// ----------------------------------------------------------------------------
// Индикация состояния
// ----------------------------------------------------------------------------
namespace Status {
  constexpr uint8_t LED_READY    = 5; // PA5;   // Зеленый LED на Nucleo (LD2)
  constexpr uint8_t LED_MOVING   = 3; // PB3;   // Дополнительный LED (синий)
  constexpr uint8_t LED_ERROR    = 4; // PB4;   // Дополнительный LED (красный)
}

// ----------------------------------------------------------------------------
// Пользовательский интерфейс
// ----------------------------------------------------------------------------
namespace UI {
  constexpr uint8_t USER_BUTTON_1 = 0; // PC0;  // Дополнительные кнопки
  constexpr uint8_t USER_BUTTON_2 = 1; // PC1;
  constexpr uint8_t POTENTIOMETER = 6; // PA6;  // Потенциометр для ручного управления
}

// ----------------------------------------------------------------------------
// Коммуникации
// ----------------------------------------------------------------------------
namespace Comms {
  // UART1 уже используется Serial (PA9-TX, PA10-RX)
  constexpr uint8_t UART2_TX = 2; // PA2;
  constexpr uint8_t UART2_RX = 3; // PA3;

  // I2C для дополнительных датчиков
  constexpr uint8_t I2C1_SDA = 7; // PB7;
  constexpr uint8_t I2C1_SCL = 6; // PB6;

  // SPI для энкодеров или других устройств
  constexpr uint8_t SPI1_MOSI = 7; // PA7;
  constexpr uint8_t SPI1_MISO = 6; // PA6;
  constexpr uint8_t SPI1_SCK  = 5; // PA5;
  constexpr uint8_t SPI1_CS   = 4; // PA4;
}

// ----------------------------------------------------------------------------
// Аналоговые входы
// ----------------------------------------------------------------------------
namespace Analog {
  constexpr uint8_t CURRENT_SENSE_1 = 0; // PA0;  // Ток двигателя 1
  constexpr uint8_t CURRENT_SENSE_2 = 1; // PA1;  // Ток двигателя 2
  constexpr uint8_t CURRENT_SENSE_3 = 4; // PA4;  // Ток двигателя 3
  constexpr uint8_t TEMPERATURE     = 2; // PC2;  // Датчик температуры
  constexpr uint8_t VOLTAGE_12V     = 3; // PC3;  // Мониторинг питания
}

} // namespace Pins
