#pragma once

#include "pins_stm32_nucleo_names.h"

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
//    uint8_t enable_pin;
    uint8_t limit_switch_pin;  // Концевик для homing
    uint8_t fault_pin;         // Сигнал ошибки драйвера (опционально)
  };

// Привод 1
  constexpr DrivePins DRIVE_1 = {
      .step_pin          = PE_13, // Шаг
      .dir_pin           = PC_7, // Направление
//      .enable_pin        = 0,  // Разрешение
      .limit_switch_pin  = PC_2, // Концевик
      .fault_pin         = PD_7  // Ошибка драйвера
  };

// Привод 2
  constexpr DrivePins DRIVE_2 = {
      .step_pin          = PF_15, // Шаг
      .dir_pin           = PB_5, // Направление
//      .enable_pin        = 0,  // Разрешение
      .limit_switch_pin  = PF_4, // Концевик
      .fault_pin         = PD_7  // Ошибка драйвера
  };

// Привод 3
  constexpr DrivePins DRIVE_3 = {
      .step_pin          = PG_14, // Шаг
      .dir_pin           = PB_3, // Направление
//      .enable_pin        = 0,  // Разрешение
      .limit_switch_pin  = PB_6, // Концевик
      .fault_pin         = PD_7  // Ошибка драйвера
  };

// Привод 4
  constexpr DrivePins DRIVE_4 = {
      .step_pin          = PG_9,  // Шаг
      .dir_pin           = PA_4,  // Направление
//      .enable_pin        = 0,  // Разрешение
      .limit_switch_pin  = PB_2,  // Концевик
      .fault_pin         = PD_7   // Ошибка драйвера
  };

// ----------------------------------------------------------------------------
// Система безопасности
// ----------------------------------------------------------------------------
namespace Safety {
  constexpr uint8_t EMERGENCY_STOP_BUTTON = BUTTON_USER; // Синяя пользовательская кнопка
  constexpr uint8_t ESTOP_OUTPUT          = PC_8;  // Выход для внешней аварийной цепи
  constexpr uint8_t BUZZER                = PC_9;  // Зуммер для предупреждений
}

// ----------------------------------------------------------------------------
// Индикация состояния
// ----------------------------------------------------------------------------
namespace Status {
  constexpr uint8_t LED_READY    = LED_GREEN;  // Зеленый LED на Nucleo (LD2)
  constexpr uint8_t LED_MOVING   = LED_BLUE;  // Дополнительный LED (синий)
  constexpr uint8_t LED_ERROR    = LED_RED;  // Дополнительный LED (красный)
}

// ----------------------------------------------------------------------------
// Пользовательский интерфейс
// ----------------------------------------------------------------------------
//namespace UI {
//  constexpr uint8_t USER_BUTTON_1 = PC_10; // Дополнительная кнопка
//  constexpr uint8_t USER_BUTTON_2 = PC_11; // Дополнительная кнопка
//  constexpr uint8_t POTENTIOMETER = PC_12; // Потенциометр для ручного управления
//}

// ----------------------------------------------------------------------------
// Коммуникации
// ----------------------------------------------------------------------------
//namespace Comms {
//  // UART1 уже используется Serial (PA9-TX, PA10-RX)
//  constexpr uint8_t UART2_TX = 2;
//  constexpr uint8_t UART2_RX = 3;
//
//  // I2C для дополнительных датчиков
//  constexpr uint8_t I2C1_SDA = 7;
//  constexpr uint8_t I2C1_SCL = 6;
//
//  // SPI для энкодеров или других устройств
//  constexpr uint8_t SPI1_MOSI = 7;
//  constexpr uint8_t SPI1_MISO = 6;
//  constexpr uint8_t SPI1_SCK  = 5;
//  constexpr uint8_t SPI1_CS   = 4;
//}

// ----------------------------------------------------------------------------
// Аналоговые входы
// ----------------------------------------------------------------------------
//namespace Analog {
//  constexpr uint8_t CURRENT_SENSE_1 = 0; // PA0;  // Ток двигателя 1
//  constexpr uint8_t CURRENT_SENSE_2 = 1; // PA1;  // Ток двигателя 2
//  constexpr uint8_t CURRENT_SENSE_3 = 4; // PA4;  // Ток двигателя 3
//  constexpr uint8_t TEMPERATURE     = 2; // PC2;  // Датчик температуры
//  constexpr uint8_t VOLTAGE_12V     = 3; // PC3;  // Мониторинг питания
//}

} // namespace Pins
