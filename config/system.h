#pragma once

namespace System {
  // Выводить ли логи в последовательный порт
  constexpr bool SERIAL_LOGS = true;
  // Выводить ли управление моторами последовательный порт (для эмулятора)
  constexpr bool SERIAL_CONTROLS = true;
  // Выводить ли команды в serial
  constexpr bool UI_ECHO_COMMANDS_TO_SERIAL = true;
  // Выводить ли полное состояние в serial
  constexpr bool UI_DEBUG_TO_SERIAL = true;

  // Периоды обновления подсистем (микросекунды)
  constexpr bool INTERVAL_UPDATE_SHEDULER_MCS = 1;
  constexpr bool INTERVAL_UPDATE_DRIVES_MCS = 2;
  constexpr bool INTERVAL_UPDATE_KINEMATICS_MCS = 10;
  constexpr bool INTERVAL_UPDATE_TRAJECTORY_MCS = 10;
  constexpr bool INTERVAL_UPDATE_CORE_MCS = 20;
  constexpr bool INTERVAL_UPDATE_UI_MCS = 50;
  constexpr bool INTERVAL_UPDATE_MONITORING_MCS = 100;
};
