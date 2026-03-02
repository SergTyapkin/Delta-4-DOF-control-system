#pragma once

#include <Arduino.h>
#include "../../config/system.h"

class Logger {
public:
  enum Level {
    LEVEL_DEBUG,
    LEVEL_INFO,
    LEVEL_WARNING,
    LEVEL_ERROR,
    LEVEL_CRITICAL,
    LEVEL_CONTROLS,
  };
  enum DriveEmulatorCmd {
    EMULATOR_CMD_STEP,
    EMULATOR_CMD_DIR,
    EMULATOR_CMD_ENABLE
  };

  static void init(Level min_level = LEVEL_INFO) {
    min_level_ = min_level;
  }

  template<typename... Args>
  static void debug(const char* format, Args... args) {
    log(LEVEL_DEBUG, "DEBUG", format, args...);
  }

  template<typename... Args>
  static void info(const char* format, Args... args) {
    log(LEVEL_INFO, "INFO", format, args...);
  }

  template<typename... Args>
  static void warning(const char* format, Args... args) {
    log(LEVEL_WARNING, "WARNING", format, args...);
  }

  template<typename... Args>
  static void error(const char* format, Args... args) {
    log(LEVEL_ERROR, "ERROR", format, args...);
  }

  template<typename... Args>
  static void critical(const char* format, Args... args) {
    log(LEVEL_CRITICAL, "CRITICAL", format, args...);
  }

  template<typename... Args>
  static void controls(DriveEmulatorCmd cmd, uint8_t drive_id, bool value) {
    const char* cmd_text;
    switch (cmd) {
      case EMULATOR_CMD_DIR:
        cmd_text = "DIR";
        break;
      case EMULATOR_CMD_STEP:
        cmd_text = "STEP";
        break;
      case EMULATOR_CMD_ENABLE:
        cmd_text = "ENABLE";
        break;
    }
    log(LEVEL_CONTROLS, "CONTROLS", "[%s] #%d %s", cmd_text, drive_id, value ? "true" : "false");
  }

private:
  static Level min_level_;

  template<typename... Args>
  static void log(Level level, const char* level_str, const char* format, Args... args) {
    if (level < min_level_) return;
    if (!System::SERIAL_LOGS && level < LEVEL_CONTROLS) return;
    if (!System::SERIAL_CONTROLS && level == LEVEL_CONTROLS) return;

    char buffer[128];
    snprintf(buffer, sizeof(buffer), "[%s] ", level_str);
    Serial.print(buffer);

    char message[96];
    snprintf(message, sizeof(message), format, args...);
    Serial.println(message);
  }
};
