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
  static void controls(const char* format, Args... args) {
    log(LEVEL_CONTROLS, "CONTROLS", format, args...);
  }

  template<typename... Args>
  static void critical(const char* format, Args... args) {
    log(LEVEL_CRITICAL, "CRITICAL", format, args...);
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
