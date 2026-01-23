#pragma once

#include <Arduino.h>

class Logger {
public:
  enum class Level {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
  };

  static void init(Level min_level = Level::INFO) {
    min_level_ = min_level;
  }

  template<typename... Args>
  static void debug(const char* format, Args... args) {
    log(Level::DEBUG, format, args...);
  }

  template<typename... Args>
  static void info(const char* format, Args... args) {
    log(Level::INFO, format, args...);
  }

  template<typename... Args>
  static void warning(const char* format, Args... args) {
    log(Level::WARNING, format, args...);
  }

  template<typename... Args>
  static void error(const char* format, Args... args) {
    log(Level::ERROR, format, args...);
  }

  template<typename... Args>
  static void critical(const char* format, Args... args) {
    log(Level::CRITICAL, format, args...);
  }

private:
  static Level min_level_;

  template<typename... Args>
  static void log(Level level, const char* format, Args... args) {
    if (level < min_level_) return;

    const char* level_str = "";
    switch (level) {
      case Level::DEBUG:    level_str = "DEBUG"; break;
      case Level::INFO:     level_str = "INFO"; break;
      case Level::WARNING:  level_str = "WARNING"; break;
      case Level::ERROR:    level_str = "ERROR"; break;
      case Level::CRITICAL: level_str = "CRITICAL"; break;
    }

    char buffer[256];
    snprintf(buffer, sizeof(buffer), "[%s] ", level_str);
    Serial.print(buffer);

    char message[192];
    snprintf(message, sizeof(message), format, args...);
    Serial.println(message);
  }
};
