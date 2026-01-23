// src/UI/CommandParser.h
#pragma once

#include <Arduino.h>
#include <vector>

class CommandParser {
public:
  CommandParser();

  // Настройка парсера
  void setDelimiter(char delim) { delimiter_ = delim; }
  void setMaxArgs(uint8_t max) { max_args_ = max; }

  // Парсинг строки
  bool parse(const String& input);

  // Получение результатов
  const std::vector<String>& getTokens() const { return tokens_; }
  size_t getTokenCount() const { return tokens_.size(); }
  String getToken(uint8_t index) const;

  // Очистка
  void clear();

private:
  char delimiter_;
  uint8_t max_args_;
  std::vector<String> tokens_;
};
