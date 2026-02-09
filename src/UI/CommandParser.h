#pragma once

#include <Arduino.h>
#include <vector>
#include <string>

class CommandParser {
public:
  CommandParser();

  // Настройка парсера
  void setDelimiter(char delim) { delimiter_ = delim; }
  void setMaxArgs(uint8_t max) { max_args_ = max; }

  // Парсинг строки
  bool parse(const std::string& input);

  // Получение результатов
  const std::vector<std::string>& getTokens() const { return tokens_; }
  size_t getTokenCount() const { return tokens_.size(); }
  std::string getToken(uint8_t index) const;

  // Очистка
  void clear();

private:
  char delimiter_;
  uint8_t max_args_;
  std::vector<std::string> tokens_;
};
