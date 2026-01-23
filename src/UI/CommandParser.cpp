// src/UI/CommandParser.cpp
#include "CommandParser.h"

CommandParser::CommandParser() :
    delimiter_(' '),
    max_args_(10) {
}

bool CommandParser::parse(const String& input) {
  clear();

  if (input.length() == 0) {
    return false;
  }

  int start = 0;
  int end = input.indexOf(delimiter_);

  while (end != -1 && tokens_.size() < max_args_) {
    String token = input.substring(start, end);
    token.trim();

    if (token.length() > 0) {
      tokens_.push_back(token);
    }

    start = end + 1;
    end = input.indexOf(delimiter_, start);
  }

  // Последний токен
  if (start < input.length() && tokens_.size() < max_args_) {
    String token = input.substring(start);
    token.trim();

    if (token.length() > 0) {
      tokens_.push_back(token);
    }
  }

  return tokens_.size() > 0;
}

String CommandParser::getToken(uint8_t index) const {
  if (index < tokens_.size()) {
    return tokens_[index];
  }
  return "";
}

void CommandParser::clear() {
  tokens_.clear();
}
