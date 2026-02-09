#include "CommandParser.h"
#include "../../src/utils/Utils.h"

CommandParser::CommandParser() :
    delimiter_(' '),
    max_args_(10) {
}

bool CommandParser::parse(const std::string& input) {
  clear();

  if (input.length() == 0) {
    return false;
  }

  int start = 0;
  int end = input.find(delimiter_);

  while (end != -1 && tokens_.size() < max_args_) {
    std::string token = input.substr(start, end);
    Utils::trim(token);

    if (token.length() > 0) {
      tokens_.push_back(token);
    }

    start = end + 1;
    end = input.find(delimiter_, start);
  }

  // Последний токен
  if (start < input.length() && tokens_.size() < max_args_) {
    std::string token = input.substr(start);
    Utils::trim(token);

    if (token.length() > 0) {
      tokens_.push_back(token);
    }
  }

  return tokens_.size() > 0;
}

std::string CommandParser::getToken(uint8_t index) const {
  if (index < tokens_.size()) {
    return tokens_[index];
  }
  return "";
}

void CommandParser::clear() {
  tokens_.clear();
}
