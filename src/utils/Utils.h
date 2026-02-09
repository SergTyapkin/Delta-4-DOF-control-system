#pragma once

#include <sstream>
#include <string>

namespace Utils {
  template<typename T>
  std::string toString(T value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
  }

  static inline std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
  }

  static inline std::string toUpper(const std::string& str) {
    std::string result = str;
    for (char& c : result) {
      c = ::toupper(c);
    }
    return result;
  }

  static inline float toFloat(const std::string& str) {
    return atof(str.c_str());
  }
};
