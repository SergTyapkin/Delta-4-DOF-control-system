#pragma once

#include <Arduino.h>  // для математических функций
#undef PI
#undef TWO_PI
#undef DEG_TO_RAD
#undef RAD_TO_DEG
#undef min
#undef max

namespace MathUtils {
  // Константы
  constexpr float PI = 3.14159265358979323846f;
  constexpr float TWO_PI = 2.0f * PI;
  constexpr float DEG_TO_RAD = PI / 180.0f;
  constexpr float RAD_TO_DEG = 180.0f / PI;

  // Преобразование градусы <-> радианы
  inline float toRadians(float degrees) { return degrees * DEG_TO_RAD; }
  inline float toDegrees(float radians) { return radians * RAD_TO_DEG; }

  // Ограничение значения в диапазон - замена std::min/max
  template<typename T>
  inline T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
  }

  // Линейная интерполяция
  template<typename T>
  inline T lerp(T a, T b, float t) {
    return a + (b - a) * t;
  }

  // Проверка на равенство с допуском
  inline bool nearlyEqual(float a, float b, float epsilon = 0.0001f) {
    return abs(a - b) <= epsilon;  // abs вместо fabs
  }

  // Сигнум
  template<typename T>
  inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
  }

  // Ограничение угла в диапазон [-PI, PI]
  inline float wrapAngle(float angle) {
    // Вместо fmod - ручная реализация
    angle = angle - TWO_PI * floor(angle / TWO_PI);  // аналог fmod
    if (angle > PI) angle -= TWO_PI;
    if (angle < -PI) angle += TWO_PI;
    return angle;
  }

  // Минимум/максимум из трёх значений
  template<typename T>
  inline T min3(T a, T b, T c) {
    return (a < b) ? (a < c ? a : c) : (b < c ? b : c);
  }

  template<typename T>
  inline T max3(T a, T b, T c) {
    return (a > b) ? (a > c ? a : c) : (b > c ? b : c);
  }

  // Аппроксимация sqrt для ускорения (если нужна)
  inline float fastSqrt(float x) {
    union {
      float f;
      int32_t i;
    } conv = { .f = x };

    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f *= 1.5f - (x * 0.5f * conv.f * conv.f);
    return 1.0f / conv.f;
  }
};
