#pragma once

#include <cmath>
#include <cstdint>

class Vector3 {
public:
  float x, y, z;

  // Конструкторы
  Vector3() : x(0), y(0), z(0) {}
  Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  // Базовые операции
  Vector3 operator+(const Vector3& v) const {
    return Vector3(x + v.x, y + v.y, z + v.z);
  }

  Vector3 operator-(const Vector3& v) const {
    return Vector3(x - v.x, y - v.y, z - v.z);
  }

  Vector3 operator*(float scalar) const {
    return Vector3(x * scalar, y * scalar, z * scalar);
  }

  Vector3 operator/(float scalar) const {
    return Vector3(x / scalar, y / scalar, z / scalar);
  }

  Vector3& operator+=(const Vector3& v) {
    x += v.x; y += v.y; z += v.z;
    return *this;
  }

  Vector3& operator-=(const Vector3& v) {
    x -= v.x; y -= v.y; z -= v.z;
    return *this;
  }

  // Скалярное произведение
  float dot(const Vector3& v) const {
    return x * v.x + y * v.y + z * v.z;
  }

  // Векторное произведение
  Vector3 cross(const Vector3& v) const {
    return Vector3(
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    );
  }

  // Длина вектора
  float length() const {
    return sqrtf(x * x + y * y + z * z);
  }

  // Нормализация
  Vector3 normalized() const {
    float len = length();
    if (len > 0.0001f) {
      return Vector3(x / len, y / len, z / len);
    }
    return Vector3(0, 0, 0);
  }

  // Квадрат длины (более быстрый для сравнений)
  float lengthSquared() const {
    return x * x + y * y + z * z;
  }

  // Расстояние между векторами
  float distanceTo(const Vector3& v) const {
    return (*this - v).length();
  }

  // Проверка на равенство с допуском
  bool equals(const Vector3& v, float tolerance = 0.0001f) const {
    return fabs(x - v.x) < tolerance &&
           fabs(y - v.y) < tolerance &&
           fabs(z - v.z) < tolerance;
  }

  // Ограничение значений
  Vector3 clamp(float min_val, float max_val) const {
    return Vector3(
        fminf(fmaxf(x, min_val), max_val),
        fminf(fmaxf(y, min_val), max_val),
        fminf(fmaxf(z, min_val), max_val)
    );
  }
};

// Умножение скаляра на вектор
inline Vector3 operator*(float scalar, const Vector3& v) {
  return v * scalar;
}
