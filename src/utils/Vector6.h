#pragma once

#include <Arduino.h>
#include "Vector3.h"
#include "../../src/utils/MathUtils.h"

class Vector6 {
public:
  float x, y, z;      // Позиция (мм)
  float ax, ay, az;   // Ориентация (радианы) - углы Эйлера

  // Конструкторы
  constexpr Vector6() : x(0), y(0), z(0), ax(0), ay(0), az(0) {}

  constexpr Vector6(float x_, float y_, float z_) :
      x(x_), y(y_), z(z_), ax(0), ay(0), az(0) {}

  constexpr Vector6(float x_, float y_, float z_, float ax_, float ay_, float az_) :
      x(x_), y(y_), z(z_), ax(ax_), ay(ay_), az(az_) {}

  // Конструктор из Vector3 (позиция)
  constexpr explicit Vector6(const Vector3& pos) :
      x(pos.x), y(pos.y), z(pos.z), ax(0), ay(0), az(0) {}

  // Конструктор из двух Vector3 (позиция + ориентация)
  constexpr Vector6(const Vector3& pos, const Vector3& orient) :
      x(pos.x), y(pos.y), z(pos.z), ax(orient.x), ay(orient.y), az(orient.z) {}

  // Преобразование в Vector3 (только позиция)
  Vector3 toPosition() const { return Vector3(x, y, z); }

  // Преобразование в Vector3 (только ориентация)
  Vector3 toOrientation() const { return Vector3(ax, ay, az); }

  // Базовые операции с полным вектором
  Vector6 operator+(const Vector6& v) const {
    return Vector6(
        x + v.x, y + v.y, z + v.z,
        ax + v.ax, ay + v.ay, az + v.az
    );
  }

  Vector6 operator-(const Vector6& v) const {
    return Vector6(
        x - v.x, y - v.y, z - v.z,
        ax - v.ax, ay - v.ay, az - v.az
    );
  }

  Vector6 operator*(float scalar) const {
    return Vector6(
        x * scalar, y * scalar, z * scalar,
        ax * scalar, ay * scalar, az * scalar
    );
  }

  Vector6 operator/(float scalar) const {
    float inv = 1.0f / scalar;
    return Vector6(
        x * inv, y * inv, z * inv,
        ax * inv, ay * inv, az * inv
    );
  }

  Vector6& operator+=(const Vector6& v) {
    x += v.x; y += v.y; z += v.z;
    ax += v.ax; ay += v.ay; az += v.az;
    return *this;
  }

  Vector6& operator-=(const Vector6& v) {
    x -= v.x; y -= v.y; z -= v.z;
    ax -= v.ax; ay -= v.ay; az -= v.az;
    return *this;
  }

  // Покомпонентные операции
  Vector6 operator-() const {
    return Vector6(-x, -y, -z, -ax, -ay, -az);
  }

  // Доступ по индексу (0-5)
  float& operator[](int index) {
    switch(index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
      case 3: return ax;
      case 4: return ay;
      case 5: return az;
      default: return x;  // Защита от выхода за границы
    }
  }

  const float& operator[](int index) const {
    switch(index) {
      case 0: return x;
      case 1: return y;
      case 2: return z;
      case 3: return ax;
      case 4: return ay;
      case 5: return az;
      default: return x;
    }
  }

  // Скалярное произведение полных 6D векторов
  float dot(const Vector6& v) const {
    return x * v.x + y * v.y + z * v.z +
           ax * v.ax + ay * v.ay + az * v.az;
  }

  // Скалярное произведение только позиции
  float dotPosition(const Vector6& v) const {
    return x * v.x + y * v.y + z * v.z;
  }

  // Скалярное произведение только ориентации
  float dotOrientation(const Vector6& v) const {
    return ax * v.ax + ay * v.ay + az * v.az;
  }

  // Длина полного 6D вектора
  float length() const {
    return sqrtf(x*x + y*y + z*z + ax*ax + ay*ay + az*az);
  }

  // Длина позиционной части
  float lengthPosition() const {
    return sqrtf(x*x + y*y + z*z);
  }

  // Длина ориентационной части
  float lengthOrientation() const {
    return sqrtf(ax*ax + ay*ay + az*az);
  }

  // Квадрат длины (быстрее)
  float lengthSquared() const {
    return x*x + y*y + z*z + ax*ax + ay*ay + az*az;
  }

  // Квадрат длины позиции
  float lengthPositionSquared() const {
    return x*x + y*y + z*z;
  }

  // Квадрат длины ориентации
  float lengthOrientationSquared() const {
    return ax*ax + ay*ay + az*az;
  }

  // Нормализация полного вектора
  Vector6 normalized() const {
    float len = length();
    if (len > 0.0001f) {
      return *this / len;
    }
    return Vector6(0, 0, 0, 0, 0, 0);
  }

  // Нормализация только позиции
  Vector6 normalizedPosition() const {
    float len = lengthPosition();
    if (len > 0.0001f) {
      return Vector6(x/len, y/len, z/len, ax, ay, az);
    }
    return Vector6(0, 0, 0, ax, ay, az);
  }

  // Расстояние между полными векторами
  float distanceTo(const Vector6& v) const {
    float dx = x - v.x;
    float dy = y - v.y;
    float dz = z - v.z;
    float dax = ax - v.ax;
    float day = ay - v.ay;
    float daz = az - v.az;
    return sqrtf(dx*dx + dy*dy + dz*dz + dax*dax + day*day + daz*daz);
  }

  // Расстояние только по позиции
  float distancePositionTo(const Vector6& v) const {
    float dx = x - v.x;
    float dy = y - v.y;
    float dz = z - v.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
  }

  // Угловое расстояние (разница ориентаций)
  float distanceOrientationTo(const Vector6& v) const {
    float dax = ax - v.ax;
    float day = ay - v.ay;
    float daz = az - v.az;
    // Нормализация углов
    dax = normalizeAngle(dax);
    day = normalizeAngle(day);
    daz = normalizeAngle(daz);
    return sqrtf(dax*dax + day*day + daz*daz);
  }

  // Проверка на равенство с допуском
  bool equals(const Vector6& v, float tolerance = 0.0001f) const {
    return fabs(x - v.x) < tolerance &&
           fabs(y - v.y) < tolerance &&
           fabs(z - v.z) < tolerance &&
           fabs(ax - v.ax) < tolerance &&
           fabs(ay - v.ay) < tolerance &&
           fabs(az - v.az) < tolerance;
  }

  // Проверка равенства только позиции
  bool equalsPosition(const Vector6& v, float tolerance = 0.0001f) const {
    return fabs(x - v.x) < tolerance &&
           fabs(y - v.y) < tolerance &&
           fabs(z - v.z) < tolerance;
  }

  // Проверка равенства только ориентации
  bool equalsOrientation(const Vector6& v, float tolerance = 0.0001f) const {
    float dax = normalizeAngle(ax - v.ax);
    float day = normalizeAngle(ay - v.ay);
    float daz = normalizeAngle(az - v.az);
    return fabs(dax) < tolerance &&
           fabs(day) < tolerance &&
           fabs(daz) < tolerance;
  }

  // Ограничение значений
  Vector6 clamp(float min_val, float max_val) const {
    return Vector6(
        fminf(fmaxf(x, min_val), max_val),
        fminf(fmaxf(y, min_val), max_val),
        fminf(fmaxf(z, min_val), max_val),
        fminf(fmaxf(ax, min_val), max_val),
        fminf(fmaxf(ay, min_val), max_val),
        fminf(fmaxf(az, min_val), max_val)
    );
  }

  // Ограничение только позиции
  Vector6 clampPosition(float min_val, float max_val) const {
    return Vector6(
        fminf(fmaxf(x, min_val), max_val),
        fminf(fmaxf(y, min_val), max_val),
        fminf(fmaxf(z, min_val), max_val),
        ax, ay, az
    );
  }

  // Ограничение только ориентации
  Vector6 clampOrientation(float min_val, float max_val) const {
    return Vector6(
        x, y, z,
        fminf(fmaxf(ax, min_val), max_val),
        fminf(fmaxf(ay, min_val), max_val),
        fminf(fmaxf(az, min_val), max_val)
    );
  }

  // Нормализация углов в диапазон [-PI, PI]
  static float normalizeAngle(float angle) {
    while (angle > MathUtils::PI) angle -= MathUtils::TWO_PI;
    while (angle < -MathUtils::PI) angle += MathUtils::TWO_PI;
    return angle;
  }

  // Нормализация всех угловых компонент
  void normalizeAngles() {
    ax = normalizeAngle(ax);
    ay = normalizeAngle(ay);
    az = normalizeAngle(az);
  }

  // Получение указателя на данные (для работы с матрицами)
  float* data() { return &x; }
  const float* data() const { return &x; }

  // Вывод в Serial (для отладки)
  void print() const {
    Serial.print("(");
    Serial.print(x, 2); Serial.print(", ");
    Serial.print(y, 2); Serial.print(", ");
    Serial.print(z, 2); Serial.print(") [");
    Serial.print(ax * 57.2958f, 1); Serial.print("°, ");
    Serial.print(ay * 57.2958f, 1); Serial.print("°, ");
    Serial.print(az * 57.2958f, 1); Serial.print("°]");
  }

  void println() const {
    print();
    Serial.println();
  }
};

// Умножение скаляра на вектор
inline Vector6 operator*(float scalar, const Vector6& v) {
  return v * scalar;
}

// Векторное произведение для 3D части (результат в позиции)
inline Vector6 crossPosition(const Vector6& a, const Vector6& b) {
  return Vector6(
      a.y * b.z - a.z * b.y,
      a.z * b.x - a.x * b.z,
      a.x * b.y - a.y * b.x,
      0, 0, 0
  );
}
