// utils/Matrix.h
#pragma once

#include <Arduino.h>
#include "Vector3.h"

class Matrix {
public:
  // Размеры матриц
  static const uint8_t SIZE_3x3 = 3;
  static const uint8_t SIZE_4x4 = 4;
  static const uint8_t JACOBIAN_ROWS = 4;  // Для 4-DOF: 4 угла
  static const uint8_t JACOBIAN_COLS = 3;  // 3 координаты (x,y,z)

  // ==================== Базовые операции ====================

  // Умножение матриц 4x4: C = A * B
  static void multiply(const float A[4][4], const float B[4][4], float C[4][4]) {
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        C[i][j] = 0;
        for (uint8_t k = 0; k < 4; k++) {
          C[i][j] += A[i][k] * B[k][j];
        }
      }
    }
  }

  // Умножение матрицы 4x4 на вектор 3 (с учетом w=1 для точек)
  static Vector3 multiplyPoint(const float M[4][4], const Vector3& v) {
    float x = M[0][0] * v.x + M[0][1] * v.y + M[0][2] * v.z + M[0][3];
    float y = M[1][0] * v.x + M[1][1] * v.y + M[1][2] * v.z + M[1][3];
    float z = M[2][0] * v.x + M[2][1] * v.y + M[2][2] * v.z + M[2][3];
    float w = M[3][0] * v.x + M[3][1] * v.y + M[3][2] * v.z + M[3][3];

    // Перспективное деление (если нужно)
    if (fabs(w) > 1e-6f) {
      return Vector3(x / w, y / w, z / w);
    }
    return Vector3(x, y, z);
  }

  // Умножение матрицы 4x4 на вектор 4
  static void multiply(const float M[4][4], const float v[4], float result[4]) {
    for (uint8_t i = 0; i < 4; i++) {
      result[i] = 0;
      for (uint8_t j = 0; j < 4; j++) {
        result[i] += M[i][j] * v[j];
      }
    }
  }

  // Умножение матрицы 3x3 на вектор 3
  static Vector3 multiply(const float M[3][3], const Vector3& v) {
    return Vector3(
        M[0][0] * v.x + M[0][1] * v.y + M[0][2] * v.z,
        M[1][0] * v.x + M[1][1] * v.y + M[1][2] * v.z,
        M[2][0] * v.x + M[2][1] * v.y + M[2][2] * v.z
    );
  }

  // Умножение матрицы 4x3 на вектор 3
  static void multiply(const float M[4][3], const float v[3], float result[4]) {
    for (uint8_t i = 0; i < 4; i++) {
      result[i] = 0;
      for (uint8_t j = 0; j < 3; j++) {
        result[i] += M[i][j] * v[j];
      }
    }
  }

  // Умножение матрицы 4x3 на вектор 3
  static void multiply(const float M[4][3], const Vector3& v, float result[4]) {
    float v_array[3] = {v.x, v.y, v.z};
    multiply(M, v_array, result);
  }

  // Транспонирование матрицы 4x4
  static void transpose(const float A[4][4], float At[4][4]) {
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        At[j][i] = A[i][j];
      }
    }
  }

  // Транспонирование матрицы 3x3
  static void transpose(const float A[3][3], float At[3][3]) {
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 3; j++) {
        At[j][i] = A[i][j];
      }
    }
  }

  // Транспонирование матрицы Якобиана (4x3 -> 3x4)
  static void transposeJacobian(const float J[4][3], float Jt[3][4]) {
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 3; j++) {
        Jt[j][i] = J[i][j];
      }
    }
  }

  // Транспонирование обратной матрицы Якобиана (3x4 -> 4x3)
  static void transposeJacobian(const float J[3][4], float Jt[4][3]) {
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        Jt[j][i] = J[i][j];
      }
    }
  }

  // ==================== Определитель и ранг ====================

  // Определитель матрицы 3x3
  static float determinant3x3(const float M[3][3]) {
    return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
           - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
           + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  }

  // Определитель матрицы 4x4
  static float determinant4x4(const float M[4][4]) {
    float det = 0;

    // Разложение по первой строке
    for (uint8_t i = 0; i < 4; i++) {
      float submatrix[3][3];
      for (uint8_t j = 1; j < 4; j++) {
        uint8_t col = 0;
        for (uint8_t k = 0; k < 4; k++) {
          if (k == i) continue;
          submatrix[j-1][col++] = M[j][k];
        }
      }
      float subdet = determinant3x3(submatrix);
      det += (i % 2 == 0 ? 1 : -1) * M[0][i] * subdet;
    }
    return det;
  }

  // Вычисление JTJ для Якобиана (4x3 -> 3x3)
  static void computeJTJ(const float J[4][3], float JTJ[3][3]) {
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 3; j++) {
        JTJ[i][j] = 0;
        for (uint8_t k = 0; k < 4; k++) {
          JTJ[i][j] += J[k][i] * J[k][j];
        }
      }
    }
  }

  // Определитель для неквадратных матриц (через JTJ)
  static float determinantJacobian(const float J[4][3]) {
    float JTJ[3][3];
    computeJTJ(J, JTJ);
    return determinant3x3(JTJ);
  }

  // Ранг матрицы 3x3 (число ненулевых сингулярных чисел)
  static uint8_t rank3x3(const float M[3][3], float tolerance = 1e-6f) {
    float det = determinant3x3(M);
    if (fabs(det) < tolerance) return 2;  // Упрощенно

    // Проверка на вырожденность строк/столбцов
    bool zeroRow = true;
    for (uint8_t i = 0; i < 3; i++) {
      zeroRow = true;
      for (uint8_t j = 0; j < 3; j++) {
        if (fabs(M[i][j]) > tolerance) {
          zeroRow = false;
          break;
        }
      }
      if (zeroRow) return 2;
    }

    return 3;
  }

  // ==================== Обратная матрица ====================

  // Обратная матрица 3x3
  static bool inverse3x3(const float M[3][3], float inv[3][3]) {
    float det = determinant3x3(M);
    if (fabs(det) < 1e-9f) return false;

    float invDet = 1.0f / det;

    inv[0][0] =  (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * invDet;
    inv[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) * invDet;
    inv[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * invDet;
    inv[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) * invDet;
    inv[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * invDet;
    inv[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) * invDet;
    inv[2][0] =  (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * invDet;
    inv[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) * invDet;
    inv[2][2] =  (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * invDet;

    return true;
  }

  // Псевдообратная матрица для Якобиана J (4x3)
  // result = (J^T * J)^-1 * J^T  размером [3x4]
  static bool pseudoInverseJacobian(const float J[4][3], float result[3][4]) {
    // 1. Вычисляем JTJ = J^T * J (3x3)
    float JTJ[3][3];
    computeJTJ(J, JTJ);

    // 2. Вычисляем (JTJ)^-1 (3x3)
    float invJTJ[3][3];
    if (!inverse3x3(JTJ, invJTJ)) {
      return false;
    }

    // 3. Вычисляем J^T (3x4)
    float Jt[3][4];
    transposeJacobian(J, Jt);

    // 4. result = invJTJ * Jt (3x4)
    for (uint8_t i = 0; i < 3; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        result[i][j] = 0;
        for (uint8_t k = 0; k < 3; k++) {
          result[i][j] += invJTJ[i][k] * Jt[k][j];
        }
      }
    }

    return true;
  }

  // ==================== Матрицы преобразований ====================

  // Единичная матрица 4x4
  static void identity4x4(float M[4][4]) {
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        M[i][j] = (i == j) ? 1.0f : 0.0f;
      }
    }
  }

  // Матрица поворота вокруг оси X (угол в радианах)
  static void rotationX(float angle, float M[4][4]) {
    identity4x4(M);
    float c = cosf(angle);
    float s = sinf(angle);
    M[1][1] = c;   M[1][2] = -s;
    M[2][1] = s;   M[2][2] = c;
  }

  // Матрица поворота вокруг оси Y
  static void rotationY(float angle, float M[4][4]) {
    identity4x4(M);
    float c = cosf(angle);
    float s = sinf(angle);
    M[0][0] = c;   M[0][2] = s;
    M[2][0] = -s;  M[2][2] = c;
  }

  // Матрица поворота вокруг оси Z
  static void rotationZ(float angle, float M[4][4]) {
    identity4x4(M);
    float c = cosf(angle);
    float s = sinf(angle);
    M[0][0] = c;   M[0][1] = -s;
    M[1][0] = s;   M[1][1] = c;
  }

  // Матрица поворота по углам Эйлера (ZYX - для робототехники)
  // angles: x = roll, y = pitch, z = yaw
  static void rotationEuler(const Vector3& angles, float M[4][4]) {
    float Rx[4][4], Ry[4][4], Rz[4][4], temp[4][4];

    rotationX(angles.x, Rx);
    rotationY(angles.y, Ry);
    rotationZ(angles.z, Rz);

    // R = Rz * Ry * Rx (для углов ZYX)
    multiply(Rz, Ry, temp);
    multiply(temp, Rx, M);
  }

  // Матрица трансляции
  static void translation(const Vector3& offset, float M[4][4]) {
    identity4x4(M);
    M[0][3] = offset.x;
    M[1][3] = offset.y;
    M[2][3] = offset.z;
  }

  // Матрица масштабирования (равномерное)
  static void scaling(float scale, float M[4][4]) {
    identity4x4(M);
    M[0][0] = scale;
    M[1][1] = scale;
    M[2][2] = scale;
  }

  // Матрица масштабирования (по осям)
  static void scaling(const Vector3& scale, float M[4][4]) {
    identity4x4(M);
    M[0][0] = scale.x;
    M[1][1] = scale.y;
    M[2][2] = scale.z;
  }

  // ==================== Преобразование векторов ====================

  // Поворот вектора по углам Эйлера
  static Vector3 rotateVector(const Vector3& v, const Vector3& euler_angles) {
    float R[4][4];
    rotationEuler(euler_angles, R);

    float v4[4] = {v.x, v.y, v.z, 1.0f};
    float result[4];
    multiply(R, v4, result);

    return Vector3(result[0], result[1], result[2]);
  }

  // Поворот вектора по углам Эйлера вокруг точки
  static Vector3 rotateVectorAroundPoint(const Vector3& v, const Vector3& euler_angles, const Vector3& center) {
    float R[4][4];
    rotationEuler(euler_angles, R);

    Vector3 vec_from_center = v - center;
    float v4[4] = {vec_from_center.x, vec_from_center.y, vec_from_center.z, 1.0f};
    float result[4];
    multiply(R, v4, result);

    Vector3 rotated_vec_from_center = Vector3(result[0], result[1], result[2]);
    return rotated_vec_from_center + center;
  }

  // Поворот вектора вокруг произвольной оси (формула Родрига)
  static Vector3 rotateAroundAxis(const Vector3& v, const Vector3& axis, float angle) {
    Vector3 a = axis.normalized();
    float c = cosf(angle);
    float s = sinf(angle);

    // v_rot = v*c + (a×v)*s + a*(a·v)*(1-c)
    Vector3 cross = a.cross(v);
    float dot = a.dot(v);

    return v * c + cross * s + a * dot * (1.0f - c);
  }

  // Комбинированное преобразование (поворот + смещение)
  static Vector3 transformPoint(const Vector3& point, const float M[4][4]) {
    return multiplyPoint(M, point);
  }

  // ==================== Утилиты ====================

  // Копирование матрицы
  static void copy4x4(const float src[4][4], float dst[4][4]) {
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 4; j++) {
        dst[i][j] = src[i][j];
      }
    }
  }

  // Вывод матрицы в Serial (для отладки)
  static void print4x4(const float M[4][4], const char* name = "Matrix") {
    Serial.print(name);
    Serial.println(" =");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print("  [ ");
      for (uint8_t j = 0; j < 4; j++) {
        Serial.print(M[i][j], 4);
        if (j < 3) Serial.print(", ");
      }
      Serial.println(" ]");
    }
  }

  static void print3x3(const float M[3][3], const char* name = "Matrix") {
    Serial.print(name);
    Serial.println(" =");
    for (uint8_t i = 0; i < 3; i++) {
      Serial.print("  [ ");
      for (uint8_t j = 0; j < 3; j++) {
        Serial.print(M[i][j], 4);
        if (j < 2) Serial.print(", ");
      }
      Serial.println(" ]");
    }
  }
};
