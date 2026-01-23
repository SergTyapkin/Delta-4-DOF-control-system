// src/Core/TrajectoryGenerator.h
#pragma once

#include "utils/Vector3.h"
#include "utils/MathUtils.h"
#include "config/limits.h"
#include <vector>

enum class TrajectoryType {
  LINEAR,     // Линейная интерполяция
  SPLINE,     // Сплайн
  CIRCULAR,   // Круговая
  POINT_TO_POINT // Из точки в точку
};

class TrajectoryGenerator {
public:
  TrajectoryGenerator();

  // Настройка параметров
  void setType(TrajectoryType type) { type_ = type; }
  void setMaxVelocity(float velocity) { max_velocity_ = velocity; }
  void setMaxAcceleration(float acceleration) { max_acceleration_ = acceleration; }
  void setMaxJerk(float jerk) { max_jerk_ = jerk; }

  // Генерация траектории
  bool generate(const Vector3& start, const Vector3& end);
  bool generate(const std::vector<Vector3>& points);
  bool generateCircle(const Vector3& center, float radius,
                      const Vector3& normal, float start_angle, float end_angle);

  // Получение следующей точки
  bool getNextPoint(Vector3& point, uint32_t dt_ms);

  // Управление
  void start();
  void stop();
  void pause();
  void resume();

  // Состояние
  bool isRunning() const { return is_running_; }
  bool isPaused() const { return is_paused_; }
  bool isFinished() const { return is_finished_; }

  float getProgress() const;
  float getTimeRemaining() const;
  float getMaxVelocity() const { return max_velocity_; }

  // Сброс
  void reset();

private:
  // Параметры траектории
  TrajectoryType type_;
  float max_velocity_;
  float max_acceleration_;
  float max_jerk_;

  // Состояние
  bool is_running_;
  bool is_paused_;
  bool is_finished_;

  // Текущая траектория
  std::vector<Vector3> trajectory_points_;
  uint32_t current_point_index_;
  float current_time_;
  float total_time_;

  // Профиль скорости (S-кривая)
  struct VelocityProfile {
    float t0, t1, t2, t3; // Время фаз
    float v0, v1, v2, v3; // Скорости в фазах
    float distance;
    bool is_s_curve;
  };

  VelocityProfile current_profile_;

  // Приватные методы
  void generateLinearTrajectory(const Vector3& start, const Vector3& end);
  void generateSplineTrajectory(const std::vector<Vector3>& points);
  void calculateVelocityProfile(float distance);
  Vector3 interpolateLinear(float t) const;
  Vector3 interpolateSpline(float t) const;
  Vector3 interpolateCircular(float t) const;

  // Вспомогательные методы
  float calculateTrajectoryTime(float distance) const;
  Vector3 calculateSplinePoint(float t, const Vector3& p0, const Vector3& p1,
                               const Vector3& p2, const Vector3& p3) const;
};
