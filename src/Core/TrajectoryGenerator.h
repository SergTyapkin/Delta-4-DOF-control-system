#pragma once

#include <Arduino.h>
#include "../../src/utils/Vector3.h"
#include "../../config/limits.h"

class TrajectoryGenerator {
public:
  enum TrajectoryType {
    TRAJ_LINEAR,
    TRAJ_SPLINE,
    TRAJ_CIRCULAR,
    TRAJ_POINT_TO_POINT
  };

  TrajectoryGenerator();

  // Настройка параметров
  void setType(TrajectoryType type) { type_ = type; }
  void setMaxVelocity(float velocity) { max_velocity_ = velocity; }
  void setMaxAcceleration(float acceleration) { max_acceleration_ = acceleration; }
  void setMaxJerk(float jerk) { max_jerk_ = jerk; }

  // Генерация траектории
  bool generate(const Vector3& start, const Vector3& end);
  bool generate(const Vector3 points[], uint8_t num_points);
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
  static const uint8_t MAX_TRAJECTORY_POINTS = 32;
  Vector3 trajectory_points_[MAX_TRAJECTORY_POINTS];
  uint8_t trajectory_point_count_;
  uint8_t current_segment_;

  Vector3 start_point_;
  Vector3 end_point_;
  Vector3 center_point_;
  Vector3 normal_vector_;
  float radius_;
  float start_angle_;
  float end_angle_;

  uint32_t current_time_;
  uint32_t total_time_;
  uint32_t segment_start_time_;
  float segment_progress_;

  // Профиль скорости
  struct VelocityProfile {
    float acceleration_time;
    float cruise_time;
    float deceleration_time;
    float cruise_velocity;
    float total_distance;
  };

  VelocityProfile current_profile_;

  // Приватные методы
  void generateLinearTrajectory(const Vector3& start, const Vector3& end);
  void generateSplineTrajectory(const Vector3 points[], uint8_t num_points);
  void generateCircularTrajectory(const Vector3& center, float radius,
                                  const Vector3& normal, float start_angle, float end_angle);
  void calculateVelocityProfile(float distance);

  Vector3 interpolateLinear(float t) const;
  Vector3 interpolateSpline(float t) const;
  Vector3 interpolateCircular(float t) const;
  Vector3 calculateSplinePoint(float t, const Vector3& p0, const Vector3& p1,
                               const Vector3& p2, const Vector3& p3) const;

  // Вспомогательные методы
  float calculateTrajectoryTime(float distance) const;
  float getSegmentLength(uint8_t segment) const;
  float getSegmentTime(uint8_t segment) const;
};
