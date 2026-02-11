#include <Arduino.h>
#include "TrajectoryGenerator.h"
#include "../../src/utils/Logger.h"
#include "../../src/utils/MathUtils.h"


TrajectoryGenerator::TrajectoryGenerator() :
    type_(TRAJ_LINEAR),
    max_velocity_(Limits::VELOCITY.max_linear_velocity * 0.5f),
    max_acceleration_(Limits::VELOCITY.max_linear_acceleration * 0.5f),
    max_jerk_(Limits::VELOCITY.max_jerk),
    is_running_(false),
    is_paused_(false),
    is_finished_(false),
    trajectory_point_count_(0),
    current_segment_(0),
    current_time_(0),
    total_time_(0),
    segment_start_time_(0),
    segment_progress_(0) {

  current_profile_.acceleration_time = 0;
  current_profile_.cruise_time = 0;
  current_profile_.deceleration_time = 0;
  current_profile_.cruise_velocity = 0;
  current_profile_.total_distance = 0;
}

bool TrajectoryGenerator::generate(const Vector3& start, const Vector3& end) {
  reset();

  float distance = start.distanceTo(end);
  if (distance < 0.001f) {
    Logger::warning("Trajectory distance too small: %.3f mm", distance);
    return false;
  }

  trajectory_point_count_ = 2;
  trajectory_points_[0] = start;
  trajectory_points_[1] = end;
  start_point_ = start;
  end_point_ = end;

  switch (type_) {
    case TRAJ_LINEAR:
    case TRAJ_POINT_TO_POINT:
      generateLinearTrajectory(start, end);
      total_time_ = calculateTrajectoryTime(distance);
      break;

    default:
      Logger::warning("Trajectory type %d not implemented for 2 points", type_);
      return false;
  }

  calculateVelocityProfile(distance);

  Logger::info("Trajectory generated: %.1f mm, %.2f s, %.1f mm/s",
               distance, total_time_ / 1000.0f, max_velocity_);

  return true;
}

bool TrajectoryGenerator::generate(const Vector3 points[], uint8_t num_points) {
  if (num_points < 2 || num_points > MAX_TRAJECTORY_POINTS) {
    Logger::error("Invalid number of trajectory points");
    return false;
  }

  reset();

  trajectory_point_count_ = num_points;
  for (uint8_t i = 0; i < num_points; i++) {
    trajectory_points_[i] = points[i];
  }

  start_point_ = points[0];
  end_point_ = points[num_points - 1];

  float total_distance = 0;
  for (uint8_t i = 1; i < num_points; i++) {
    total_distance += points[i-1].distanceTo(points[i]);
  }

  switch (type_) {
    case TRAJ_LINEAR:
    case TRAJ_POINT_TO_POINT:
      total_time_ = calculateTrajectoryTime(total_distance);
      break;

    case TRAJ_SPLINE:
      generateSplineTrajectory(points, num_points);
      break;

    default:
      Logger::warning("Trajectory type %d not implemented for multiple points", type_);
      return false;
  }

  calculateVelocityProfile(total_distance);

  Logger::info("Multi-point trajectory generated: %d points, %.1f mm, %.2f s",
               num_points, total_distance, total_time_ / 1000.0f);

  return true;
}

bool TrajectoryGenerator::generateCircle(const Vector3& center, float radius,
                                         const Vector3& normal, float start_angle,
                                         float end_angle) {
  if (radius <= 0 || fabs(end_angle - start_angle) < 0.001f) {
    Logger::error("Invalid circle parameters");
    return false;
  }

  reset();
  type_ = TRAJ_CIRCULAR;

  center_point_ = center;
  radius_ = radius;
  normal_vector_ = normal;
  start_angle_ = start_angle;
  end_angle_ = end_angle;

  generateCircularTrajectory(center, radius, normal, start_angle, end_angle);

  float arc_length = radius * fabs(end_angle - start_angle);
  total_time_ = calculateTrajectoryTime(arc_length);
  calculateVelocityProfile(arc_length);

  Logger::info("Circular trajectory generated: R=%.1f mm, angle=%.1f-%.1f deg, %.1f mm, %.2f s",
               radius, start_angle * MathUtils::RAD_TO_DEG, end_angle * MathUtils::RAD_TO_DEG,
               arc_length, total_time_ / 1000.0f);

  return true;
}

bool TrajectoryGenerator::getNextPoint(Vector3& point, uint32_t dt_ms) {
  if (!is_running_ || is_paused_ || is_finished_ || trajectory_point_count_ < 2) {
    return false;
  }

  current_time_ += dt_ms;

  if (current_time_ >= total_time_) {
    current_time_ = total_time_;
    is_finished_ = true;
    point = trajectory_points_[trajectory_point_count_ - 1];
    return true;
  }

  float t = (float)current_time_ / total_time_;

  switch (type_) {
    case TRAJ_LINEAR:
    case TRAJ_POINT_TO_POINT:
      point = interpolateLinear(t);
      break;

    case TRAJ_SPLINE:
      point = interpolateSpline(t);
      break;

    case TRAJ_CIRCULAR:
      point = interpolateCircular(t);
      break;

    default:
      point = trajectory_points_[0];
      break;
  }

  return true;
}

void TrajectoryGenerator::start() {
  if (trajectory_point_count_ < 2) {
    Logger::warning("Cannot start: no trajectory generated");
    return;
  }

  is_running_ = true;
  is_paused_ = false;
  is_finished_ = false;
  current_time_ = 0;
  current_segment_ = 0;
  segment_start_time_ = 0;
  segment_progress_ = 0;
}

void TrajectoryGenerator::stop() {
  is_running_ = false;
  is_finished_ = true;
}

void TrajectoryGenerator::pause() {
  if (is_running_ && !is_paused_) {
    is_paused_ = true;
  }
}

void TrajectoryGenerator::resume() {
  if (is_running_ && is_paused_) {
    is_paused_ = false;
  }
}

float TrajectoryGenerator::getProgress() const {
  if (total_time_ <= 0) return 0;
  return (float)current_time_ / total_time_;
}

float TrajectoryGenerator::getTimeRemaining() const {
  return (total_time_ - current_time_) / 1000.0f;
}

void TrajectoryGenerator::reset() {
  trajectory_point_count_ = 0;
  current_segment_ = 0;
  current_time_ = 0;
  total_time_ = 0;
  segment_start_time_ = 0;
  segment_progress_ = 0;
  is_running_ = false;
  is_paused_ = false;
  is_finished_ = false;

  current_profile_.acceleration_time = 0;
  current_profile_.cruise_time = 0;
  current_profile_.deceleration_time = 0;
  current_profile_.cruise_velocity = 0;
  current_profile_.total_distance = 0;
}

void TrajectoryGenerator::generateLinearTrajectory(const Vector3& start, const Vector3& end) {
  // Линейная траектория - точки уже установлены
}

void TrajectoryGenerator::generateSplineTrajectory(const Vector3 points[], uint8_t num_points) {
  // Для сплайна используем существующие точки
  // Время рассчитывается на основе общего расстояния
  float total_distance = 0;
  for (uint8_t i = 1; i < num_points; i++) {
    total_distance += points[i-1].distanceTo(points[i]);
  }
  total_time_ = calculateTrajectoryTime(total_distance);
}

void TrajectoryGenerator::generateCircularTrajectory(const Vector3& center, float radius,
                                                     const Vector3& normal, float start_angle,
                                                     float end_angle) {
  // Генерируем точки окружности для интерполяции
  trajectory_point_count_ = 50;
  if (trajectory_point_count_ > MAX_TRAJECTORY_POINTS) {
    trajectory_point_count_ = MAX_TRAJECTORY_POINTS;
  }

  float angle_step = (end_angle - start_angle) / (trajectory_point_count_ - 1);

  for (uint8_t i = 0; i < trajectory_point_count_; i++) {
    float angle = start_angle + i * angle_step;

    // Базовая окружность в плоскости XY
    if (fabs(normal.z) > 0.99f) {
      trajectory_points_[i] = center + Vector3(cosf(angle) * radius,
                                               sinf(angle) * radius, 0);
    } else {
      // Упрощенная реализация - предполагаем окружность в плоскости XY
      trajectory_points_[i] = center + Vector3(cosf(angle) * radius,
                                               sinf(angle) * radius, 0);
    }
  }

  start_point_ = trajectory_points_[0];
  end_point_ = trajectory_points_[trajectory_point_count_ - 1];
}

void TrajectoryGenerator::calculateVelocityProfile(float distance) {
  if (distance <= 0 || max_velocity_ <= 0 || max_acceleration_ <= 0) {
    return;
  }

  float t_accel = max_velocity_ / max_acceleration_;
  float d_accel = 0.5f * max_acceleration_ * t_accel * t_accel;

  if (d_accel * 2 >= distance) {
    // Треугольный профиль
    float max_vel = sqrtf(max_acceleration_ * distance);
    current_profile_.acceleration_time = max_vel / max_acceleration_;
    current_profile_.deceleration_time = current_profile_.acceleration_time;
    current_profile_.cruise_time = 0;
    current_profile_.cruise_velocity = max_vel;
  } else {
    // Трапецеидальный профиль
    current_profile_.acceleration_time = t_accel;
    current_profile_.deceleration_time = t_accel;
    current_profile_.cruise_time = (distance - 2 * d_accel) / max_velocity_;
    current_profile_.cruise_velocity = max_velocity_;
  }

  current_profile_.total_distance = distance;
}

Vector3 TrajectoryGenerator::interpolateLinear(float t) const {
  if (trajectory_point_count_ < 2) {
    return Vector3(0, 0, 0);
  }

  if (t <= 0) return trajectory_points_[0];
  if (t >= 1) return trajectory_points_[trajectory_point_count_ - 1];

  // Для двух точек - простая линейная интерполяция
  if (trajectory_point_count_ == 2) {
    return trajectory_points_[0] * (1 - t) + trajectory_points_[1] * t;
  }

  // Для нескольких точек - интерполяция по сегментам
  float segment_len = 1.0f / (trajectory_point_count_ - 1);
  uint8_t segment = (uint8_t)(t / segment_len);

  if (segment >= trajectory_point_count_ - 1) {
    return trajectory_points_[trajectory_point_count_ - 1];
  }

  float local_t = (t - segment * segment_len) / segment_len;
  return trajectory_points_[segment] * (1 - local_t) +
         trajectory_points_[segment + 1] * local_t;
}

Vector3 TrajectoryGenerator::interpolateSpline(float t) const {
  if (trajectory_point_count_ < 4) {
    return interpolateLinear(t);
  }

  float segment = t * (trajectory_point_count_ - 3);
  int i = (int)segment;
  float local_t = segment - i;

  if (i < 0) i = 0;
  if (i > trajectory_point_count_ - 4) i = trajectory_point_count_ - 4;

  return calculateSplinePoint(local_t,
                              trajectory_points_[i],
                              trajectory_points_[i + 1],
                              trajectory_points_[i + 2],
                              trajectory_points_[i + 3]);
}

Vector3 TrajectoryGenerator::interpolateCircular(float t) const {
  return interpolateLinear(t);
}

float TrajectoryGenerator::calculateTrajectoryTime(float distance) const {
  if (max_velocity_ <= 0) return 0;

  float t_accel = max_velocity_ / max_acceleration_;
  float d_accel = 0.5f * max_acceleration_ * t_accel * t_accel;

  if (d_accel * 2 >= distance) {
    return 2 * sqrtf(distance / max_acceleration_) * 1000.0f;
  } else {
    float cruise_distance = distance - 2 * d_accel;
    float cruise_time = cruise_distance / max_velocity_;
    return (2 * t_accel + cruise_time) * 1000.0f;
  }
}

Vector3 TrajectoryGenerator::calculateSplinePoint(float t, const Vector3& p0,
                                                  const Vector3& p1, const Vector3& p2,
                                                  const Vector3& p3) const {
  float t2 = t * t;
  float t3 = t2 * t;

  return p0 * (-0.5f * t3 + t2 - 0.5f * t) +
         p1 * (1.5f * t3 - 2.5f * t2 + 1.0f) +
         p2 * (-1.5f * t3 + 2.0f * t2 + 0.5f * t) +
         p3 * (0.5f * t3 - 0.5f * t2);
}

float TrajectoryGenerator::getSegmentLength(uint8_t segment) const {
  if (segment >= trajectory_point_count_ - 1) return 0;
  return trajectory_points_[segment].distanceTo(trajectory_points_[segment + 1]);
}

float TrajectoryGenerator::getSegmentTime(uint8_t segment) const {
  float dist = getSegmentLength(segment);
  float total_dist = current_profile_.total_distance;
  if (total_dist <= 0) return 0;
  return (dist / total_dist) * total_time_;
}
