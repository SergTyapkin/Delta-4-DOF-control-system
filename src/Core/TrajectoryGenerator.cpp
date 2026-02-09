#include <cmath>
#include "TrajectoryGenerator.h"
#include "../../src/utils/Logger.h"

TrajectoryGenerator::TrajectoryGenerator() :
    type_(TrajectoryType::LINEAR),
    max_velocity_(Limits::VELOCITY.max_linear_velocity * 0.5f),
    max_acceleration_(Limits::VELOCITY.max_linear_acceleration * 0.5f),
    max_jerk_(Limits::VELOCITY.max_jerk),
    is_running_(false),
    is_paused_(false),
    is_finished_(false),
    current_point_index_(0),
    current_time_(0),
    total_time_(0),
    current_profile_{0, 0, 0, 0, 0, 0, 0, 0, 0, false} {
}

bool TrajectoryGenerator::generate(const Vector3& start, const Vector3& end) {
  reset();

  // Проверка расстояния
  float distance = start.distanceTo(end);
  if (distance < 0.001f) {
    Logger::warning("Trajectory distance too small: %.3f mm", distance);
    return false;
  }

  // Добавляем точки
  trajectory_points_.clear();
  trajectory_points_.push_back(start);
  trajectory_points_.push_back(end);

  // Генерируем траекторию в зависимости от типа
  switch (type_) {
    case TrajectoryType::LINEAR:
      generateLinearTrajectory(start, end);
      break;

    case TrajectoryType::POINT_TO_POINT:
      // Для point-to-point просто запоминаем точки
      total_time_ = calculateTrajectoryTime(distance);
      break;

    default:
      Logger::warning("Trajectory type %d not implemented for 2 points",
                      static_cast<int>(type_));
      return false;
  }

  // Рассчитываем профиль скорости
  calculateVelocityProfile(distance);

  Logger::info("Trajectory generated: %.1f mm, %.2f s, %.1f mm/s",
               distance, total_time_, max_velocity_);

  return true;
}

bool TrajectoryGenerator::generate(const std::vector<Vector3>& points) {
  if (points.size() < 2) {
    Logger::error("Trajectory needs at least 2 points");
    return false;
  }

  reset();
  trajectory_points_ = points;

  // Рассчитываем общее расстояние
  float total_distance = 0;
  for (size_t i = 1; i < points.size(); i++) {
    total_distance += points[i-1].distanceTo(points[i]);
  }

  // Генерируем траекторию
  switch (type_) {
    case TrajectoryType::LINEAR:
    case TrajectoryType::POINT_TO_POINT:
      // Для линейной и P2P просто используем расстояние
      total_time_ = calculateTrajectoryTime(total_distance);
      break;

    case TrajectoryType::SPLINE:
      generateSplineTrajectory(points);
      break;

    default:
      Logger::warning("Trajectory type %d not implemented for multiple points",
                      static_cast<int>(type_));
      return false;
  }

  calculateVelocityProfile(total_distance);

  Logger::info("Multi-point trajectory generated: %d points, %.1f mm, %.2f s",
               points.size(), total_distance, total_time_);

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
  type_ = TrajectoryType::CIRCULAR;

  // Генерируем точки окружности
  const int NUM_POINTS = 50;
  float angle_step = (end_angle - start_angle) / (NUM_POINTS - 1);

  // Создаем базисные векторы для плоскости окружности
  Vector3 u(1, 0, 0);
  Vector3 v(0, 1, 0);

  // Если normal не (0,0,1), нужно повернуть плоскость
  // (упрощенная реализация для normal = (0,0,1))

  for (int i = 0; i < NUM_POINTS; i++) {
    float angle = start_angle + i * angle_step;
    Vector3 point = center + Vector3(
        cosf(angle) * radius,
        sinf(angle) * radius,
        0
    );
    trajectory_points_.push_back(point);
  }

  // Рассчитываем длину окружности (части)
  float arc_length = radius * fabs(end_angle - start_angle);
  total_time_ = calculateTrajectoryTime(arc_length);
  calculateVelocityProfile(arc_length);

  Logger::info("Circular trajectory generated: R=%.1f mm, angle=%.1f-%.1f deg, %.1f mm, %.2f s",
               radius, MathUtils::toDegrees(start_angle),
               MathUtils::toDegrees(end_angle), arc_length, total_time_);

  return true;
}

bool TrajectoryGenerator::getNextPoint(Vector3& point, uint32_t dt_ms) {
  if (!is_running_ || is_paused_ || is_finished_) {
    return false;
  }

  float dt_s = dt_ms / 1000.0f;
  current_time_ += dt_s;

  if (current_time_ >= total_time_) {
    current_time_ = total_time_;
    is_finished_ = true;

    // Возвращаем последнюю точку
    point = trajectory_points_.back();
    return true;
  }

  // Интерполируем точку в зависимости от типа траектории
  switch (type_) {
    case TrajectoryType::LINEAR:
    case TrajectoryType::POINT_TO_POINT:
      point = interpolateLinear(current_time_ / total_time_);
      break;

    case TrajectoryType::SPLINE:
      point = interpolateSpline(current_time_ / total_time_);
      break;

    case TrajectoryType::CIRCULAR:
      point = interpolateCircular(current_time_ / total_time_);
      break;

    default:
      point = trajectory_points_[0];
      break;
  }

  return true;
}

void TrajectoryGenerator::start() {
  if (trajectory_points_.size() < 2) {
    Logger::warning("Cannot start: no trajectory generated");
    return;
  }

  is_running_ = true;
  is_paused_ = false;
  is_finished_ = false;
  current_time_ = 0;
  current_point_index_ = 0;

  Logger::debug("Trajectory started");
}

void TrajectoryGenerator::stop() {
  is_running_ = false;
  is_finished_ = true;
  Logger::debug("Trajectory stopped");
}

void TrajectoryGenerator::pause() {
  if (is_running_ && !is_paused_) {
    is_paused_ = true;
    Logger::debug("Trajectory paused");
  }
}

void TrajectoryGenerator::resume() {
  if (is_running_ && is_paused_) {
    is_paused_ = false;
    Logger::debug("Trajectory resumed");
  }
}

float TrajectoryGenerator::getProgress() const {
  if (total_time_ <= 0) return 0;
  return current_time_ / total_time_;
}

float TrajectoryGenerator::getTimeRemaining() const {
  return total_time_ - current_time_;
}

void TrajectoryGenerator::reset() {
  trajectory_points_.clear();
  current_point_index_ = 0;
  current_time_ = 0;
  total_time_ = 0;
  is_running_ = false;
  is_paused_ = false;
  is_finished_ = false;
  current_profile_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, false};
}

void TrajectoryGenerator::generateLinearTrajectory(const Vector3& start, const Vector3& end) {
  // Для линейной траектории точки уже есть
  // Рассчитываем только время
  float distance = start.distanceTo(end);
  total_time_ = calculateTrajectoryTime(distance);
}

void TrajectoryGenerator::generateSplineTrajectory(const std::vector<Vector3>& points) {
  // Упрощенная реализация кубического сплайна
  // В реальности нужен более сложный алгоритм

  // Рассчитываем общее расстояние для оценки времени
  float total_distance = 0;
  for (size_t i = 1; i < points.size(); i++) {
    total_distance += points[i-1].distanceTo(points[i]);
  }

  total_time_ = calculateTrajectoryTime(total_distance);
}

void TrajectoryGenerator::calculateVelocityProfile(float distance) {
  // S-образный профиль скорости (трапецеидальный с плавными переходами)

  if (distance <= 0 || max_velocity_ <= 0 || max_acceleration_ <= 0) {
    Logger::warning("Invalid parameters for velocity profile");
    return;
  }

  // Время разгона до максимальной скорости
  float t_accel = max_velocity_ / max_acceleration_;
  float distance_accel = 0.5f * max_acceleration_ * t_accel * t_accel;

  // Если расстояние слишком мало для достижения максимальной скорости
  if (distance_accel * 2 >= distance) {
    // Треугольный профиль
    float max_reachable_velocity = sqrtf(max_acceleration_ * distance);

    current_profile_.is_s_curve = false;
    current_profile_.t0 = 0;
    current_profile_.t1 = max_reachable_velocity / max_acceleration_;
    current_profile_.t2 = current_profile_.t1;
    current_profile_.t3 = current_profile_.t1 * 2;

    current_profile_.v0 = 0;
    current_profile_.v1 = max_reachable_velocity;
    current_profile_.v2 = max_reachable_velocity;
    current_profile_.v3 = 0;

    current_profile_.distance = distance;
  } else {
    // Трапецеидальный профиль
    float cruise_distance = distance - 2 * distance_accel;
    float t_cruise = cruise_distance / max_velocity_;

    current_profile_.is_s_curve = false;
    current_profile_.t0 = 0;
    current_profile_.t1 = t_accel;
    current_profile_.t2 = t_accel + t_cruise;
    current_profile_.t3 = t_accel + t_cruise + t_accel;

    current_profile_.v0 = 0;
    current_profile_.v1 = max_velocity_;
    current_profile_.v2 = max_velocity_;
    current_profile_.v3 = 0;

    current_profile_.distance = distance;
  }

  // Обновляем общее время
  total_time_ = current_profile_.t3;
}

Vector3 TrajectoryGenerator::interpolateLinear(float t) const {
  if (trajectory_points_.size() < 2) {
    return Vector3(0, 0, 0);
  }

  if (t <= 0) return trajectory_points_[0];
  if (t >= 1) return trajectory_points_.back();

  // Для двух точек - линейная интерполяция
  if (trajectory_points_.size() == 2) {
    return trajectory_points_[0] * (1 - t) + trajectory_points_[1] * t;
  }

  // Для нескольких точек - интерполяция по отрезкам
  float segment_length = 1.0f / (trajectory_points_.size() - 1);
  size_t segment = static_cast<int>(t / segment_length);

  if (segment >= trajectory_points_.size() - 1) {
    return trajectory_points_.back();
  }

  float local_t = (t - segment * segment_length) / segment_length;
  return trajectory_points_[segment] * (1 - local_t) +
         trajectory_points_[segment + 1] * local_t;
}

Vector3 TrajectoryGenerator::interpolateSpline(float t) const {
  // Упрощенная интерполяция сплайном
  // В реальности нужна более сложная реализация

  if (trajectory_points_.size() < 4) {
    // Если точек мало, используем линейную интерполяцию
    return interpolateLinear(t);
  }

  // Находим сегмент сплайна
  float segment = t * (trajectory_points_.size() - 3);
  int i = static_cast<int>(segment);
  float local_t = segment - i;

  // Ограничиваем индексы
  i = MathUtils::clamp(i, 0, static_cast<int>(trajectory_points_.size() - 4));

  return calculateSplinePoint(local_t,
                              trajectory_points_[i],
                              trajectory_points_[i + 1],
                              trajectory_points_[i + 2],
                              trajectory_points_[i + 3]);
}

Vector3 TrajectoryGenerator::interpolateCircular(float t) const {
  if (trajectory_points_.empty()) {
    return Vector3(0, 0, 0);
  }

  // Линейная интерполяция по сгенерированным точкам окружности
  return interpolateLinear(t);
}

float TrajectoryGenerator::calculateTrajectoryTime(float distance) const {
  if (max_velocity_ <= 0) return 0;

  // Оценка времени с учетом ускорения и замедления
  float acceleration_time = max_velocity_ / max_acceleration_;
  float acceleration_distance = 0.5f * max_acceleration_ * acceleration_time * acceleration_time;

  if (acceleration_distance * 2 >= distance) {
    // Треугольный профиль
    return 2 * sqrtf(distance / max_acceleration_);
  } else {
    // Трапецеидальный профиль
    float cruise_distance = distance - 2 * acceleration_distance;
    float cruise_time = cruise_distance / max_velocity_;
    return 2 * acceleration_time + cruise_time;
  }
}

Vector3 TrajectoryGenerator::calculateSplinePoint(float t, const Vector3& p0,
                                                  const Vector3& p1, const Vector3& p2,
                                                  const Vector3& p3) const {
  // Кубический сплайн Catmull-Rom
  float t2 = t * t;
  float t3 = t2 * t;

  Vector3 point = p0 * (-0.5f * t3 + t2 - 0.5f * t) +
                  p1 * (1.5f * t3 - 2.5f * t2 + 1.0f) +
                  p2 * (-1.5f * t3 + 2.0f * t2 + 0.5f * t) +
                  p3 * (0.5f * t3 - 0.5f * t2);

  return point;
}
