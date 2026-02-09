#pragma once

#include <cstdint>
#include <functional>
#include "../../config/pins_config.h"
#include "../../src/utils/MathUtils.h"

class Drive {
public:
  // Режимы работы привода
  enum class Mode {
    DISABLED,       // Двигатель отключен
    HOMING,         // Поиск нуля
    POSITION,       // Позиционное управление
    VELOCITY,       // Управление скоростью
    TORQUE,         // Управление моментом (если поддерживается)
    CALIBRATING     // Калибровка
  };

  // Состояния привода
  enum class State {
    IDLE,
    MOVING,
    HOMING_IN_PROGRESS,
    ERROR,
    LIMIT_TRIGGERED
  };

  // Направление homing
  enum class HomingDirection {
    POSITIVE,   // К положительному пределу
    NEGATIVE,   // К отрицательному пределу
    TO_LIMIT    // К концевику
  };

  // Конфигурация привода
  struct Config {
    // Пины
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    uint8_t limit_switch_pin;
    uint8_t fault_pin;

    // Параметры двигателя
    float steps_per_revolution;  // Шагов на оборот (например, 200)
    float microsteps;            // Коэффициент микрошагов (например, 16)
    float gear_ratio;            // Передаточное отношение редуктора

    // Пределы
    float max_velocity;          // Макс. скорость (рад/с)
    float max_acceleration;      // Макс. ускорение (рад/с²)
    float max_jerk;              // Макс. рывок (рад/с³)

    // Параметры homing
    float homing_velocity;       // Скорость homing (рад/с)
    float homing_acceleration;   // Ускорение homing (рад/с²)
    HomingDirection homing_direction; // Направление homing

    // Ток
    float run_current;           // Рабочий ток (А)
    float hold_current;          // Ток удержания (А)

    // Механические параметры
    float backlash_compensation; // Компенсация люфта (рад)
    bool invert_direction;       // Инвертировать направление

    Config() :
        step_pin(0), dir_pin(0), enable_pin(0),
        limit_switch_pin(0), fault_pin(0),
        steps_per_revolution(200.0f), microsteps(16.0f), gear_ratio(1.0f),
        max_velocity(10.0f), max_acceleration(50.0f), max_jerk(100.0f),
        homing_velocity(2.0f), homing_acceleration(10.0f),
        homing_direction(HomingDirection::NEGATIVE),
        run_current(1.0f), hold_current(0.5f),
        backlash_compensation(0.0f), invert_direction(false) {}
  };

  // Параметры движения
  struct MotionParams {
    float target_position;    // Целевая позиция (радианы)
    float target_velocity;    // Целевая скорость (рад/с)
    float acceleration;       // Ускорение (рад/с²)
    float jerk;               // Рывок (рад/с³)

    MotionParams() :
        target_position(0), target_velocity(0),
        acceleration(0), jerk(0) {}
  };

  // Конструктор
  Drive();

  // Инициализация
  bool init(const Config& config, uint8_t drive_id = 0);

  // Обновление состояния (вызывается периодически)
  void update(uint32_t delta_time_ms);

  // Управление режимами
  void enable();
  void disable();
  void setMode(Mode mode);

  // Управление движением
  bool moveToPosition(float position_rad, float velocity = 0);
  bool moveToPositionRelative(float delta_rad, float velocity = 0);
  bool setVelocity(float velocity_rad_s);

  // Homing
  bool startHoming();
  bool isHomingComplete() const;

  // Получение состояния
  State getState() const { return state_; }
  Mode getMode() const { return mode_; }
  float getPosition() const { return current_position_; }
  float getVelocity() const { return current_velocity_; }
  float getTargetPosition() const { return motion_params_.target_position; }
  bool isMoving() const { return state_ == State::MOVING; }
  bool isEnabled() const { return enabled_; }
  bool isHomed() const { return is_homed_; }
  bool isLimitTriggered();

  // Настройка параметров
  void setLimits(float min_position, float max_position);
  void setCurrent(float run_current, float hold_current);
  void setBacklashCompensation(float backlash);

  // Callback'и для событий
  typedef std::function<void(State, State)> StateChangeCallback;
  typedef std::function<void(bool)> HomingCompleteCallback;
  typedef std::function<void(float, float)> PositionUpdateCallback;

  void setStateChangeCallback(StateChangeCallback callback);
  void setHomingCompleteCallback(HomingCompleteCallback callback);
  void setPositionUpdateCallback(PositionUpdateCallback callback);

  // Аварийная остановка
  void emergencyStop();

  // Сброс ошибок
  bool resetError();

  // Отладка
  void printDebugInfo() const;

private:
  // Конфигурация
  Config config_;
  uint8_t drive_id_;

  // Состояние
  State state_;
  State previous_state_;
  Mode mode_;

  // Позиция и движение
  float current_position_;      // Текущая позиция (радианы)
  float current_velocity_;      // Текущая скорость (рад/с)
  float target_position_;       // Целевая позиция (радианы)
  MotionParams motion_params_;

  // Homing
  bool is_homing_;
  bool is_homed_;
  HomingDirection homing_direction_;
  uint32_t homing_start_time_;

  // Пределы
  float min_position_;
  float max_position_;

  // Управление шагами
  bool enabled_;
  bool direction_;
  uint32_t step_interval_;      // Интервал между шагами (микросекунды)
  uint32_t last_step_time_;
  uint32_t steps_to_execute_;
  uint32_t steps_executed_;

  // Трапецеидальный профиль скорости
  struct VelocityProfile {
    float acceleration_distance;
    float deceleration_distance;
    float cruise_distance;
    float max_reached_velocity;
    uint32_t acceleration_time;
    uint32_t deceleration_time;
    uint32_t cruise_time;
    bool is_trapezoidal;
  };

  VelocityProfile current_profile_;

  // Callback'и
  StateChangeCallback state_change_callback_;
  HomingCompleteCallback homing_complete_callback_;
  PositionUpdateCallback position_update_callback_;

  // Приватные методы
  void updateStepGenerator();
  void generateStep();
  void updateVelocityProfile();
  void calculateTrapezoidalProfile(float distance, float max_velocity,
                                   float acceleration, float deceleration);

  // Проверки безопасности
  bool checkLimits(float position) const;
  bool checkVelocity(float velocity) const;
  bool checkAcceleration(float acceleration) const;

  // Вспомогательные методы
  float stepsToRadians(int32_t steps) const;
  int32_t radiansToSteps(float radians) const;
  uint32_t calculateStepInterval(float velocity_rad_s) const;

  // Обработка сценариев
  void handleHoming(uint32_t delta_time_ms);
  void handlePositionControl(uint32_t delta_time_ms);
  void handleVelocityControl(uint32_t delta_time_ms);

  // Обработка прерываний
  void handleLimitSwitch();
  void handleDriverFault();

  // Аппаратно-зависимые методы
  void setStepPin(bool high);
  void setDirectionPin(bool direction);
  void setEnablePin(bool enable);
  bool readLimitSwitch() const;
  bool readFaultPin() const;

  // Симуляция ШИМ для микрошагов (если драйвер поддерживает)
  void setMicrostepPWM(uint8_t microstep_level);

  // Компенсация люфта
  float applyBacklashCompensation(float position);
};
