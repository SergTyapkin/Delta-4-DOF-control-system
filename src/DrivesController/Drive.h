#pragma once

#include <Arduino.h>
#include "../../config/pins_config.h"

class Drive {
public:
  // Режимы работы привода
  enum Mode {
    MODE_DISABLED,
    MODE_HOMING,
    MODE_POSITION,
    MODE_VELOCITY
  };

  // Состояния привода
  enum State {
    STATE_IDLE,
    STATE_MOVING,
    STATE_HOMING_IN_PROGRESS,
    STATE_ERROR,
    STATE_LIMIT_TRIGGERED
  };

  // Направление homing
  enum HomingDirection {
    HOMING_POSITIVE,
    HOMING_NEGATIVE,
    HOMING_TO_LIMIT
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
    float steps_per_revolution;
    float microsteps;
    float gear_ratio;

    // Пределы
    float max_velocity;
    float max_acceleration;

    // Параметры homing
    float homing_velocity;
    float homing_acceleration;
    HomingDirection homing_direction;

    // Ток
    float run_current;
    float hold_current;

    // Механические параметры
    float backlash_compensation;
    bool invert_direction;

    Config() :
        step_pin(0), dir_pin(0), enable_pin(0),
        limit_switch_pin(0), fault_pin(0),
        steps_per_revolution(200.0f), microsteps(16.0f), gear_ratio(1.0f),
        max_velocity(10.0f), max_acceleration(50.0f),
        homing_velocity(2.0f), homing_acceleration(10.0f),
        homing_direction(HOMING_NEGATIVE),
        run_current(1.0f), hold_current(0.5f),
        backlash_compensation(0.0f), invert_direction(false) {}
  };

  // Параметры движения (упрощаем)
  struct MotionParams {
    float target_position;
    float target_velocity;
    float acceleration;

    MotionParams() :
        target_position(0), target_velocity(0), acceleration(0) {}
  };

  // Конструктор
  Drive();

  // Инициализация
  bool init(const Config& config, uint8_t drive_id = 0);

  // Обновление состояния
  void update(uint32_t delta_time_ms);

  // Управление режимами
  void enable();
  void disable();
  void setMode(Mode mode);

  // Управление движением
  bool moveToPosition(float position_rad, float velocity = 0);
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
  bool isMoving() const { return state_ == STATE_MOVING; }
  bool isEnabled() const { return enabled_; }
  bool isHomed() const { return is_homed_; }
  bool isLimitTriggered();

  // Настройка параметров
  void setLimits(float min_position, float max_position);
  void setCurrent(float run_current, float hold_current);
  void setBacklashCompensation(float backlash);

  // Callback'и с контекстом
  typedef void (*StateChangeCallback)(uint8_t, State, State, void*);
  typedef void (*HomingCompleteCallback)(uint8_t, bool, void*);
  typedef void (*PositionUpdateCallback)(uint8_t, float, float, void*);

  void setStateChangeCallback(StateChangeCallback callback, void* context = nullptr);
  void setHomingCompleteCallback(HomingCompleteCallback callback, void* context = nullptr);
  void setPositionUpdateCallback(PositionUpdateCallback callback, void* context = nullptr);

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
  float current_position_;
  float current_velocity_;
  float target_position_;
  MotionParams motion_params_;

  // Homing
  bool is_homing_;
  bool is_homed_;
  uint32_t homing_start_time_;

  // Пределы
  float min_position_;
  float max_position_;

  // Управление шагами
  bool enabled_;
  bool direction_;
  uint32_t step_interval_;
  uint32_t last_step_time_;
  int32_t steps_remaining_;

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
  uint32_t profile_start_time_;

  // Callback'и с контекстом
  StateChangeCallback state_change_callback_;
  void* state_change_context_;

  HomingCompleteCallback homing_complete_callback_;
  void* homing_complete_context_;

  PositionUpdateCallback position_update_callback_;
  void* position_update_context_;

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

  // Аппаратно-зависимые методы
  void setStepPin(bool high);
  void setDirectionPin(bool direction);
  void setEnablePin(bool enable);
  bool readLimitSwitch() const;
  bool readFaultPin() const;

  // Компенсация люфта
  float applyBacklashCompensation(float position);
};
