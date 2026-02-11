#pragma once

#include <Arduino.h>
#include "RobotState.h"
#include "TrajectoryGenerator.h"
#include "../../src/Core/Kinematics/DeltaSolver.h"
#include "../../src/DrivesController/DrivesController.h"
#include "../../src/utils/CircularBuffer.h"
#include "../../config/limits.h"

class Core {
public:
  // Конфигурация ядра
  struct Config {
    DeltaSolver::DeltaConfig kinematics_config;
    DrivesController::Config drives_config;
    float default_velocity;          // Скорость по умолчанию (мм/с)
    float default_acceleration;      // Ускорение по умолчанию (мм/с²)
    float max_jerk;                  // Максимальный рывок (мм/с³)
    float trajectory_update_rate;    // Частота обновления траектории (Гц)

    Config() :
        default_velocity(Limits::VELOCITY.max_linear_velocity * 0.5f),
        default_acceleration(Limits::VELOCITY.max_linear_acceleration * 0.5f),
        max_jerk(Limits::VELOCITY.max_jerk),
        trajectory_update_rate(100.0f) {}
  };

  // Режимы работы
  enum Mode {
    MODE_IDLE,
    MODE_DIRECT_JOINT,
    MODE_CARTESIAN,
    MODE_TRAJECTORY,
    MODE_HOMING,
    MODE_TEACHING,
    MODE_ERROR
  };

  // Команды высокого уровня
  struct Command {
    enum Type {
      CMD_MOVE_TO_POINT,
      CMD_MOVE_JOINTS,
      CMD_EXECUTE_TRAJECTORY,
      CMD_TEACH_POINT,
      CMD_RUN_PROGRAM,
      CMD_STOP,
      CMD_PAUSE,
      CMD_RESUME,
      CMD_EMERGENCY_STOP,
      CMD_SET_HOME,
      CMD_CALIBRATE
    };

    Type type;
    Vector3 target_point;
    float joint_angles[3];
    TrajectoryGenerator::TrajectoryType trajectory_type;
    float velocity;
    float acceleration;
    uint32_t id;
    uint32_t timeout;

    Command() :
        type(CMD_STOP),
        target_point(0, 0, 0),
        trajectory_type(TrajectoryGenerator::TRAJ_LINEAR),
        velocity(0),
        acceleration(0),
        id(0),
        timeout(10000) {
      joint_angles[0] = joint_angles[1] = joint_angles[2] = 0;
    }
  };

  // Состояния выполнения команды
  enum CommandStatus {
    STATUS_PENDING,
    STATUS_EXECUTING,
    STATUS_COMPLETED,
    STATUS_FAILED,
    STATUS_CANCELLED
  };

  // Результат выполнения команды
  struct CommandResult {
    uint32_t command_id;
    CommandStatus status;
    uint16_t error_code;
    char error_message[48];
    uint32_t execution_time;

    CommandResult() :
        command_id(0),
        status(STATUS_PENDING),
        error_code(0),
        execution_time(0) {
      error_message[0] = '\0';
    }
  };

  // Конструктор
  Core();

  // Инициализация
  bool init(const Config& config);

  // Обновление состояния
  void update(uint32_t delta_time_ms);

  // Управление командами
  uint32_t executeCommand(const Command& command);
  bool addCommandToQueue(const Command& command);
  void clearCommandQueue();

  // Управление движением
  bool moveToPoint(const Vector3& point, float velocity = 0,
                   TrajectoryGenerator::TrajectoryType traj_type = TrajectoryGenerator::TRAJ_LINEAR);
  bool moveJoints(const float angles[3], float velocity = 0);
  bool setCartesianVelocity(const Vector3& velocity);
  bool setJointVelocity(const float velocities[3]);

  // Homing и калибровка
  bool performHoming();
  bool calibrate();
  bool setHomePosition(const Vector3& home_point);

  // Управление состоянием
  void stop();
  void pause();
  void resume();
  void emergencyStop();
  bool reset();

  // Получение состояния
  Mode getMode() const { return mode_; }
  RobotState getState() const;
  CommandStatus getCommandStatus(uint32_t command_id) const;
  Vector3 getCurrentPosition() const;
  void getCurrentJoints(float angles[3]) const;
  bool isMoving() const;
  bool isHomed() const;
  bool isReady() const;

  // Траектории
  bool startTrajectory(const Vector3& start, const Vector3& end,
                       TrajectoryGenerator::TrajectoryType type, float velocity);
  bool addTrajectoryPoint(const Vector3& point);
  bool clearTrajectory();

  // Обучение
  bool teachPoint(const Vector3& point, uint32_t point_id = 0);

  // Callback'и с контекстом
  typedef void (*StateUpdateCallback)(const RobotState&, void*);
  typedef void (*CommandCompleteCallback)(const CommandResult&, void*);
  typedef void (*ModeChangeCallback)(Mode, Mode, void*);
  typedef void (*ErrorCallback)(uint16_t, const char*, void*);

  void setStateUpdateCallback(StateUpdateCallback callback, void* context = nullptr);
  void setCommandCompleteCallback(CommandCompleteCallback callback, void* context = nullptr);
  void setModeChangeCallback(ModeChangeCallback callback, void* context = nullptr);
  void setErrorCallback(ErrorCallback callback, void* context = nullptr);

  // Диагностика
  void printStatus() const;
  void printKinematicsInfo() const;
  void printDriveInfo() const;

private:
  // Конфигурация
  Config config_;

  // Компоненты
  DeltaSolver kinematics_;
  DrivesController drives_controller_;
  RobotState robot_state_;
  TrajectoryGenerator trajectory_generator_;

  // Состояние
  Mode mode_;
  Mode previous_mode_;
  bool is_initialized_;
  bool is_homed_;

  // Управление командами
  static const uint8_t COMMAND_QUEUE_SIZE = 16;
  CircularBuffer<Command, COMMAND_QUEUE_SIZE> command_queue_;
  Command current_command_;
  CommandStatus current_command_status_;
  uint32_t current_command_id_;
  uint32_t current_command_start_time_;

  static const uint8_t COMMAND_HISTORY_SIZE = 16;
  CommandResult command_history_[COMMAND_HISTORY_SIZE];
  uint8_t command_history_index_;

  // Траектории
  static const uint8_t MAX_TRAJECTORY_POINTS = 32;
  Vector3 trajectory_points_[MAX_TRAJECTORY_POINTS];
  uint8_t trajectory_points_count_;
  uint32_t current_trajectory_point_;
  bool trajectory_in_progress_;

  // Обучение
  struct TeachPoint {
    uint32_t id;
    Vector3 point;
  };
  static const uint8_t MAX_TEACH_POINTS = 20;
  TeachPoint teach_points_[MAX_TEACH_POINTS];
  uint8_t teach_points_count_;

  // Callback'и
  StateUpdateCallback state_update_callback_;
  void* state_update_context_;

  CommandCompleteCallback command_complete_callback_;
  void* command_complete_context_;

  ModeChangeCallback mode_change_callback_;
  void* mode_change_context_;

  ErrorCallback error_callback_;
  void* error_context_;

  // Приватные методы
  void processCommandQueue();
  void executeCurrentCommand();
  void updateState();
  void updateTrajectory();

  // Обработчики команд
  bool handleMoveToPoint(const Command& cmd);
  bool handleMoveJoints(const Command& cmd);
  bool handleExecuteTrajectory(const Command& cmd);
  bool handleTeachPoint(const Command& cmd);
  bool handleStop(const Command& cmd);
  bool handlePause(const Command& cmd);
  bool handleResume(const Command& cmd);
  bool handleEmergencyStop(const Command& cmd);
  bool handleSetHome(const Command& cmd);
  bool handleCalibrate(const Command& cmd);

  // Вспомогательные методы
  bool convertToJointAngles(const Vector3& point, float angles[3]);
  bool checkPointSafety(const Vector3& point);
  bool checkJointSafety(const float angles[3]);
  void logCommand(const Command& cmd, CommandStatus status,
                  uint16_t error_code = 0, const char* error_msg = "");

  // Обработчики событий от компонентов
  void onDriveStateChanged(uint8_t index, Drive::State old_state,
                           Drive::State new_state);
  void onDrivesHomingComplete(bool success);
  void onDrivesCommandComplete(uint32_t commands_executed);

  // Переменные для контроля времени
  uint32_t last_update_time_;
  uint32_t last_state_update_time_;
  uint32_t trajectory_update_interval_;
};
