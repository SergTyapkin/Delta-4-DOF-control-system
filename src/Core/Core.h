#pragma once

#include <functional>
#include <map>
#include <string>
#include "RobotState.h"
#include "TrajectoryGenerator.h"
#include "../../src/Core/Kinematics/DeltaSolver.h"
#include "../../src/DrivesController/DrivesController.h"
#include "../../src/utils/CircularBuffer.h"
#include "../../src/utils/Utils.h"
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
        trajectory_update_rate(100.0f) {} // 100 Гц
  };

  // Режимы работы
  enum class Mode {
    IDLE,               // Ожидание команд
    DIRECT_JOINT,       // Прямое управление шарнирами
    CARTESIAN,          // Управление в декартовых координатах
    TRAJECTORY,         // Выполнение траектории
    HOMING,             // Калибровка/поиск нуля
    TEACHING,           // Режим обучения
    ERROR               // Ошибка
  };

  // Команды высокого уровня
  struct Command {
    enum class Type {
      MOVE_TO_POINT,          // Движение к точке (x,y,z)
      MOVE_JOINTS,            // Движение шарниров
      EXECUTE_TRAJECTORY,     // Выполнение траектории
      TEACH_POINT,            // Запомнить точку
      RUN_PROGRAM,            // Запуск программы
      STOP,                   // Остановка
      PAUSE,                  // Пауза
      RESUME,                 // Продолжить
      EMERGENCY_STOP,         // Аварийная остановка
      SET_HOME,               // Установить нулевую позицию
      CALIBRATE               // Калибровка
    };

    Type type;
    Vector3 target_point;           // Целевая точка (мм)
    std::array<float, 3> joint_angles; // Углы шарниров (рад)
    TrajectoryType trajectory_type;   // Тип траектории
    float velocity;                  // Скорость (мм/с или %)
    float acceleration;              // Ускорение (мм/с²)
    uint32_t id;                     // ID команды
    uint32_t timeout;                // Таймаут (мс)

    Command() :
        type(Type::STOP),
        target_point(0, 0, 0),
        joint_angles({0, 0, 0}),
        trajectory_type(TrajectoryType::LINEAR),
        velocity(0),
        acceleration(0),
        id(0),
        timeout(10000) {} // 10 секунд по умолчанию
  };

  // Состояния выполнения команды
  enum class CommandStatus {
    PENDING,        // Ожидает выполнения
    EXECUTING,      // Выполняется
    COMPLETED,      // Успешно завершена
    FAILED,         // Завершена с ошибкой
    CANCELLED       // Отменена
  };

  // Результат выполнения команды
  struct CommandResult {
    uint32_t command_id;
    CommandStatus status;
    uint16_t error_code;
    std::string error_message;
    uint32_t execution_time; // Время выполнения (мс)

    CommandResult() :
        command_id(0),
        status(CommandStatus::PENDING),
        error_code(0),
        execution_time(0) {}
  };

  // Конструктор
  Core();

  // Инициализация
  bool init(const Config& config);

  // Обновление состояния (вызывается периодически)
  void update(uint32_t delta_time_ms);

  // Управление командами
  uint32_t executeCommand(const Command& command);
  bool addCommandToQueue(const Command& command);
  void clearCommandQueue();

  // Управление движением
  bool moveToPoint(const Vector3& point, float velocity = 0,
                   TrajectoryType traj_type = TrajectoryType::LINEAR);

  bool moveJoints(const std::array<float, 3>& angles, float velocity = 0);

  bool setCartesianVelocity(const Vector3& velocity);
  bool setJointVelocity(const std::array<float, 3>& velocities);

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
  std::array<float, 3> getCurrentJoints() const;
  bool isMoving() const;
  bool isHomed() const;
  bool isReady() const;

  // Траектории
  bool startTrajectory(const Vector3& start, const Vector3& end,
                       TrajectoryType type, float velocity);

  bool addTrajectoryPoint(const Vector3& point);
  bool clearTrajectory();

  // Обучение (teaching)
  bool teachPoint(const Vector3& point, uint32_t point_id = 0);
  bool saveTeachPoints();
  bool loadTeachPoints();

  // Callback'и для событий
  typedef std::function<void(const RobotState&)> StateUpdateCallback;
  typedef std::function<void(const CommandResult&)> CommandCompleteCallback;
  typedef std::function<void(Mode, Mode)> ModeChangeCallback;
  typedef std::function<void(uint16_t, const std::string&)> ErrorCallback;

  void setStateUpdateCallback(StateUpdateCallback callback);
  void setCommandCompleteCallback(CommandCompleteCallback callback);
  void setModeChangeCallback(ModeChangeCallback callback);
  void setErrorCallback(ErrorCallback callback);

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
  bool is_moving_;
  bool is_paused_;

  // Управление командами
  CircularBuffer<Command, 64> command_queue_;
  Command current_command_;
  CommandStatus current_command_status_;
  uint32_t current_command_id_;
  uint32_t current_command_start_time_;

  std::array<CommandResult, 16> command_history_;
  uint8_t command_history_index_;

  // Траектории
  std::vector<Vector3> trajectory_points_;
  uint32_t current_trajectory_point_;
  bool trajectory_in_progress_;

  // Обучение
  std::map<uint32_t, Vector3> teach_points_;

  // Callback'и
  StateUpdateCallback state_update_callback_;
  CommandCompleteCallback command_complete_callback_;
  ModeChangeCallback mode_change_callback_;
  ErrorCallback error_callback_;

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
  bool convertToJointAngles(const Vector3& point, std::array<float, 3>& angles);
  bool checkPointSafety(const Vector3& point);
  bool checkJointSafety(const std::array<float, 3>& angles);
  void logCommand(const Command& cmd, CommandStatus status,
                  uint16_t error_code = 0, const std::string& error_msg = "");

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
