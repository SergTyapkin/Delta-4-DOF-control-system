#pragma once

#include <Arduino.h>
#include "Drive.h"
#include "../../src/utils/CircularBuffer.h"
#include "../../config/robot_params.h"
#include "../../config/limits.h"

class DrivesController {
public:
  // Конфигурация контроллера
  struct Config {
    Drive::Config drive_configs[RobotParams::MOTORS_COUNT];  // обычный массив вместо std::array
    float sync_tolerance;          // Допуск синхронизации (рад)
    uint32_t sync_timeout;         // Таймаут синхронизации (мс)
    bool enable_sync_move;         // Включить синхронное движение
    bool stop_on_single_error;     // Останавливать все при ошибке одного

    Config() :
        sync_tolerance(Limits::TOLERANCE.angular_tolerance),
        sync_timeout(Limits::TIME.controller_sync_timeout),
        enable_sync_move(true),
        stop_on_single_error(true) {}
  };

  // Режимы работы контроллера
  enum Mode {
    INDEPENDENT,    // Каждый привод независим
    SYNCHRONIZED    // Синхронизированное движение
  };

  // Состояния контроллера
  enum State {
    IDLE,
    HOMING,
    MOVING,
    SYNCING,
    ERROR,
    EMERGENCY_STOP
  };

  // Команды управления
  struct Command {
    enum Type {
      MOVE_TO_POSITION,      // Движение в позицию
      MOVE_TO_POSITION_SYNC, // Синхронное движение
      SET_VELOCITY,          // Установка скорости
      HOME,                  // Homing
      ENABLE,                // Включить приводы
      DISABLE,               // Отключить приводы
      STOP,                  // Остановка
      EMERGENCY_STOP         // Аварийная остановка
    };

    Type type;
    float positions[RobotParams::MOTORS_COUNT];      // Позиции для осей (радианы)
    float velocities[RobotParams::MOTORS_COUNT];     // Скорости для осей (рад/с)
    float acceleration;      // Ускорение (рад/с²)
    uint32_t timeout;        // Таймаут (мс)

    Command() : type(STOP), acceleration(0), timeout(0) {
      positions[0] = positions[1] = positions[2] = positions[3] = 0;
      velocities[0] = velocities[1] = velocities[2] = velocities[3] = 0;
    }
  };

  // Конструктор
  DrivesController();

  // Инициализация
  bool init(const Config& config);

  // Обновление состояния
  void update(uint32_t delta_time_ms);

  // Управление командами
  bool executeCommand(const Command& command);
  bool addCommandToQueue(const Command& command);
  void clearCommandQueue();

  // Прямое управление приводами
  bool moveToPosition(const float positions[RobotParams::MOTORS_COUNT], float velocity = 0);
  bool moveToPositionSync(const float positions[RobotParams::MOTORS_COUNT], float velocity = 0);
  bool setVelocities(const float velocities[RobotParams::MOTORS_COUNT]);

  // Homing
  bool homeAll();
  bool homeSingle(uint8_t drive_index);
  bool isHomingComplete() const;
  bool isAllHomed() const;

  // Управление питанием
  void enableAll();
  void disableAll();

  // Аварийные функции
  void emergencyStop();
  void softStop();
  bool resetErrors();

  // Получение состояния
  State getState() const { return state_; }
  Mode getMode() const { return mode_; }
  void getPositions(float positions[RobotParams::MOTORS_COUNT]) const;
  void getVelocities(float velocities[RobotParams::MOTORS_COUNT]) const;
  void getTargetPositions(float targets[RobotParams::MOTORS_COUNT]) const;

  // Получение состояния отдельных приводов
  Drive::State getDriveState(uint8_t index) const;
  bool isDriveEnabled(uint8_t index) const;
  bool isDriveHomed(uint8_t index) const;
  bool isDriveMoving(uint8_t index) const;
  bool isMoving() const;

  // Статистика
  uint32_t getCommandsExecuted() const { return commands_executed_; }
  uint32_t getCommandsFailed() const { return commands_failed_; }
  uint32_t getQueueSize() const { return command_queue_.size(); }

  // Callback'и - указатели на функции
  typedef void (*StateChangeCallback)(State, State, void*);
  typedef void (*HomingCompleteCallback)(bool, void*);
  typedef void (*CommandCompleteCallback)(uint32_t, void*);
  typedef void (*DriveStateCallback)(uint8_t, Drive::State, Drive::State, void*);

  void setStateChangeCallback(StateChangeCallback callback, void* context);
  void setHomingCompleteCallback(HomingCompleteCallback callback, void* context);
  void setCommandCompleteCallback(CommandCompleteCallback callback, void* context);
  void setDriveStateCallback(DriveStateCallback callback, void* context);

  // Диагностика
  void printStatus() const;
  void printDriveInfo(uint8_t index) const;

private:
  // Конфигурация
  Config config_;

  // Приводы
  Drive drives_[RobotParams::MOTORS_COUNT];

  // Состояние
  State state_;
  State previous_state_;
  Mode mode_;

  // Очередь команд
  CircularBuffer<Command, 16> command_queue_;  // фиксированный размер
  Command current_command_;
  bool command_in_progress_;
  uint32_t command_start_time_;

  // Синхронизация
  bool sync_in_progress_;
  uint32_t sync_start_time_;
  bool sync_drives_done_[RobotParams::MOTORS_COUNT];

  // Статистика
  uint32_t commands_executed_;
  uint32_t commands_failed_;
  uint32_t last_update_time_;

  // Callback'и и их context'ы
  StateChangeCallback state_change_callback_;
  HomingCompleteCallback homing_complete_callback_;
  CommandCompleteCallback command_complete_callback_;
  DriveStateCallback drive_state_callback_;
  void* state_change_callback_context_;
  void* homing_complete_callback_context_;
  void* command_complete_callback_context_;
  void* drive_state_callback_context_;

  // Приватные методы
  void processCommandQueue();
  void updateSyncMove();
  void checkSyncCompletion();

  // Обработчики событий приводов
  void onDriveStateChanged(uint8_t index, Drive::State old_state,
                           Drive::State new_state);
  void onDrivePositionUpdated(uint8_t index, float position, float velocity);
  void onDriveHomingComplete(uint8_t index, bool success);

  // Вспомогательные методы
  bool checkDrivesReady() const;
  bool checkPositionsValid(const float positions[RobotParams::MOTORS_COUNT]) const;
  bool checkVelocitiesValid(const float velocities[RobotParams::MOTORS_COUNT]) const;
  void updateControllerState();
};
