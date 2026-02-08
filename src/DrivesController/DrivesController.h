#pragma once

#include "Drive.h"
#include "../../src/utils/CircularBuffer.h"
#include <array>

class DrivesController {
public:
  // Конфигурация контроллера
  struct Config {
    std::array<Drive::Config, 3> drive_configs;
    float sync_tolerance;          // Допуск синхронизации (рад)
    uint32_t sync_timeout;         // Таймаут синхронизации (мс)
    bool enable_sync_move;         // Включить синхронное движение
    bool stop_on_single_error;     // Останавливать все при ошибке одного

    Config() :
        sync_tolerance(0.01f),     // ~0.57 градуса
        sync_timeout(5000),        // 5 секунд
        enable_sync_move(true),
        stop_on_single_error(true) {}
  };

  // Режимы работы контроллера
  enum class Mode {
    INDEPENDENT,    // Каждый привод независим
    SYNCHRONIZED,   // Синхронизированное движение
    MASTER_SLAVE    // Ведущий-ведомый
  };

  // Состояния контроллера
  enum class State {
    IDLE,
    HOMING,
    MOVING,
    SYNCING,
    ERROR,
    EMERGENCY_STOP
  };

  // Команды управления
  struct Command {
    enum class Type {
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
    std::array<float, 3> positions;  // Позиции для осей (радианы)
    std::array<float, 3> velocities; // Скорости для осей (рад/с)
    float acceleration;               // Ускорение (рад/с²)
    uint32_t timeout;                 // Таймаут (мс)
    uint8_t drive_mask;               // Маска приводов (бит 0-2)

    Command() :
        type(Type::STOP),
        positions({0, 0, 0}),
        velocities({0, 0, 0}),
        acceleration(0),
        timeout(0),
        drive_mask(0b111) {} // Все приводы по умолчанию
  };

  // Конструктор
  DrivesController();

  // Инициализация
  bool init(const Config& config);

  // Обновление состояния (вызывается периодически)
  void update(uint32_t delta_time_ms);

  // Управление командами
  bool executeCommand(const Command& command);
  bool addCommandToQueue(const Command& command);
  void clearCommandQueue();

  // Прямое управление приводами
  bool moveToPosition(const std::array<float, 3>& positions,
                      float velocity = 0,
                      uint8_t drive_mask = 0b111);

  bool moveToPositionSync(const std::array<float, 3>& positions,
                          float velocity = 0);

  bool setVelocities(const std::array<float, 3>& velocities);

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
  std::array<float, 3> getPositions() const;
  std::array<float, 3> getVelocities() const;
  std::array<float, 3> getTargetPositions() const;

  // Получение состояния отдельных приводов
  Drive::State getDriveState(uint8_t index) const;
  bool isDriveEnabled(uint8_t index) const;
  bool isDriveHomed(uint8_t index) const;
  bool isDriveMoving(uint8_t index) const;

  // Статистика
  uint32_t getCommandsExecuted() const { return commands_executed_; }
  uint32_t getCommandsFailed() const { return commands_failed_; }
  uint32_t getQueueSize() const { return command_queue_.size(); }

  // Callback'и
  typedef std::function<void(State, State)> StateChangeCallback;
  typedef std::function<void(bool)> HomingCompleteCallback;
  typedef std::function<void(uint32_t)> CommandCompleteCallback;
  typedef std::function<void(uint8_t, Drive::State, Drive::State)> DriveStateCallback;

  void setStateChangeCallback(StateChangeCallback callback);
  void setHomingCompleteCallback(HomingCompleteCallback callback);
  void setCommandCompleteCallback(CommandCompleteCallback callback);
  void setDriveStateCallback(DriveStateCallback callback);

  // Диагностика
  void printStatus() const;
  void printDriveInfo(uint8_t index) const;

private:
  // Конфигурация
  Config config_;

  // Приводы
  std::array<Drive, 3> drives_;

  // Состояние
  State state_;
  State previous_state_;
  Mode mode_;

  // Очередь команд
  CircularBuffer<Command, 32> command_queue_;
  Command current_command_;
  bool command_in_progress_;
  uint32_t command_start_time_;

  // Синхронизация
  bool sync_in_progress_;
  uint32_t sync_start_time_;
  std::array<bool, 3> sync_drives_done_;

  // Статистика
  uint32_t commands_executed_;
  uint32_t commands_failed_;
  uint32_t last_update_time_;

  // Callback'и
  StateChangeCallback state_change_callback_;
  HomingCompleteCallback homing_complete_callback_;
  CommandCompleteCallback command_complete_callback_;
  DriveStateCallback drive_state_callback_;

  // Приватные методы
  void processCommandQueue();
  void executeCurrentCommand();
  void updateSyncMove();
  void checkSyncCompletion();

  // Обработчики событий приводов
  void onDriveStateChanged(uint8_t index, Drive::State old_state,
                           Drive::State new_state);
  void onDrivePositionUpdated(uint8_t index, float position, float velocity);
  void onDriveHomingComplete(uint8_t index, bool success);

  // Вспомогательные методы
  bool checkDrivesReady() const;
  bool checkPositionsValid(const std::array<float, 3>& positions) const;
  bool checkVelocitiesValid(const std::array<float, 3>& velocities) const;
  void updateControllerState();

  // Логирование
  void logCommand(const Command& cmd, bool success = true) const;
};
