#pragma once

#include <Arduino.h>  // для String, HardwareSerial, etc

#include "../../src/Core/Core.h"
#include "../../src/utils/CircularBuffer.h"

class UI;

// Замена std::function на указатели для экономии памяти
typedef bool (*CommandHandler)(UI* ui, const String& args);
typedef void (*StateCallback)(const RobotState& state);

class UI {
public:
  // Конфигурация UI
  struct Config {
    uint32_t update_interval;   // Интервал обновления (мс)
    uint32_t command_timeout;   // Таймаут команд (мс)
    bool echo_commands;         // Эхо-вывод команд
    bool debug_mode;            // Режим отладки

    Config() :
        update_interval(50),    // 20 Гц
        command_timeout(10000), // 10 секунд
        echo_commands(true),
        debug_mode(false) {}
  };

  // Режимы работы UI
  enum class Mode {
    INTERACTIVE,    // Интерактивный режим (команды с Serial)
    AUTONOMOUS,     // Автономный режим (выполнение программы)
    TEACHING,       // Режим обучения
    DIAGNOSTIC,     // Диагностический режим
    CONFIG          // Режим конфигурации
  };

  // Команды UI - упрощённая версия
  struct UICommand {
    enum Type {
      SYSTEM,         // Системные команды
      MOTION,         // Команды движения
      CONFIG,         // Команды конфигурации
      QUERY,          // Запросы состояния
      PROGRAM,        // Управление программами
      EMERGENCY       // Аварийные команды
    };

    Type type;
    const char* name;
    const char* description;
    CommandHandler handler;
  };

  // Конструктор
  UI();

  // Инициализация
  void init(Core* robot_core, const Config& config = Config());

  // Обновление состояния (вызывается периодически)
  void update();

  // Обработка входящих данных
  void processInput(const String& input);
  void processInput(const char* input, size_t length);

  // Отправка состояния
  void sendState(const RobotState& state);
  void sendMessage(const String& message, bool is_error = false);
  void sendResponse(const String& command, bool success,
                    const String& message = "");

  // Управление режимами
  void setMode(Mode mode);
  Mode getMode() const { return mode_; }

  // Регистрация команд - упрощённая версия
  void registerCommand(const char* name, const char* description,
                       bool (*handler)(UI* ui, const String& args),
                       UICommand::Type type);

  // Управление подписками
  void subscribeToState(StateCallback callback);
  void unsubscribeFromState();

  // Получение статистики
  uint32_t getCommandsProcessed() const { return commands_processed_; }
  uint32_t getErrorsCount() const { return errors_count_; }

  // Диагностика
  static void printHelp();
  void printStatus();
  void listCommands();

private:
  // Указатель на ядро робота
  Core* robot_core_;

  // Конфигурация
  Config config_;

  // Состояние
  Mode mode_;
  bool is_initialized_;
  uint32_t last_update_time_;

  // Команды
  static const int MAX_COMMANDS = 20;
  UICommand commands_[MAX_COMMANDS];
  int command_count_;

  CircularBuffer<String, 8> command_history_;  // уменьшенный буфер

  // Статистика
  uint32_t commands_processed_;
  uint32_t errors_count_;

  // Подписки
  StateCallback state_subscription_;

  // Буферы ввода/вывода
  String input_buffer_;
  bool new_input_available_;

  // Приватные методы
  void setupDefaultCommands();
  bool executeCommand(const String& command, const String& args);
  void processSerialInput();

  // Обработчики команд по умолчанию
  bool handleHelp(const String& args);
  bool handleMove(const String& args);
  bool handleHome(const String& args);
  bool handleStop(const String& args);
  bool handleStatus(const String& args);
  bool handleConfig(const String& args);
  bool handleTeach(const String& args);
  bool handleRun(const String& args);
  bool handleEmergencyStop(const String& args);
  bool handleReset(const String& args);
  bool handleList(const String& args);
  bool handleMode(const String& args);

  // Статические обертки для передачи команд в регистрацию команд
  static bool handleHelpStaticWrapper(UI* ui, const String& args);
  static bool handleMoveStaticWrapper(UI* ui, const String& args);
  static bool handleHomeStaticWrapper(UI* ui, const String& args);
  static bool handleStopStaticWrapper(UI* ui, const String& args);
  static bool handleStatusStaticWrapper(UI* ui, const String& args);
  static bool handleConfigStaticWrapper(UI* ui, const String& args);
  static bool handleTeachStaticWrapper(UI* ui, const String& args);
  static bool handleRunStaticWrapper(UI* ui, const String& args);
  static bool handleEmergencyStopStaticWrapper(UI* ui, const String& args);
  static bool handleResetStaticWrapper(UI* ui, const String& args);
  static bool handleListStaticWrapper(UI* ui, const String& args);
  static bool handleModeStaticWrapper(UI* ui, const String& args);

  // Вспомогательные методы
  static void printWelcomeMessage();
  void printPrompt();
  Vector3 parsePoint(const String& args) const;
  Vector3 parseJoints(const String& args) const;

  // Callback'и от Core
  void onStateUpdate(const RobotState& state);
  void onCommandComplete(const Core::CommandResult& result);
  void onError(uint16_t error_code, const String& error_message);
};
