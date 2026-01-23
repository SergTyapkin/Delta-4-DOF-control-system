#pragma once

#include "../Core/Core.h"
#include "utils/CircularBuffer.h"
#include "CommandParser.h"
#include <functional>
#include <map>

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

  // Команды UI
  struct UICommand {
    enum class Type {
      SYSTEM,         // Системные команды
      MOTION,         // Команды движения
      CONFIG,         // Команды конфигурации
      QUERY,          // Запросы состояния
      PROGRAM,        // Управление программами
      EMERGENCY       // Аварийные команды
    };

    Type type;
    String name;
    String description;
    std::function<bool(const String&)> handler;
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

  // Регистрация команд
  void registerCommand(const String& name, const String& description,
                       std::function<bool(const String&)> handler,
                       UICommand::Type type = UICommand::Type::SYSTEM);

  // Управление подписками
  typedef std::function<void(const RobotState&)> StateSubscriptionCallback;
  void subscribeToState(StateSubscriptionCallback callback);
  void unsubscribeFromState();

  // Получение статистики
  uint32_t getCommandsProcessed() const { return commands_processed_; }
  uint32_t getErrorsCount() const { return errors_count_; }

  // Сохранение/загрузка конфигурации
  bool saveConfig();
  bool loadConfig();

  // Диагностика
  void printHelp() const;
  void printStatus() const;
  void listCommands() const;

private:
  // Указатель на ядро робота
  Core* robot_core_;

  // Конфигурация
  Config config_;

  // Состояние
  Mode mode_;
  bool is_initialized_;
  uint32_t last_update_time_;

  // Обработка команд
  CommandParser command_parser_;
  std::map<String, UICommand> commands_;
  CircularBuffer<String, 16> command_history_;

  // Статистика
  uint32_t commands_processed_;
  uint32_t errors_count_;

  // Подписки
  StateSubscriptionCallback state_subscription_;

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

  // Вспомогательные методы
  void printWelcomeMessage() const;
  void printPrompt() const;
  Vector3 parsePoint(const String& args) const;
  std::array<float, 3> parseJoints(const String& args) const;

  // Callback'и от Core
  void onStateUpdate(const RobotState& state);
  void onCommandComplete(const Core::CommandResult& result);
  void onError(uint16_t error_code, const String& error_message);
};
