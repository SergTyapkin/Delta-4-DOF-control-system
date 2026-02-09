#pragma once

#include <Arduino.h>
#include <functional>
#include <map>
#include <string>
#include "CommandParser.h"
#include "../../src/Core/Core.h"
#include "../../src/utils/CircularBuffer.h"
#include "../../src/utils/Logger.h"

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
    std::string name;
    std::string description;
    std::function<bool(const std::string&)> handler;
  };

  // Конструктор
  UI();

  // Инициализация
  void init(Core* robot_core, const Config& config = Config());

  // Обновление состояния (вызывается периодически)
  void update();

  // Обработка входящих данных
  void processInput(const std::string& input);
  void processInput(const char* input, size_t length);

  // Отправка состояния
  void sendState(const RobotState& state);
  void sendMessage(const std::string& message, bool is_error = false);
  void sendResponse(const std::string& command, bool success,
                    const std::string& message = "");

  // Управление режимами
  void setMode(Mode mode);
  Mode getMode() const { return mode_; }

  // Регистрация команд
  void registerCommand(const std::string& name, const std::string& description,
                       std::function<bool(const std::string&)> handler,
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
  std::map<std::string, UICommand> commands_;
  CircularBuffer<std::string, 16> command_history_;

  // Статистика
  uint32_t commands_processed_;
  uint32_t errors_count_;

  // Подписки
  StateSubscriptionCallback state_subscription_;

  // Буферы ввода/вывода
  std::string input_buffer_;
  bool new_input_available_;

  // Приватные методы
  void setupDefaultCommands();
  bool executeCommand(const std::string& command, const std::string& args);
  void processSerialInput();

  // Обработчики команд по умолчанию
  bool handleHelp(const std::string& args);
  bool handleMove(const std::string& args);
  bool handleHome(const std::string& args);
  bool handleStop(const std::string& args);
  bool handleStatus(const std::string& args);
  bool handleConfig(const std::string& args);
  bool handleTeach(const std::string& args);
  bool handleRun(const std::string& args);
  bool handleEmergencyStop(const std::string& args);
  bool handleReset(const std::string& args);
  bool handleList(const std::string& args);
  bool handleMode(const std::string& args);

  // Вспомогательные методы
  void printWelcomeMessage() const;
  void printPrompt() const;
  Vector3 parsePoint(const std::string& args);
  std::array<float, 3> parseJoints(const std::string& args);

  // Callback'и от Core
  void onStateUpdate(const RobotState& state);
  void onCommandComplete(const Core::CommandResult& result);
  void onError(uint16_t error_code, const std::string& error_message);
};
