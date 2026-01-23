// src/UI/UI.cpp
#include "UI.h"
#include "utils/Logger.h"
#include <Arduino.h>

UI::UI() :
    robot_core_(nullptr),
    mode_(Mode::INTERACTIVE),
    is_initialized_(false),
    last_update_time_(0),
    commands_processed_(0),
    errors_count_(0),
    new_input_available_(false) {
}

void UI::init(Core* robot_core, const Config& config) {
  if (!robot_core) {
    Logger::error("UI: Robot core is null");
    return;
  }

  robot_core_ = robot_core;
  config_ = config;

  // Настройка Serial (если еще не настроен)
  if (!Serial) {
    Serial.begin(115200);
    while (!Serial) {
      delay(10);
    }
  }

  // Настройка парсера команд
  command_parser_.setDelimiter(' ');
  command_parser_.setMaxArgs(10);

  // Регистрация команд по умолчанию
  setupDefaultCommands();

  // Подписка на события ядра
  robot_core_->setStateUpdateCallback(
      [this](const RobotState& state) {
        onStateUpdate(state);
      }
  );

  robot_core_->setCommandCompleteCallback(
      [this](const Core::CommandResult& result) {
        onCommandComplete(result);
      }
  );

  robot_core_->setErrorCallback(
      [this](uint16_t error_code, const String& error_message) {
        onError(error_code, error_message);
      }
  );

  is_initialized_ = true;
  last_update_time_ = millis();

  printWelcomeMessage();
  printHelp();
}

void UI::update() {
  if (!is_initialized_) return;

  uint32_t current_time = millis();

  // Обработка Serial ввода
  processSerialInput();

  // Периодические обновления
  if (current_time - last_update_time_ >= config_.update_interval) {
    // Можно добавить периодические задачи здесь
    last_update_time_ = current_time;
  }

  // Проверка состояния ядра робота
  static uint32_t last_status_check = 0;
  if (current_time - last_status_check > 1000) { // Раз в секунду
    if (robot_core_->isReady()) {
      // Робот готов
    } else if (robot_core_->getMode() == Core::Mode::ERROR) {
      sendMessage("Robot in ERROR state!", true);
    }
    last_status_check = current_time;
  }
}

void UI::processInput(const String& input) {
  if (!is_initialized_ || input.length() == 0) {
    return;
  }

  if (config_.echo_commands) {
    Serial.print("> ");
    Serial.println(input);
  }

  // Добавляем в историю
  command_history_.push(input);

  // Парсим команду
  String command;
  String args;

  int space_pos = input.indexOf(' ');
  if (space_pos > 0) {
    command = input.substring(0, space_pos);
    args = input.substring(space_pos + 1);
  } else {
    command = input;
    args = "";
  }

  command.toUpperCase();

  // Выполняем команду
  bool success = executeCommand(command, args);

  if (!success) {
    sendResponse(command, false, "Unknown command or invalid arguments");
    errors_count_++;
  }

  commands_processed_++;

  // Выводим приглашение для следующей команды
  if (mode_ == Mode::INTERACTIVE) {
    printPrompt();
  }
}

void UI::processInput(const char* input, size_t length) {
  if (length == 0) return;

  String input_str(input, length);
  input_str.trim();

  if (input_str.length() > 0) {
    processInput(input_str);
  }
}

void UI::sendState(const RobotState& state) {
  if (state_subscription_) {
    state_subscription_(state);
  }

  // Также отправляем в Serial для отладки
  if (config_.debug_mode) {
    Serial.println(state.serialize());
  }
}

void UI::sendMessage(const String& message, bool is_error) {
  if (is_error) {
    Serial.print("[ERROR] ");
  } else {
    Serial.print("[INFO] ");
  }
  Serial.println(message);
}

void UI::sendResponse(const String& command, bool success, const String& message) {
  Serial.print("[");
  Serial.print(command);
  Serial.print("] ");

  if (success) {
    Serial.print("OK");
  } else {
    Serial.print("ERROR");
  }

  if (message.length() > 0) {
    Serial.print(": ");
    Serial.print(message);
  }

  Serial.println();
}

void UI::setMode(Mode mode) {
  if (mode_ == mode) return;

  Mode old_mode = mode_;
  mode_ = mode;

  String mode_str;
  switch (mode_) {
    case Mode::INTERACTIVE: mode_str = "INTERACTIVE"; break;
    case Mode::AUTONOMOUS: mode_str = "AUTONOMOUS"; break;
    case Mode::TEACHING: mode_str = "TEACHING"; break;
    case Mode::DIAGNOSTIC: mode_str = "DIAGNOSTIC"; break;
    case Mode::CONFIG: mode_str = "CONFIG"; break;
  }

  sendMessage("Mode changed to " + mode_str);
}

void UI::registerCommand(const String& name, const String& description,
                         std::function<bool(const String&)> handler,
                         UICommand::Type type) {

  UICommand cmd;
  cmd.type = type;
  cmd.name = name.toUpperCase();
  cmd.description = description;
  cmd.handler = handler;

  commands_[cmd.name] = cmd;

  Logger::debug("Command registered: %s", cmd.name.c_str());
}

void UI::subscribeToState(StateSubscriptionCallback callback) {
  state_subscription_ = callback;
}

void UI::unsubscribeFromState() {
  state_subscription_ = nullptr;
}

bool UI::saveConfig() {
  // Здесь должна быть реализация сохранения конфигурации
  sendMessage("Configuration saved");
  return true;
}

bool UI::loadConfig() {
  // Здесь должна быть реализация загрузки конфигурации
  sendMessage("Configuration loaded");
  return true;
}

void UI::printHelp() const {
  Serial.println("=== Delta Robot Control System ===");
  Serial.println("Available commands:");
  Serial.println("  help                    - Show this help");
  Serial.println("  move X Y Z [V]          - Move to point (mm) with optional velocity");
  Serial.println("  joints A1 A2 A3 [V]     - Move joints (deg) with optional velocity");
  Serial.println("  home                    - Perform homing sequence");
  Serial.println("  stop                    - Stop movement");
  Serial.println("  estop                   - Emergency stop");
  Serial.println("  status                  - Show robot status");
  Serial.println("  config                  - Show configuration");
  Serial.println("  teach X Y Z [ID]        - Teach point (save position)");
  Serial.println("  run [PROGRAM]           - Run program");
  Serial.println("  reset                   - Reset errors");
  Serial.println("  list                    - List all commands");
  Serial.println("  mode [MODE]             - Set UI mode");
  Serial.println("");
  Serial.println("Examples:");
  Serial.println("  move 100 50 -300 50     - Move to (100,50,-300) at 50 mm/s");
  Serial.println("  joints 30 45 60         - Move joints to 30°, 45°, 60°");
  Serial.println("  teach 0 0 -400          - Teach home position");
}

void UI::printStatus() const {
  Serial.println("=== UI Status ===");

  String mode_str;
  switch (mode_) {
    case Mode::INTERACTIVE: mode_str = "INTERACTIVE"; break;
    case Mode::AUTONOMOUS: mode_str = "AUTONOMOUS"; break;
    case Mode::TEACHING: mode_str = "TEACHING"; break;
    case Mode::DIAGNOSTIC: mode_str = "DIAGNOSTIC"; break;
    case Mode::CONFIG: mode_str = "CONFIG"; break;
  }

  Serial.printf("Mode: %s\n", mode_str.c_str());
  Serial.printf("Commands processed: %d\n", commands_processed_);
  Serial.printf("Errors: %d\n", errors_count_);
  Serial.printf("Command history: %d/%d\n",
                command_history_.size(), command_history_.capacity());
}

void UI::listCommands() const {
  Serial.println("=== Registered Commands ===");

  for (const auto& pair : commands_) {
    const UICommand& cmd = pair.second;

    Serial.printf("%-20s - %s\n",
                  cmd.name.c_str(),
                  cmd.description.c_str());
  }

  Serial.printf("Total: %d commands\n", commands_.size());
}

void UI::setupDefaultCommands() {
  // Регистрация команд по умолчанию
  registerCommand("HELP", "Show help",
                  [this](const String& args) { return handleHelp(args); },
                  UICommand::Type::SYSTEM);

  registerCommand("MOVE", "Move to point X Y Z [VELOCITY]",
                  [this](const String& args) { return handleMove(args); },
                  UICommand::Type::MOTION);

  registerCommand("JOINTS", "Move joints A1 A2 A3 [VELOCITY]",
                  [this](const String& args) { return handleMove(args); },
                  UICommand::Type::MOTION);

  registerCommand("HOME", "Perform homing",
                  [this](const String& args) { return handleHome(args); },
                  UICommand::Type::MOTION);

  registerCommand("STOP", "Stop movement",
                  [this](const String& args) { return handleStop(args); },
                  UICommand::Type::MOTION);

  registerCommand("ESTOP", "Emergency stop",
                  [this](const String& args) { return handleEmergencyStop(args); },
                  UICommand::Type::EMERGENCY);

  registerCommand("STATUS", "Show robot status",
                  [this](const String& args) { return handleStatus(args); },
                  UICommand::Type::QUERY);

  registerCommand("CONFIG", "Show configuration",
                  [this](const String& args) { return handleConfig(args); },
                  UICommand::Type::CONFIG);

  registerCommand("TEACH", "Teach point X Y Z [ID]",
                  [this](const String& args) { return handleTeach(args); },
                  UICommand::Type::PROGRAM);

  registerCommand("RUN", "Run program [NAME]",
                  [this](const String& args) { return handleRun(args); },
                  UICommand::Type::PROGRAM);

  registerCommand("RESET", "Reset errors",
                  [this](const String& args) { return handleReset(args); },
                  UICommand::Type::SYSTEM);

  registerCommand("LIST", "List all commands",
                  [this](const String& args) { return handleList(args); },
                  UICommand::Type::SYSTEM);

  registerCommand("MODE", "Set UI mode [MODE]",
                  [this](const String& args) { return handleMode(args); },
                  UICommand::Type::SYSTEM);
}

bool UI::executeCommand(const String& command, const String& args) {
  auto it = commands_.find(command);
  if (it == commands_.end()) {
    return false;
  }

  try {
    return it->second.handler(args);
  } catch (...) {
    sendResponse(command, false, "Command execution failed");
    return false;
  }
}

void UI::processSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input_buffer_.length() > 0) {
        new_input_available_ = true;
      }
    } else if (c == 0x08 || c == 0x7F) { // Backspace or Delete
      if (input_buffer_.length() > 0) {
        input_buffer_.remove(input_buffer_.length() - 1);
        // Эхо backspace
        Serial.print("\b \b");
      }
    } else if (c >= 32 && c <= 126) { // Printable characters
      input_buffer_ += c;
      if (config_.echo_commands) {
        Serial.print(c);
      }
    }
    // Игнорируем другие символы
  }

  if (new_input_available_) {
    processInput(input_buffer_);
    input_buffer_.clear();
    new_input_available_ = false;
  }
}

bool UI::handleHelp(const String& args) {
  printHelp();
  return true;
}

bool UI::handleMove(const String& args) {
  if (!robot_core_->isReady()) {
    sendResponse("MOVE", false, "Robot not ready");
    return false;
  }

  command_parser_.parse(args);
  auto tokens = command_parser_.getTokens();

  if (tokens.size() < 3) {
    sendResponse("MOVE", false, "Need X Y Z coordinates");
    return false;
  }

  float x = tokens[0].toFloat();
  float y = tokens[1].toFloat();
  float z = tokens[2].toFloat();

  float velocity = 0;
  if (tokens.size() >= 4) {
    velocity = tokens[3].toFloat();
  }

  Vector3 point(x, y, z);
  bool success = robot_core_->moveToPoint(point, velocity);

  sendResponse("MOVE", success,
               success ? "Movement started" : "Failed to start movement");

  return success;
}

bool UI::handleHome(const String& args) {
  bool success = robot_core_->performHoming();

  sendResponse("HOME", success,
               success ? "Homing started" : "Failed to start homing");

  return success;
}

bool UI::handleStop(const String& args) {
  robot_core_->stop();
  sendResponse("STOP", true, "Stopped");
  return true;
}

bool UI::handleStatus(const String& args) {
  RobotState state = robot_core_->getState();
  state.print();
  return true;
}

bool UI::handleConfig(const String& args) {
  robot_core_->printStatus();
  robot_core_->printKinematicsInfo();
  return true;
}

bool UI::handleTeach(const String& args) {
  command_parser_.parse(args);
  auto tokens = command_parser_.getTokens();

  if (tokens.size() < 3) {
    sendResponse("TEACH", false, "Need X Y Z coordinates");
    return false;
  }

  float x = tokens[0].toFloat();
  float y = tokens[1].toFloat();
  float z = tokens[2].toFloat();

  uint32_t point_id = 0;
  if (tokens.size() >= 4) {
    point_id = tokens[3].toInt();
  }

  Vector3 point(x, y, z);
  bool success = robot_core_->teachPoint(point, point_id);

  sendResponse("TEACH", success,
               success ? "Point taught" : "Failed to teach point");

  return success;
}

bool UI::handleRun(const String& args) {
  // Здесь должна быть реализация запуска программ
  sendResponse("RUN", false, "Not implemented yet");
  return false;
}

bool UI::handleEmergencyStop(const String& args) {
  robot_core_->emergencyStop();
  sendResponse("ESTOP", true, "Emergency stop activated");
  return true;
}

bool UI::handleReset(const String& args) {
  bool success = robot_core_->reset();
  sendResponse("RESET", success,
               success ? "Reset successful" : "Reset failed");
  return success;
}

bool UI::handleList(const String& args) {
  listCommands();
  return true;
}

bool UI::handleMode(const String& args) {
  if (args.length() == 0) {
    // Показать текущий режим
    String mode_str;
    switch (mode_) {
      case Mode::INTERACTIVE: mode_str = "INTERACTIVE"; break;
      case Mode::AUTONOMOUS: mode_str = "AUTONOMOUS"; break;
      case Mode::TEACHING: mode_str = "TEACHING"; break;
      case Mode::DIAGNOSTIC: mode_str = "DIAGNOSTIC"; break;
      case Mode::CONFIG: mode_str = "CONFIG"; break;
    }

    Serial.print("Current mode: ");
    Serial.println(mode_str);
    return true;
  }

  String mode_arg = args;
  mode_arg.toUpperCase();

  if (mode_arg == "INTERACTIVE") {
    setMode(Mode::INTERACTIVE);
  } else if (mode_arg == "AUTONOMOUS") {
    setMode(Mode::AUTONOMOUS);
  } else if (mode_arg == "TEACHING") {
    setMode(Mode::TEACHING);
  } else if (mode_arg == "DIAGNOSTIC") {
    setMode(Mode::DIAGNOSTIC);
  } else if (mode_arg == "CONFIG") {
    setMode(Mode::CONFIG);
  } else {
    sendResponse("MODE", false, "Unknown mode");
    return false;
  }

  return true;
}

Vector3 UI::parsePoint(const String& args) const {
  command_parser_.parse(args);
  auto tokens = command_parser_.getTokens();

  if (tokens.size() < 3) {
    return Vector3(0, 0, 0);
  }

  return Vector3(
      tokens[0].toFloat(),
      tokens[1].toFloat(),
      tokens[2].toFloat()
  );
}

std::array<float, 3> UI::parseJoints(const String& args) const {
  command_parser_.parse(args);
  auto tokens = command_parser_.getTokens();

  std::array<float, 3> joints = {0, 0, 0};

  if (tokens.size() >= 3) {
    joints[0] = tokens[0].toFloat() * (3.1415926535f / 180.0f);
    joints[1] = tokens[1].toFloat() * (3.1415926535f / 180.0f);
    joints[2] = tokens[2].toFloat() * (3.1415926535f / 180.0f);
  }

  return joints;
}

void UI::printWelcomeMessage() const {
  Serial.println();
  Serial.println("========================================");
  Serial.println("    DELTA ROBOT CONTROL SYSTEM v1.0");
  Serial.println("========================================");
  Serial.println();
}

void UI::printPrompt() const {
  if (mode_ == Mode::INTERACTIVE) {
    Serial.print("> ");
    Serial.flush();
  }
}

void UI::onStateUpdate(const RobotState& state) {
  sendState(state);
}

void UI::onCommandComplete(const Core::CommandResult& result) {
  switch (result.status) {
    case Core::CommandStatus::COMPLETED:
      sendMessage("Command " + String(result.command_id) + " completed", false);
      break;

    case Core::CommandStatus::FAILED:
      sendMessage("Command " + String(result.command_id) +
                  " failed: " + result.error_message, true);
      break;

    case Core::CommandStatus::CANCELLED:
      sendMessage("Command " + String(result.command_id) + " cancelled", false);
      break;

    default:
      break;
  }
}

void UI::onError(uint16_t error_code, const String& error_message) {
  sendMessage("Error " + String(error_code) + ": " + error_message, true);
}
