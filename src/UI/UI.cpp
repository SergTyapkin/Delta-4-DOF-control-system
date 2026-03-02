#include "UI.h"
#include "../../src/utils/Logger.h"
#include "../../src/utils/MathUtils.h"

UserInterface::UserInterface() :
    robot_core_(nullptr),
    mode_(Mode::INTERACTIVE),
    is_initialized_(false),
    last_update_time_(0),
    command_count_(0),
    commands_processed_(0),
    errors_count_(0),
    state_subscription_(nullptr),
    new_input_available_(false) {
}

void UserInterface::init(Core* robot_core, const Config& config) {
  if (!robot_core) {
    Logger::error("UI: Robot core is null");
    return;
  }

  robot_core_ = robot_core;
  config_ = config;

  // Настройка Serial
  if (!Serial) {
    Serial.begin(115200);
    delay(100);
  }

  // Регистрация команд по умолчанию
  setupDefaultCommands();

  is_initialized_ = true;
  last_update_time_ = millis();

  printWelcomeMessage();
  printHelp();
}

void UserInterface::update() {
  if (!is_initialized_) return;

  uint32_t current_time = millis();

  // Обработка Serial ввода
  processSerialInput();

  // Периодические обновления
  if (current_time - last_update_time_ >= config_.update_interval) {
    last_update_time_ = current_time;
  }
}

void UserInterface::processInput(const String& input) {
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
  int space_pos = input.indexOf(' ');
  String command, args;

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

  // Выводим приглашение
  if (mode_ == Mode::INTERACTIVE) {
    printPrompt();
  }
}

void UserInterface::processInput(const char* input, size_t length) {
  if (length == 0) return;

  String input_str = String(input);
  input_str.trim();

  if (input_str.length() > 0) {
    processInput(input_str);
  }
}

void UserInterface::sendState(const RobotState& state) {
  if (state_subscription_) {
    state_subscription_(state);
  }

  // Также отправляем в Serial для отладки
  if (config_.debug_mode) {
    Serial.println(state.serialize().c_str());
  }
}

void UserInterface::sendMessage(const String& message, bool is_error) {
  if (is_error) {
    Serial.print("[ERROR] ");
  } else {
    Serial.print("[INFO] ");
  }
//  Serial.println(message);
}

void UserInterface::sendResponse(const String& command, bool success, const String& message) {
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

void UserInterface::setMode(Mode mode) {
  if (mode_ == mode) return;

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

void UserInterface::registerCommand(const char* name, const char* description,
                         bool (*handler)(UserInterface* UI, const String& args),
                         UICommand::Type type) {
  if (command_count_ >= MAX_COMMANDS) {
    Logger::error("UI: Too many commands");
    return;
  }

  UICommand cmd;
  cmd.type = type;
  cmd.name = name;
  cmd.description = description;
  cmd.handler = handler;

  commands_[command_count_++] = cmd;

  Logger::debug("Command registered: %s", name);
}

void UserInterface::subscribeToState(StateCallback callback) {
  state_subscription_ = callback;
}

void UserInterface::unsubscribeFromState() {
  state_subscription_ = nullptr;
}

void UserInterface::printHelp() {
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

void UserInterface::printStatus() {
  Serial.println("=== UI Status ===");

  String mode_str;
  switch (mode_) {
    case Mode::INTERACTIVE: mode_str = "INTERACTIVE"; break;
    case Mode::AUTONOMOUS: mode_str = "AUTONOMOUS"; break;
    case Mode::TEACHING: mode_str = "TEACHING"; break;
    case Mode::DIAGNOSTIC: mode_str = "DIAGNOSTIC"; break;
    case Mode::CONFIG: mode_str = "CONFIG"; break;
  }

  Serial.print("Mode: ");
  Serial.println(mode_str);

  Serial.print("Commands processed: ");
  Serial.println(commands_processed_);

  Serial.print("Errors: ");
  Serial.println(errors_count_);
}

void UserInterface::listCommands() {
  Serial.println("=== Registered Commands ===");

  for (int i = 0; i < command_count_; i++) {
    const UICommand& cmd = commands_[i];
    Serial.print(cmd.name);
    Serial.print(" - ");
    Serial.println(cmd.description);
  }

  Serial.print("Total commands: ");
  Serial.println(command_count_);
}

void UserInterface::setupDefaultCommands() {
  // Регистрация команд по умолчанию
  registerCommand("HELP", "Show help", handleHelpStaticWrapper, UICommand::SYSTEM);
  registerCommand("MOVE", "Move to point X Y Z [AX AY AZ] [VELOCITY]", handleMoveStaticWrapper, UICommand::MOTION);
  registerCommand("HOME", "Perform homing", handleHomeStaticWrapper, UICommand::MOTION);
  registerCommand("STOP", "Stop movement", handleStopStaticWrapper, UICommand::MOTION);
  registerCommand("ESTOP", "Emergency stop", handleEmergencyStopStaticWrapper, UICommand::EMERGENCY);
  registerCommand("STATUS", "Show robot status", handleStatusStaticWrapper, UICommand::QUERY);
  registerCommand("CONFIG", "Show configuration", handleConfigStaticWrapper, UICommand::CONFIG);
  registerCommand("TEACH", "Teach point X Y Z [AX AY AZ] [ID]", handleTeachStaticWrapper, UICommand::PROGRAM);
  registerCommand("RUN", "Run program [NAME]", handleRunStaticWrapper, UICommand::PROGRAM);
  registerCommand("RESET", "Reset errors", handleResetStaticWrapper, UICommand::SYSTEM);
  registerCommand("LIST", "List all commands", handleListStaticWrapper, UICommand::SYSTEM);
  registerCommand("MODE", "Set UI mode [MODE]", handleModeStaticWrapper, UICommand::SYSTEM);
}

bool UserInterface::executeCommand(const String& command, const String& args) {
  for (int i = 0; i < command_count_; i++) {
    if (command == commands_[i].name) {
      return commands_[i].handler(this, args);
    }
  }
  return false;
}

void UserInterface::processSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (input_buffer_.length() > 0) {
        new_input_available_ = true;
      }
    } else if (c == 0x08 || c == 0x7F) { // Backspace or Delete
      if (input_buffer_.length() > 0) {
        input_buffer_.remove(input_buffer_.length() - 1);
        Serial.print("\b \b");
      }
    } else if (c >= 32 && c <= 126) { // Printable characters
      input_buffer_ += c;
      if (config_.echo_commands) {
        Serial.print(c);
      }
    }
  }

  if (new_input_available_) {
    processInput(input_buffer_);
    input_buffer_ = "";
    new_input_available_ = false;
  }
}

bool UserInterface::handleHelp(const String& args) {
  printHelp();
  return true;
}

bool UserInterface::handleMove(const String& args) {
  if (!robot_core_->isReady()) {
    sendResponse("MOVE", false, "Robot not ready");
    return false;
  }

  Vector6 position(0, 0, 0, 0, 0, 0);
  float velocity = 0;

  char* token = strtok((char*)args.c_str(), " ");
  int count = 0;

  while (token != nullptr && count < 7) {
    float val = atof(token);
    if (count < 3) {
      (&position.x)[count] = val;
    } else if (count < 6) {
      (&position.ax)[count - 3] = val * MathUtils::DEG_TO_RAD;
    } else {
      velocity = val;
    }
    token = strtok(nullptr, " ");
    count++;
  }

  if (count < 3) {
    sendResponse("MOVE", false, "Need X Y Z coordinates");
    return false;
  }

  bool success = robot_core_->moveToPosition(position, velocity);
  sendResponse("MOVE", success, success ? "Movement started" : "Failed to start movement");
  return success;
}

bool UserInterface::handleHome(const String& args) {
  bool success = robot_core_->performHoming();
  sendResponse("HOME", success, success ? "Homing started" : "Failed to start homing");
  return success;
}

bool UserInterface::handleStop(const String& args) {
  robot_core_->stop();
  sendResponse("STOP", true, "Stopped");
  return true;
}

bool UserInterface::handleStatus(const String& args) {
  RobotState state = robot_core_->getState();
  state.print();
  return true;
}

bool UserInterface::handleConfig(const String& args) {
  robot_core_->printStatus();
  robot_core_->printKinematicsInfo();
  return true;
}

bool UserInterface::handleTeach(const String& args) {
  Vector6 position(0, 0, 0, 0, 0, 0);
  uint32_t point_id = 0;

  char* token = strtok((char*)args.c_str(), " ");
  int count = 0;

  while (token != nullptr && count < 7) {
    float val = atof(token);
    if (count < 3) {
      (&position.x)[count] = val;
    } else if (count < 6) {
      (&position.ax)[count - 3] = val * MathUtils::DEG_TO_RAD;
    } else {
      point_id = atoi(token);
    }
    token = strtok(nullptr, " ");
    count++;
  }

  if (count < 3) {
    sendResponse("TEACH", false, "Need X Y Z coordinates");
    return false;
  }

  bool success = robot_core_->teachPosition(position, point_id);
  sendResponse("TEACH", success, success ? "Point taught" : "Failed to teach point");
  return success;
}

bool UserInterface::handleRun(const String& args) {
  sendResponse("RUN", false, "Not implemented yet");
  return false;
}

bool UserInterface::handleEmergencyStop(const String& args) {
  robot_core_->emergencyStop();
  sendResponse("ESTOP", true, "Emergency stop activated");
  return true;
}

bool UserInterface::handleReset(const String& args) {
  bool success = robot_core_->reset();
  sendResponse("RESET", success, success ? "Reset successful" : "Reset failed");
  return success;
}

bool UserInterface::handleList(const String& args) {
  listCommands();
  return true;
}

bool UserInterface::handleMode(const String& args) {
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

// Статические обертки для передачи команд в регистрацию команд
bool UserInterface::handleHelpStaticWrapper(UserInterface* UI, const String& args) {return UI->handleHelp(args);}
bool UserInterface::handleMoveStaticWrapper(UserInterface* UI, const String& args) {return UI->handleMove(args);}
bool UserInterface::handleHomeStaticWrapper(UserInterface* UI, const String& args) {return UI->handleHome(args);}
bool UserInterface::handleStopStaticWrapper(UserInterface* UI, const String& args) {return UI->handleStop(args);}
bool UserInterface::handleStatusStaticWrapper(UserInterface* UI, const String& args) {return UI->handleStatus(args);}
bool UserInterface::handleConfigStaticWrapper(UserInterface* UI, const String& args) {return UI->handleConfig(args);}
bool UserInterface::handleTeachStaticWrapper(UserInterface* UI, const String& args) {return UI->handleTeach(args);}
bool UserInterface::handleRunStaticWrapper(UserInterface* UI, const String& args) {return UI->handleRun(args);}
bool UserInterface::handleEmergencyStopStaticWrapper(UserInterface* UI, const String& args) {return UI->handleEmergencyStop(args);}
bool UserInterface::handleResetStaticWrapper(UserInterface* UI, const String& args) {return UI->handleReset(args);}
bool UserInterface::handleListStaticWrapper(UserInterface* UI, const String& args) {return UI->handleList(args);}
bool UserInterface::handleModeStaticWrapper(UserInterface* UI, const String& args) {return UI->handleMode(args);}


void UserInterface::printWelcomeMessage() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("    DELTA ROBOT CONTROL SYSTEM v1.0");
  Serial.println("========================================");
  Serial.println();
}

void UserInterface::printPrompt() {
  if (mode_ == Mode::INTERACTIVE) {
    Serial.print("> ");
    Serial.flush();
  }
}

void UserInterface::onStateUpdate(const RobotState& state) {
  sendState(state);
}

void UserInterface::onCommandComplete(const Core::CommandResult& result) {
  switch (result.status) {
    case Core::STATUS_COMPLETED:
      sendMessage(String("Command ") + result.command_id + " completed", false);
      break;

    case Core::STATUS_FAILED:
      sendMessage(String("Command ") + result.command_id + " failed: " + result.error_message, true);
      break;

    case Core::STATUS_CANCELLED:
      sendMessage(String("Command ") + result.command_id + " cancelled", false);
      break;

    default:
      break;
  }
}

void UserInterface::onError(uint16_t error_code, const String& error_message) {
  sendMessage(String("Error ") + error_code + ": " + error_message, true);
}
