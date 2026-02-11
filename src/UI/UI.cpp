#include "UI.h"
#include "../../src/utils/Logger.h"
#include "../../src/utils/MathUtils.h"

UI::UI() :
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

void UI::init(Core* robot_core, const Config& config) {
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

void UI::update() {
  if (!is_initialized_) return;

  uint32_t current_time = millis();

  // Обработка Serial ввода
  processSerialInput();

  // Периодические обновления
  if (current_time - last_update_time_ >= config_.update_interval) {
    last_update_time_ = current_time;
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

void UI::processInput(const char* input, size_t length) {
  if (length == 0) return;

  String input_str = String(input);
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
    Serial.println(state.serialize().c_str());
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

void UI::registerCommand(const char* name, const char* description,
                         bool (*handler)(UI* ui, const String& args),
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

void UI::subscribeToState(StateCallback callback) {
  state_subscription_ = callback;
}

void UI::unsubscribeFromState() {
  state_subscription_ = nullptr;
}

void UI::printHelp() {
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

void UI::printStatus() {
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

void UI::listCommands() {
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

void UI::setupDefaultCommands() {
  // Регистрация команд по умолчанию
  registerCommand("HELP", "Show help", handleHelpStaticWrapper, UICommand::SYSTEM);
  registerCommand("MOVE", "Move to point X Y Z [VELOCITY]", handleMoveStaticWrapper, UICommand::MOTION);
  registerCommand("HOME", "Perform homing", handleHomeStaticWrapper, UICommand::MOTION);
  registerCommand("STOP", "Stop movement", handleStopStaticWrapper, UICommand::MOTION);
  registerCommand("ESTOP", "Emergency stop", handleEmergencyStopStaticWrapper, UICommand::EMERGENCY);
  registerCommand("STATUS", "Show robot status", handleStatusStaticWrapper, UICommand::QUERY);
  registerCommand("CONFIG", "Show configuration", handleConfigStaticWrapper, UICommand::CONFIG);
  registerCommand("TEACH", "Teach point X Y Z [ID]", handleTeachStaticWrapper, UICommand::PROGRAM);
  registerCommand("RUN", "Run program [NAME]", handleRunStaticWrapper, UICommand::PROGRAM);
  registerCommand("RESET", "Reset errors", handleResetStaticWrapper, UICommand::SYSTEM);
  registerCommand("LIST", "List all commands", handleListStaticWrapper, UICommand::SYSTEM);
  registerCommand("MODE", "Set UI mode [MODE]", handleModeStaticWrapper, UICommand::SYSTEM);
}

bool UI::executeCommand(const String& command, const String& args) {
  for (int i = 0; i < command_count_; i++) {
    if (command == commands_[i].name) {
      return commands_[i].handler(this, args);
    }
  }
  return false;
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

bool UI::handleHelp(const String& args) {
  printHelp();
  return true;
}

bool UI::handleMove(const String& args) {
  if (!robot_core_->isReady()) {
    sendResponse("MOVE", false, "Robot not ready");
    return false;
  }

  Vector3 point(0, 0, 0);
  float velocity = 0;

  char* token = strtok((char*)args.c_str(), " ");
  int count = 0;

  while (token != nullptr && count < 4) {
    float val = atof(token);
    if (count < 3) {
      (&point.x)[count] = val;
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

  bool success = robot_core_->moveToPoint(point, velocity);
  sendResponse("MOVE", success, success ? "Movement started" : "Failed to start movement");
  return success;
}

bool UI::handleHome(const String& args) {
  bool success = robot_core_->performHoming();
  sendResponse("HOME", success, success ? "Homing started" : "Failed to start homing");
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
  Vector3 point(0, 0, 0);
  uint32_t point_id = 0;

  char* token = strtok((char*)args.c_str(), " ");
  int count = 0;

  while (token != nullptr && count < 4) {
    float val = atof(token);
    if (count < 3) {
      (&point.x)[count] = val;
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

  bool success = robot_core_->teachPoint(point, point_id);
  sendResponse("TEACH", success, success ? "Point taught" : "Failed to teach point");
  return success;
}

bool UI::handleRun(const String& args) {
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
  sendResponse("RESET", success, success ? "Reset successful" : "Reset failed");
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

// Статические обертки для передачи команд в регистрацию команд
bool UI::handleHelpStaticWrapper(UI* ui, const String& args) {return ui->handleHelp(args);}
bool UI::handleMoveStaticWrapper(UI* ui, const String& args) {return ui->handleMove(args);}
bool UI::handleHomeStaticWrapper(UI* ui, const String& args) {return ui->handleHome(args);}
bool UI::handleStopStaticWrapper(UI* ui, const String& args) {return ui->handleStop(args);}
bool UI::handleStatusStaticWrapper(UI* ui, const String& args) {return ui->handleStatus(args);}
bool UI::handleConfigStaticWrapper(UI* ui, const String& args) {return ui->handleConfig(args);}
bool UI::handleTeachStaticWrapper(UI* ui, const String& args) {return ui->handleTeach(args);}
bool UI::handleRunStaticWrapper(UI* ui, const String& args) {return ui->handleRun(args);}
bool UI::handleEmergencyStopStaticWrapper(UI* ui, const String& args) {return ui->handleEmergencyStop(args);}
bool UI::handleResetStaticWrapper(UI* ui, const String& args) {return ui->handleReset(args);}
bool UI::handleListStaticWrapper(UI* ui, const String& args) {return ui->handleList(args);}
bool UI::handleModeStaticWrapper(UI* ui, const String& args) {return ui->handleMode(args);}

Vector3 UI::parsePoint(const String& args) const {
  Vector3 point(0, 0, 0);
  char* token = strtok((char*)args.c_str(), " ");
  int count = 0;

  while (token != nullptr && count < 3) {
    (&point.x)[count] = atof(token);
    token = strtok(nullptr, " ");
    count++;
  }

  return point;
}

Vector3 UI::parseJoints(const String& args) const {
  float angles[3] = {0, 0, 0};
  char* token = strtok((char*)args.c_str(), " ");
  int count = 0;

  while (token != nullptr && count < 3) {
    angles[count] = atof(token) * MathUtils::DEG_TO_RAD;
    token = strtok(nullptr, " ");
    count++;
  }

  return Vector3(angles[0], angles[1], angles[2]);
}

void UI::printWelcomeMessage() {
  Serial.println();
  Serial.println("========================================");
  Serial.println("    DELTA ROBOT CONTROL SYSTEM v1.0");
  Serial.println("========================================");
  Serial.println();
}

void UI::printPrompt() {
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

void UI::onError(uint16_t error_code, const String& error_message) {
  sendMessage(String("Error ") + error_code + ": " + error_message, true);
}
