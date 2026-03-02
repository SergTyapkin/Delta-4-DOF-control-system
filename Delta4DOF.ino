#include "src/Sheduler/Sheduler.h"
#include "src/Safety/EmergencySystem.h"
#include "src/Core/Core.h"
#include "src/UI/UI.h"
#include "src/utils/Logger.h"

#include "config/pins_config.h"
#include "config/robot_params.h"
#include "config/limits.h"
#include "config/system.h"

// ============================================================================
// Глобальные экземпляры компонентов системы
// ============================================================================

// 1. Планировщик задач - сердце real-time системы
Sheduler taskSheduler;

// 2. Система безопасности - высший приоритет
EmergencySystem emergencySys;

// 3. Ядро робота - управление логикой и кинематикой
Core robotCore;

// 4. Пользовательский интерфейс - взаимодействие с оператором
UserInterface UI;

// ============================================================================
// Объявления функций задач для планировщика
// ============================================================================

// Задача безопасности - самый высокий приоритет (выполняется каждые 1 мс)
bool safetyTask(void* context) {
  emergencySys.update();
  return true;
}

// Задача управления приводами - высокий приоритет (500 Гц)
bool drivesControlTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = micros();
  uint32_t delta_time = current_time - last_execution;

  if (delta_time >= 2000) { // 2 мс = 500 Гц
    if (!emergencySys.isEmergencyActive() && robotCore.isReady()) {
      // Здесь будет прямое управление приводами
      // В реальности это делается через Core
    }
    last_execution = current_time;
  }
  return true;
}

// Задача кинематики и планирования - средний приоритет (100 Гц)
bool kinematicsTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = millis();

  if (current_time - last_execution >= 10) { // 10 мс = 100 Гц
    if (!emergencySys.isEmergencyActive() && robotCore.isReady()) {
      // Обновление кинематики и планирование движения
      // Вызывается через Core::update()
    }
    last_execution = current_time;
  }
  return true;
}

// Задача ядра системы - средний приоритет (50 Гц)
bool coreTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = millis();

  if (current_time - last_execution >= 20) { // 20 мс = 50 Гц
    if (!emergencySys.isEmergencyActive()) {
      robotCore.update(20); // 20 мс дельта времени
    }
    last_execution = current_time;
  }
  return true;
}

// Задача пользовательского интерфейса - низкий приоритет (20 Гц)
bool uiTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = millis();

  if (current_time - last_execution >= 50) { // 50 мс = 20 Гц
    UI.update();

    // Периодическая отправка состояния (раз в секунду)
    static uint32_t last_state_send = 0;
    if (current_time - last_state_send > 1000) {
      RobotState state = robotCore.getState();
      UI.sendState(state);
      last_state_send = current_time;
    }
    last_execution = current_time;
  }
  return true;
}

// Задача мониторинга и диагностики - низкий приоритет (5 Гц)
bool monitoringTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = millis();

  if (current_time - last_execution >= 200) { // 200 мс = 5 Гц
    // Мониторинг напряжения
    float voltage = emergencySys.getSupplyVoltage();
    if (voltage < 11.0f) {
      Logger::warning("Low voltage: %.1f V", voltage);
    }

    // Мониторинг температуры (если есть датчик)
    // float temp = emergencySys.readTemperature();

    last_execution = current_time;
  }
  return true;
}

// ============================================================================
// Callback'и для системы безопасности
// ============================================================================

// Обработчик аварийной ситуации
void emergencyCallback(EmergencySystem::ErrorCode error, void* context) {
  Logger::critical("EMERGENCY CALLBACK: Error 0x%04X",
                   static_cast<uint16_t>(error));

  // Останавливаем все задачи кроме критических
  taskSheduler.emergencyStop();

  // Отправляем сообщение в UI
  UI.sendMessage("EMERGENCY STOP ACTIVATED", true);

  // Здесь можно добавить дополнительные действия:
  // - Запись лога
  // - Активация сигнализации
  // - Отправка уведомлений
}

// ============================================================================
// Настройка конфигураций компонентов
// ============================================================================

// Конфигурация системы безопасности
EmergencySystem::LimitSwitchConfig createLimitSwitchConfig() {
  EmergencySystem::LimitSwitchConfig config;
  // Конфигурация будет установлена в init()
  return config;
}

// Конфигурация приводов
Drive::Config createDriveConfig(uint8_t drive_index) {
  Drive::Config config;

  switch (drive_index) {
    case 0:
      config.step_pin = Pins::DRIVE_1.step_pin;
      config.dir_pin = Pins::DRIVE_1.dir_pin;
      //config.enable_pin = Pins::DRIVE_1.enable_pin;
      config.limit_switch_pin = Pins::DRIVE_1.limit_switch_pin;
      config.fault_pin = Pins::DRIVE_1.fault_pin;
      break;

    case 1:
      config.step_pin = Pins::DRIVE_2.step_pin;
      config.dir_pin = Pins::DRIVE_2.dir_pin;
      //config.enable_pin = Pins::DRIVE_2.enable_pin;
      config.limit_switch_pin = Pins::DRIVE_2.limit_switch_pin;
      config.fault_pin = Pins::DRIVE_2.fault_pin;
      break;

    case 2:
      config.step_pin = Pins::DRIVE_3.step_pin;
      config.dir_pin = Pins::DRIVE_3.dir_pin;
      //config.enable_pin = Pins::DRIVE_3.enable_pin;
      config.limit_switch_pin = Pins::DRIVE_3.limit_switch_pin;
      config.fault_pin = Pins::DRIVE_3.fault_pin;
      break;

    case 3:
      config.step_pin = Pins::DRIVE_4.step_pin;
      config.dir_pin = Pins::DRIVE_4.dir_pin;
      //config.enable_pin = Pins::DRIVE_4.enable_pin;
      config.limit_switch_pin = Pins::DRIVE_4.limit_switch_pin;
      config.fault_pin = Pins::DRIVE_4.fault_pin;
      break;
  }

  // Общие параметры приводов
  config.steps_per_revolution = RobotParams::STEPS_PER_REVOLUTION;
  config.microsteps_per_revolution = RobotParams::MICROSTEPS_PER_REVOLUTION;
  config.gear_ratio = RobotParams::GEAR_RATIO;

  config.max_velocity = Limits::DRIVERS.max_joint_velocity; // рад/с
  config.max_acceleration = Limits::DRIVERS.max_joint_acceleration; // рад/с²

  config.homing_velocity = Limits::DRIVERS.homing_velocity; // рад/с
  config.homing_acceleration = Limits::DRIVERS.homing_acceleration; // рад/с²
  config.homing_direction = Limits::DRIVERS.homing_direction;

  //config.run_current = 1.5f; // А
  //config.hold_current = 0.8f; // А

  config.backlash_compensation = Limits::DRIVERS.backlash_compensation; // рад
  config.invert_direction = false;

  return config;
}

// Конфигурация контроллера приводов
DrivesController::Config createDrivesControllerConfig() {
  DrivesController::Config config;

  // Конфигурация трех приводов
  config.drive_configs[0] = createDriveConfig(0);
  config.drive_configs[1] = createDriveConfig(1);
  config.drive_configs[2] = createDriveConfig(2);
  config.drive_configs[3] = createDriveConfig(3);

  return config;
}

// Конфигурация кинематики
Kinematics::DeltaConfig createKinematicsConfig() {
  Kinematics::DeltaConfig config;
  return config;
}

// Конфигурация ядра системы
Core::Config createCoreConfig() {
  Core::Config config;

  config.kinematics_config = createKinematicsConfig();
  config.drives_config = createDrivesControllerConfig();

  return config;
}

// Конфигурация пользовательского интерфейса
UserInterface::Config createUIConfig() {
  UserInterface::Config config;

  config.update_interval = System::INTERVAL_UPDATE_UI_MCS;
  config.command_timeout = Limits::TIME.max_task_time;
  config.echo_commands = System::UI_ECHO_COMMANDS_TO_SERIAL;
  config.debug_mode = System::UI_DEBUG_TO_SERIAL;

  return config;
}

// ============================================================================
// Функции инициализации системы
// ============================================================================

// Инициализация всех компонентов системы
bool initializeSystem() {
  Logger::info("========================================");
  Logger::info("   DELTA ROBOT SYSTEM INITIALIZATION");
  Logger::info("========================================");

  // Шаг 1: Инициализация системы безопасности (самое важное!)
  Logger::info("Step 1: Initializing Safety System...");
  emergencySys.init();
  emergencySys.setEmergencyCallback(emergencyCallback, nullptr);
  Logger::info("Safety System: READY");

  // Шаг 2: Инициализация ядра робота
  Logger::info("Step 2: Initializing Robot Core...");
  Core::Config core_config = createCoreConfig();
  if (!robotCore.init(core_config)) {
    Logger::critical("Failed to initialize Robot Core!");
    emergencySys.triggerEmergency(EmergencySystem::ErrorCode::CORE_INIT_FAILED);
    return false;
  }
  Logger::info("Robot Core: READY");

  // Шаг 3: Инициализация пользовательского интерфейса
  Logger::info("Step 3: Initializing User Interface...");
  UserInterface::Config ui_config = createUIConfig();
  UI.init(&robotCore, ui_config);
  Logger::info("User Interface: READY");

  // Шаг 4: Настройка планировщика задач
  Logger::info("Step 4: Setting up Task Sheduler...");
  taskSheduler.init();

  // Добавление задач с приоритетами:
  // Критические задачи
  taskSheduler.addTask(safetyTask, nullptr, System::INTERVAL_UPDATE_SHEDULER_MCS, Sheduler::PRIORITY_CRITICAL);

  // Высокоприоритетные задачи (управление)
  taskSheduler.addTask(drivesControlTask, nullptr, System::INTERVAL_UPDATE_DRIVES_MCS, Sheduler::PRIORITY_HIGH);

  // Среднеприоритетные задачи (планирование)
  taskSheduler.addTask(kinematicsTask, nullptr, System::INTERVAL_UPDATE_KINEMATICS_MCS, Sheduler::PRIORITY_MEDIUM);
  taskSheduler.addTask(coreTask, nullptr, System::INTERVAL_UPDATE_CORE_MCS, Sheduler::PRIORITY_MEDIUM);

  // Низкоприоритетные задачи (UI, мониторинг)
  taskSheduler.addTask(uiTask, nullptr, System::INTERVAL_UPDATE_UI_MCS, Sheduler::PRIORITY_LOW);
  taskSheduler.addTask(monitoringTask, nullptr, System::INTERVAL_UPDATE_MONITORING_MCS, Sheduler::PRIORITY_IDLE);

  Logger::info("Task Sheduler: READY (%d tasks configured)", taskSheduler.getTaskCount());

  return true;
}

// Выполнение процедуры запуска системы
bool performStartupSequence() {
  Logger::info("========================================");
  Logger::info("       SYSTEM STARTUP SEQUENCE");
  Logger::info("========================================");

  // Шаг 1: Проверка безопасности
  Logger::info("Step 1: Safety checks...");
  if (emergencySys.isEmergencyActive()) {
    Logger::error("System is in emergency state! Cannot proceed.");
    return false;
  }
  Logger::info("Safety checks: PASSED");

  // Шаг 2: Включение приводов
  Logger::info("Step 2: Enabling drives...");
  // Приводы включаются через Core

  // Шаг 3: Выполнение homing
  Logger::info("Step 3: Performing homing sequence...");
  UI.sendMessage("Starting homing sequence...", false);

  bool homing_success = robotCore.performHoming();
  if (!homing_success) {
    Logger::error("Homing failed!");
    UI.sendMessage("HOMING FAILED!", true);
    return false;
  }

  // Ожидание завершения homing
  uint32_t homing_start = millis();

  while (!robotCore.isHomed() && !emergencySys.isEmergencyActive()) {
    if (millis() - homing_start > Limits::TIME.homing_timeout) {
      Logger::error("Homing timeout!");
      emergencySys.triggerEmergency(EmergencySystem::ErrorCode::HOMING_FAILED);
      return false;
    }
    delay(100);
  }

  if (emergencySys.isEmergencyActive()) {
    Logger::error("Emergency during homing!");
    return false;
  }

  Logger::info("Homing: COMPLETE");
  UI.sendMessage("Homing completed successfully", false);

  // Шаг 4: Переход в рабочее положение
  Logger::info("Step 4: Moving to ready position...");

  // Двигаемся в безопасную стартовую позицию
  if (!robotCore.moveToPosition(RobotParams::SAFE_START_POSITION)) {
    Logger::warning("Failed to move to default ready position");
  } else {
    // Ждем завершения движения (упрощенно)
    delay(2000);
  }

  Logger::info("Startup sequence: COMPLETE");
  UI.sendMessage("Startup complete", false);
  return true;
}


// Функция аварийного восстановления
void emergencyRecovery() {
  Logger::critical("ATTEMPTING EMERGENCY RECOVERY...");

  // 1. Сброс аварийного состояния безопасности
  if (!emergencySys.reset()) {
    Logger::error("Cannot reset emergency state - check hardware");
    return;
  }

  // 2. Сброс планировщика
  taskSheduler.resumeFromEmergency();

  // 3. Сброс ядра робота
  robotCore.reset();

  // 4. Повторный homing
  if (performStartupSequence()) {
    Logger::info("EMERGENCY RECOVERY SUCCESSFUL");
    UI.sendMessage("System recovered from emergency", false);
  } else {
    Logger::error("EMERGENCY RECOVERY FAILED");
  }
}

// Функция для диагностики системы
void runDiagnostics() {
  Logger::info("=== SYSTEM DIAGNOSTICS ===");

  // 1. Проверка напряжения
  float voltage = emergencySys.getSupplyVoltage();
  Logger::info("Supply voltage: %.1f V", voltage);

  // 2. Проверка состояния приводов
  Logger::info("Drive states:");
  for (int i = 0; i < RobotParams::MOTORS_COUNT; i++) {
    // Здесь нужно получить состояние каждого привода через Core
    Logger::info("  Drive %d: [status]", i);
  }

  // 3. Проверка кинематики
  Logger::info("Kinematics test:");
  if (robotCore.moveToPosition(RobotParams::SAFE_START_POSITION)) {
    Logger::info("  Test movement: OK");
  } else {
    Logger::error("  Test movement: FAILED");
  }

  // 4. Проверка планировщика
  float cpu_load = taskSheduler.getCpuLoad();
  Logger::info("CPU load: %.1f%%", cpu_load);

  Logger::info("Diagnostics complete");
}

// Обработка команд из Serial для отладки
void handleDebugCommands() {
  static String debug_buffer = "";

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (debug_buffer.length() > 0) {
        processDebugCommand(debug_buffer);
        debug_buffer = "";
      }
    } else {
      debug_buffer += c;
    }
  }
}

// Обработчик отладочных команд
void processDebugCommand(const String& command) {
  if (command == "diag") {
    runDiagnostics();
  } else if (command == "recover") {
    emergencyRecovery();
  } else if (command == "status") {
    taskSheduler.printStatistics();
    robotCore.printStatus();
  } else if (command == "reset") {
    Logger::info("Soft reset requested...");
#ifdef __AVR__
    asm("jmp 0");
#elif defined(ESP8266)
    ESP.restart();
#elif defined(ESP32)
    esp_restart();
#elif defined(STM32)
    NVIC_SystemReset();
#else
    // Для других платформ
    __asm__ volatile ("dsb 0xF":::"memory");
    __asm__ volatile ("wfi");
#endif
  } else {
    Logger::info("Unknown debug command: %s", command.c_str());
    Logger::info("Available debug commands:");
    Logger::info("  diag    - Run diagnostics");
    Logger::info("  recover - Emergency recovery");
    Logger::info("  status  - System status");
    Logger::info("  reset   - Soft reset");
  }
}

// ============================================================================
// Arduino стандартные функции
// ============================================================================

void setup() {
  // Задержка для стабилизации питания
  delay(100);

  // Показываем, что программа начала выполняться
  digitalWrite(Pins::Status::LED_READY, HIGH);
  delay(100);
  digitalWrite(Pins::Status::LED_MOVING, HIGH);
  delay(100);
  digitalWrite(Pins::Status::LED_ERROR, HIGH);
  delay(100);
  digitalWrite(Pins::Status::LED_READY, LOW);
  delay(100);
  digitalWrite(Pins::Status::LED_MOVING, LOW);
  delay(100);
  digitalWrite(Pins::Status::LED_ERROR, LOW);

  // Инициализация последовательного порта
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Logger::init(Logger::LEVEL_DEBUG);

  Logger::info("");
  Logger::info("========================================");
  Logger::info("   DELTA ROBOT CONTROL SYSTEM v1.0");
  Logger::info("   Board: STM32 Nucleo");
  Logger::info("   Build Date: " __DATE__ " " __TIME__);
  Logger::info("========================================");
  Logger::info("");

  // Инициализация системы
  if (!initializeSystem()) {
    Logger::critical("SYSTEM INITIALIZATION FAILED!");
    while (1) {
      // Мигание светодиодом при ошибке
      digitalWrite(Pins::Status::LED_ERROR, HIGH);
      delay(100);
      digitalWrite(Pins::Status::LED_ERROR, LOW);
      delay(100);
      digitalWrite(Pins::Status::LED_ERROR, HIGH);
      delay(100);
      digitalWrite(Pins::Status::LED_ERROR, LOW);
      delay(1000);
    }
  }

  // Выполнение процедуры запуска
  if (!performStartupSequence()) {
    Logger::critical("STARTUP SEQUENCE FAILED!");
    // Система остается в безопасном состоянии
  } else {
    Logger::info("");
    Logger::info("========================================");
    Logger::info("       SYSTEM READY FOR OPERATION");
    Logger::info("========================================");
    Logger::info("");

    UI.sendMessage("SYSTEM READY - Type 'help' for commands", false);

    // Включение светодиода готовности
    digitalWrite(Pins::Status::LED_READY, HIGH);
  }
}

void loop() {
  // Главный цикл - только запуск планировщика задач
  // Вся работа выполняется в задачах планировщика

  taskSheduler.run();

  // Небольшая задержка для снижения нагрузки на CPU
  delayMicroseconds(100);
}
