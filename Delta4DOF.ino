#include "src/Scheduler/Scheduler.h"
#include "src/Safety/EmergencySystem.h"
#include "src/Core/Core.h"
#include "src/UI/UI.h"
#include "config/pins_config.h"
#include "config/robot_params.h"
#include "config/limits.h"
#include "src/utils/Logger.h"

// ============================================================================
// Глобальные экземпляры компонентов системы
// ============================================================================

// 1. Планировщик задач - сердце real-time системы
Scheduler taskScheduler;

// 2. Система безопасности - высший приоритет
EmergencySystem emergencySys;

// 3. Ядро робота - управление логикой и кинематикой
Core robotCore;

// 4. Пользовательский интерфейс - взаимодействие с оператором
UI userInterface;

// ============================================================================
// Объявления функций задач для планировщика
// ============================================================================

// Задача безопасности - самый высокий приоритет (выполняется каждые 1 мс)
void safetyTask(void* context) {
  emergencySys.update();
}

// Задача управления приводами - высокий приоритет (500 Гц)
void drivesControlTask(void* context) {
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
}

// Задача кинематики и планирования - средний приоритет (100 Гц)
void kinematicsTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = millis();

  if (current_time - last_execution >= 10) { // 10 мс = 100 Гц
    if (!emergencySys.isEmergencyActive() && robotCore.isReady()) {
      // Обновление кинематики и планирование движения
      // Вызывается через Core::update()
    }
    last_execution = current_time;
  }
}

// Задача ядра системы - средний приоритет (50 Гц)
void coreTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = millis();

  if (current_time - last_execution >= 20) { // 20 мс = 50 Гц
    if (!emergencySys.isEmergencyActive()) {
      robotCore.update(20); // 20 мс дельта времени
    }
    last_execution = current_time;
  }
}

// Задача пользовательского интерфейса - низкий приоритет (20 Гц)
void uiTask(void* context) {
  static uint32_t last_execution = 0;
  uint32_t current_time = millis();

  if (current_time - last_execution >= 50) { // 50 мс = 20 Гц
    userInterface.update();

    // Периодическая отправка состояния (раз в секунду)
    static uint32_t last_state_send = 0;
    if (current_time - last_state_send > 1000) {
      RobotState state = robotCore.getState();
      userInterface.sendState(state);
      last_state_send = current_time;
    }
    last_execution = current_time;
  }
}

// Задача мониторинга и диагностики - низкий приоритет (5 Гц)
void monitoringTask(void* context) {
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
}

// ============================================================================
// Callback'и для системы безопасности
// ============================================================================

// Обработчик аварийной ситуации
void emergencyCallback(EmergencySystem::ErrorCode error, void* context) {
  Logger::critical("EMERGENCY CALLBACK: Error 0x%04X",
                   static_cast<uint16_t>(error));

  // Останавливаем все задачи кроме критических
  taskScheduler.emergencyStop();

  // Отправляем сообщение в UI
  userInterface.sendMessage("EMERGENCY STOP ACTIVATED", true);

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
config.enable_pin = Pins::DRIVE_1.enable_pin;
config.limit_switch_pin = Pins::DRIVE_1.limit_switch_pin;
config.fault_pin = Pins::DRIVE_1.fault_pin;
break;

case 1:
config.step_pin = Pins::DRIVE_2.step_pin;
config.dir_pin = Pins::DRIVE_2.dir_pin;
config.enable_pin = Pins::DRIVE_2.enable_pin;
config.limit_switch_pin = Pins::DRIVE_2.limit_switch_pin;
config.fault_pin = Pins::DRIVE_2.fault_pin;
break;

case 2:
config.step_pin = Pins::DRIVE_3.step_pin;
config.dir_pin = Pins::DRIVE_3.dir_pin;
config.enable_pin = Pins::DRIVE_3.enable_pin;
config.limit_switch_pin = Pins::DRIVE_3.limit_switch_pin;
config.fault_pin = Pins::DRIVE_3.fault_pin;
break;
}

// Общие параметры приводов
config.steps_per_revolution = RobotParams::STEPS_PER_REVOLUTION;
config.microsteps = RobotParams::MICROSTEPS;
config.gear_ratio = RobotParams::GEAR_RATIO;

config.max_velocity = 5.0f; // рад/с
config.max_acceleration = 20.0f; // рад/с²
config.max_jerk = 100.0f; // рад/с³

config.homing_velocity = 1.0f; // рад/с
config.homing_acceleration = 5.0f; // рад/с²
config.homing_direction = Drive::HomingDirection::NEGATIVE;

config.run_current = 1.5f; // А
config.hold_current = 0.8f; // А

config.backlash_compensation = 0.001f; // рад
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

  config.sync_tolerance = 0.01f; // рад
  config.sync_timeout = 5000; // мс
  config.enable_sync_move = true;
  config.stop_on_single_error = true;

  return config;
}

// Конфигурация кинематики
DeltaSolver::DeltaConfig createKinematicsConfig() {
  DeltaSolver::DeltaConfig config;

  config.base_radius = RobotParams::BASE_RADIUS;
  config.effector_radius = RobotParams::EFFECTOR_RADIUS;
  config.arm_length = RobotParams::ARM_LENGTH;
  config.forearm_length = RobotParams::FOREARM_LENGTH;

  for (int i = 0; i < 3; i++) {
    config.base_angles[i] = RobotParams::BASE_ANGLES[i] * (M_PI / 180.0f);
  }

  return config;
}

// Конфигурация ядра системы
Core::Config createCoreConfig() {
  Core::Config config;

  config.kinematics_config = createKinematicsConfig();
  config.drives_config = createDrivesControllerConfig();

  config.default_velocity = Limits::VELOCITY.max_linear_velocity * 0.3f;
  config.default_acceleration = Limits::VELOCITY.max_linear_acceleration * 0.3f;
  config.max_jerk = Limits::VELOCITY.max_jerk;
  config.trajectory_update_rate = 100.0f; // Гц

  return config;
}

// Конфигурация пользовательского интерфейса
UI::Config createUIConfig() {
  UI::Config config;

  config.update_interval = 50; // мс
  config.command_timeout = 10000; // мс
  config.echo_commands = true;
  config.debug_mode = false;

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
  UI::Config ui_config = createUIConfig();
  userInterface.init(&robotCore, ui_config);
  Logger::info("User Interface: READY");

  // Шаг 4: Настройка планировщика задач
  Logger::info("Step 4: Setting up Task Scheduler...");
  taskScheduler.init();

  // Добавление задач с приоритетами:
  // 10 (высший) - 1 (низший)

  // Критические задачи
  taskScheduler.addTask(safetyTask, nullptr, 1, Scheduler::PRIORITY_CRITICAL);

  // Высокоприоритетные задачи (управление)
  taskScheduler.addTask(drivesControlTask, nullptr, 2, Scheduler::PRIORITY_HIGH);

  // Среднеприоритетные задачи (планирование)
  taskScheduler.addTask(kinematicsTask, nullptr, 10, Scheduler::PRIORITY_MEDIUM);
  taskScheduler.addTask(coreTask, nullptr, 20, Scheduler::PRIORITY_MEDIUM);

  // Низкоприоритетные задачи (UI, мониторинг)
  taskScheduler.addTask(uiTask, nullptr, 50, Scheduler::PRIORITY_LOW);
  taskScheduler.addTask(monitoringTask, nullptr, 200, Scheduler::PRIORITY_IDLE);

  Logger::info("Task Scheduler: READY (%d tasks configured)", taskScheduler.getTaskCount());

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

  userInterface.sendMessage("Starting homing sequence...", false);

  bool homing_success = robotCore.performHoming();
  if (!homing_success) {
    Logger::error("Homing failed!");
    userInterface.sendMessage("HOMING FAILED!", true);
    return false;
  }

  // Ожидание завершения homing
  uint32_t homing_start = millis();
  const uint32_t HOMING_TIMEOUT = 30000; // 30 секунд

  while (!robotCore.isHomed() && !emergencySys.isEmergencyActive()) {
    if (millis() - homing_start > HOMING_TIMEOUT) {
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
  userInterface.sendMessage("Homing completed successfully", false);

  // Шаг 4: Переход в рабочее положение
  Logger::info("Step 4: Moving to ready position...");

  // Двигаемся в безопасную стартовую позицию
  Vector3 ready_position(0, 0, -500); // Посередине по Z
  if (!robotCore.moveToPoint(ready_position, 30.0f)) {
    Logger::warning("Failed to move to ready position");
  } else {
    // Ждем завершения движения (упрощенно)
    delay(2000);
  }

  Logger::info("Startup sequence: COMPLETE");
  return true;
}

// ============================================================================
// Arduino стандартные функции
// ============================================================================

void setup() {
  // Задержка для стабилизации питания
  delay(100);

  // Инициализация последовательного порта
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Logger::init(Logger::Level::INFO);

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
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
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

    userInterface.sendMessage("SYSTEM READY - Type 'help' for commands", false);

    // Включение светодиода готовности
    digitalWrite(Pins::Status::LED_READY, HIGH);
  }
}

void loop() {
  // Главный цикл - только запуск планировщика задач
  // Вся работа выполняется в задачах планировщика

  taskScheduler.run();

  // Небольшая задержка для снижения нагрузки на CPU
  // В реальной системе лучше использовать прерывания по таймеру
  delayMicroseconds(100);
}

// ============================================================================
// Обработчики прерываний (если нужны)
// ============================================================================

// Пример обработчика прерывания от энкодера
// void encoderISR() {
//     // Обработка в соответствующем модуле
// }

// Пример обработчика прерывания от концевика
// void limitSwitchISR() {
//     emergencySys.handleLimitSwitch();
// }

// ============================================================================
// Вспомогательные функции
// ============================================================================

// Функция для аварийного восстановления
void emergencyRecovery() {
  Logger::critical("ATTEMPTING EMERGENCY RECOVERY...");

  // 1. Сброс аварийного состояния безопасности
  if (!emergencySys.reset()) {
    Logger::error("Cannot reset emergency state - check hardware");
    return;
  }

  // 2. Сброс планировщика
  taskScheduler.resumeFromEmergency();

  // 3. Сброс ядра робота
  robotCore.reset();

  // 4. Повторный homing
  if (performStartupSequence()) {
    Logger::info("EMERGENCY RECOVERY SUCCESSFUL");
    userInterface.sendMessage("System recovered from emergency", false);
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
  for (int i = 0; i < 3; i++) {
    // Здесь нужно получить состояние каждого привода через Core
    Logger::info("  Drive %d: [status]", i);
  }

  // 3. Проверка кинематики
  Logger::info("Kinematics test:");
  Vector3 test_point(0, 0, -500);
  if (robotCore.moveToPoint(test_point, 10.0f)) {
    Logger::info("  Test movement: OK");
  } else {
    Logger::error("  Test movement: FAILED");
  }

  // 4. Проверка планировщика
  float cpu_load = taskScheduler.getCpuLoad();
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
    taskScheduler.printStatistics();
    robotCore.printStatus();
  } else if (command == "reset") {
    Logger::info("Soft reset requested...");
    ESP.restart(); // Для ESP платформ
    // Для STM32: NVIC_SystemReset();
  } else {
    Logger::info("Unknown debug command: %s", command.c_str());
    Logger::info("Available debug commands:");
    Logger::info("  diag    - Run diagnostics");
    Logger::info("  recover - Emergency recovery");
    Logger::info("  status  - System status");
    Logger::info("  reset   - Soft reset");
  }
}
