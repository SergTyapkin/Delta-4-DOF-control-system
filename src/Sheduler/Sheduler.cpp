#include "Sheduler.h"
#include <Arduino.h>
#include "../../src/utils/Logger.h"
#include <algorithm>

// Макрос для измерения времени выполнения (в микросекундах)
#define TIME_TASK_START() uint32_t task_start = micros()
#define TIME_TASK_END()   uint32_t task_end = micros()
#define GET_TASK_TIME()   (task_end - task_start)

Scheduler::Scheduler()
    : task_count_(0)
    , next_task_id_(1)
    , total_run_time_(0)
    , idle_time_(0)
    , last_statistics_time_(0)
    , emergency_stop_(false)
    , emergency_time_(0) {
}

void Scheduler::init() {
  Logger::info("Initializing Task Scheduler...");

  // Инициализация массива задач
  for (auto& task : tasks_) {
    task = Task();
  }

  Logger::info("Scheduler ready. Max tasks: %d", MAX_TASKS);
}

bool Scheduler::addTask(std::function<bool(void*)> func, void* context,
                        uint32_t period_ms, Priority priority,
                        bool enabled, bool one_shot) {

  if (task_count_ >= MAX_TASKS) {
    Logger::error("Cannot add task: maximum tasks reached (%d)", MAX_TASKS);
    return false;
  }

  if (!func) {
    Logger::error("Cannot add task: null function");
    return false;
  }

  // Находим свободный слот
  uint8_t slot = findFreeTaskSlot();
  if (slot == 255) {
    Logger::error("Cannot add task: no free slots");
    return false;
  }

  // Заполняем задачу
  tasks_[slot].id = next_task_id_++;
  tasks_[slot].function = func;
  tasks_[slot].context = context;
  tasks_[slot].period_ms = period_ms;
  tasks_[slot].next_run_time = millis() + (period_ms > 0 ? period_ms : 0);
  tasks_[slot].priority = priority;
  tasks_[slot].status = TaskStatus::READY;
  tasks_[slot].enabled = enabled;
  tasks_[slot].one_shot = one_shot;

  task_count_++;

  // Сортируем по приоритету
  sortTasksByPriority();

  Logger::debug("Task added: ID=%d, period=%dms, priority=%d",
                tasks_[slot].id, period_ms, priority);

  return true;
}

bool Scheduler::removeTask(uint8_t task_id) {
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (tasks_[i].id == task_id) {
      Logger::debug("Removing task ID=%d", task_id);

      // Сбрасываем задачу
      tasks_[i] = Task();
      task_count_--;

      // Пересортировка
      sortTasksByPriority();

      return true;
    }
  }

  Logger::warning("Task ID=%d not found for removal", task_id);
  return false;
}

bool Scheduler::suspendTask(uint8_t task_id) {
  for (auto& task : tasks_) {
    if (task.id == task_id && task.status != TaskStatus::SUSPENDED) {
      task.status = TaskStatus::SUSPENDED;
      Logger::debug("Task ID=%d suspended", task_id);
      return true;
    }
  }
  return false;
}

bool Scheduler::resumeTask(uint8_t task_id) {
  for (auto& task : tasks_) {
    if (task.id == task_id && task.status == TaskStatus::SUSPENDED) {
      task.status = TaskStatus::READY;
      task.next_run_time = millis() + task.period_ms;
      Logger::debug("Task ID=%d resumed", task_id);
      return true;
    }
  }
  return false;
}

void Scheduler::run() {
  if (emergency_stop_) {
    // В режиме экстренной остановки выполняем только критические задачи
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
      if (tasks_[i].id == 0 || !tasks_[i].enabled) continue;
      if (tasks_[i].priority != PRIORITY_CRITICAL) continue;
      if (tasks_[i].status != TaskStatus::READY) continue;

      uint32_t current_time = millis();
      if (current_time >= tasks_[i].next_run_time) {
        TIME_TASK_START();

        tasks_[i].status = TaskStatus::RUNNING;
        bool should_continue = tasks_[i].function(tasks_[i].context);
        tasks_[i].execution_count++;

        TIME_TASK_END();
        updateStatistics(i, GET_TASK_TIME());

        if (should_continue && !tasks_[i].one_shot) {
          tasks_[i].next_run_time = current_time + tasks_[i].period_ms;
          tasks_[i].status = TaskStatus::READY;
        } else {
          tasks_[i].status = TaskStatus::COMPLETED;
          tasks_[i].enabled = false;
        }
      }
    }
    return;
  }

  // Нормальный режим работы
  bool task_executed = false;
  uint32_t loop_start = micros();

  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (tasks_[i].id == 0 || !tasks_[i].enabled) continue;
    if (tasks_[i].status != TaskStatus::READY) continue;

    uint32_t current_time = millis();
    if (current_time >= tasks_[i].next_run_time) {
      TIME_TASK_START();

      task_executed = true;
      tasks_[i].status = TaskStatus::RUNNING;
      bool should_continue = tasks_[i].function(tasks_[i].context);
      tasks_[i].execution_count++;

      TIME_TASK_END();
      uint32_t exec_time = GET_TASK_TIME();
      updateStatistics(i, exec_time);

      // Проверка на превышение максимального времени выполнения
      if (exec_time / 1000 > 10) { // Более 10 мс
        Logger::warning("Task ID=%d took %d ms (max 10ms)",
                        tasks_[i].id, exec_time / 1000);
      }

      if (should_continue && !tasks_[i].one_shot) {
        tasks_[i].next_run_time = current_time + tasks_[i].period_ms;
        tasks_[i].status = TaskStatus::READY;
      } else {
        tasks_[i].status = TaskStatus::COMPLETED;
        tasks_[i].enabled = false;
      }
    }
  }

  uint32_t loop_time = micros() - loop_start;
  total_run_time_ += loop_time;

  // Если не было выполнено ни одной задачи, увеличиваем idle time
  if (!task_executed) {
    idle_time_ += loop_time;
  }

  // Периодический вывод статистики (раз в 10 секунд)
  static uint32_t last_stat_print = 0;
  uint32_t now = millis();
  if (now - last_stat_print > 10000) {
    printStatistics();
    last_stat_print = now;
  }
}

uint8_t Scheduler::findFreeTaskSlot() const {
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (tasks_[i].id == 0) {
      return i;
    }
  }
  return 255; // Нет свободных слотов
}

void Scheduler::sortTasksByPriority() {
  // Сортировка пузырьком по приоритету (высокий приоритет - первые)
  for (uint8_t i = 0; i < MAX_TASKS - 1; i++) {
    for (uint8_t j = 0; j < MAX_TASKS - i - 1; j++) {
      if (tasks_[j].id != 0 && tasks_[j + 1].id != 0) {
        if (tasks_[j].priority < tasks_[j + 1].priority) {
          // Меняем местами
          Task temp = tasks_[j];
          tasks_[j] = tasks_[j + 1];
          tasks_[j + 1] = temp;
        }
      }
    }
  }
}

void Scheduler::updateStatistics(uint8_t task_index, uint32_t execution_time) {
  tasks_[task_index].total_execution_time += execution_time;
  if (execution_time > tasks_[task_index].max_execution_time) {
    tasks_[task_index].max_execution_time = execution_time;
  }
}

float Scheduler::getCpuLoad() const {
  if (total_run_time_ == 0) return 0.0f;

  uint32_t busy_time = total_run_time_ - idle_time_;
  return (busy_time * 100.0f) / total_run_time_;
}

void Scheduler::printStatistics() const {
  Logger::info("=== Scheduler Statistics ===");
  Logger::info("CPU Load: %.1f%%", getCpuLoad());
  Logger::info("Active tasks: %d/%d", task_count_, MAX_TASKS);

  for (const auto& task : tasks_) {
    if (task.id != 0) {
      float avg_time = task.execution_count > 0 ?
                       task.total_execution_time / (1000.0f * task.execution_count) : 0;
      float max_time = task.max_execution_time / 1000.0f;

      Logger::info("  Task ID=%d: runs=%d, avg=%.2fms, max=%.2fms, status=%d",
                   task.id, task.execution_count, avg_time, max_time,
                   static_cast<int>(task.status));
    }
  }
}

void Scheduler::resetStatistics() {
  total_run_time_ = 0;
  idle_time_ = 0;
  last_statistics_time_ = millis();

  for (auto& task : tasks_) {
    if (task.id != 0) {
      task.execution_count = 0;
      task.total_execution_time = 0;
      task.max_execution_time = 0;
    }
  }

  Logger::info("Scheduler statistics reset");
}

const Scheduler::Task* Scheduler::getTaskInfo(uint8_t task_id) const {
  for (const auto& task : tasks_) {
    if (task.id == task_id) {
      return &task;
    }
  }
  return nullptr;
}

void Scheduler::emergencyStop() {
  if (!emergency_stop_) {
    emergency_stop_ = true;
    emergency_time_ = millis();

    // Приостанавливаем все некритические задачи
    for (auto& task : tasks_) {
      if (task.id != 0 && task.priority != PRIORITY_CRITICAL) {
        task.status = TaskStatus::SUSPENDED;
      }
    }

    Logger::critical("Scheduler: EMERGENCY STOP activated");
  }
}

void Scheduler::resumeFromEmergency() {
  if (emergency_stop_) {
    emergency_stop_ = false;

    // Возобновляем все задачи
    for (auto& task : tasks_) {
      if (task.id != 0 && task.status == TaskStatus::SUSPENDED) {
        task.status = TaskStatus::READY;
        task.next_run_time = millis() + task.period_ms;
      }
    }

    uint32_t emergency_duration = millis() - emergency_time_;
    Logger::info("Scheduler: Emergency cleared after %d ms", emergency_duration);
  }
}
