// src/Scheduler/Scheduler.h
#pragma once

#include <cstdint>
#include <functional>

class Scheduler {
public:
  // Максимальное количество задач
  static constexpr uint8_t MAX_TASKS = 16;

  // Приоритеты задач (чем выше число, тем выше приоритет)
  enum Priority {
    PRIORITY_CRITICAL = 10,   // Аварийные обработчики
    PRIORITY_HIGH     = 8,    // Управление приводами
    PRIORITY_MEDIUM   = 5,    // Кинематика, траектории
    PRIORITY_LOW      = 3,    // Обработка команд
    PRIORITY_IDLE     = 1     // UI, логирование
  };

  // Статус задачи
  enum class TaskStatus {
    READY,
    RUNNING,
    SUSPENDED,
    COMPLETED,
    ERROR
  };

  // Структура задачи
  struct Task {
    // Идентификатор задачи
    uint8_t id;

    // Функция задачи (возвращает true если нужно продолжать выполнение)
    std::function<bool(void*)> function;

    // Контекст для функции
    void* context;

    // Период выполнения (мс), 0 для однократного выполнения
    uint32_t period_ms;

    // Время следующего запуска
    uint32_t next_run_time;

    // Приоритет
    Priority priority;

    // Статус
    TaskStatus status;

    // Статистика
    uint32_t execution_count;
    uint32_t total_execution_time;
    uint32_t max_execution_time;

    // Флаги
    bool enabled;
    bool one_shot;

    Task() : id(0), function(nullptr), context(nullptr),
             period_ms(0), next_run_time(0), priority(PRIORITY_IDLE),
             status(TaskStatus::READY), execution_count(0),
             total_execution_time(0), max_execution_time(0),
             enabled(true), one_shot(false) {}
  };

  // Конструктор
  Scheduler();

  // Инициализация
  void init();

  // Добавление задачи
  bool addTask(std::function<bool(void*)> func, void* context,
               uint32_t period_ms, Priority priority,
               bool enabled = true, bool one_shot = false);

  // Удаление задачи
  bool removeTask(uint8_t task_id);

  // Приостановка задачи
  bool suspendTask(uint8_t task_id);

  // Возобновление задачи
  bool resumeTask(uint8_t task_id);

  // Основной цикл планировщика
  void run();

  // Получение статистики
  uint32_t getTaskCount() const { return task_count_; }
  uint32_t getTotalRunTime() const { return total_run_time_; }
  float getCpuLoad() const; // В процентах

  // Сброс статистики
  void resetStatistics();

  // Получение информации о задаче
  const Task* getTaskInfo(uint8_t task_id) const;

  // Экстренная остановка всех задач (кроме критических)
  void emergencyStop();

  // Возобновление после экстренной остановки
  void resumeFromEmergency();

private:
  // Массив задач
  Task tasks_[MAX_TASKS];

  // Счетчики
  uint8_t task_count_;
  uint8_t next_task_id_;

  // Статистика
  uint32_t total_run_time_;
  uint32_t idle_time_;
  uint32_t last_statistics_time_;

  // Состояние
  bool emergency_stop_;
  uint32_t emergency_time_;

  // Приватные методы
  uint8_t findFreeTaskSlot() const;
  void sortTasksByPriority();
  void updateStatistics(uint32_t task_index, uint32_t execution_time);
  void printStatistics() const;
};
