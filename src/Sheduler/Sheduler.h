#pragma once

#include <Arduino.h>

// Вместо std::function, используем указатели на функции
typedef bool (*TaskFunc)(void* context);


class Sheduler {
public:
  // Максимальное количество задач
  static constexpr uint8_t MAX_TASKS = 8;

  // Приоритеты задач (чем выше число, тем выше приоритет)
  enum Priority {
    PRIORITY_CRITICAL = 4,   // Аварийные обработчики
    PRIORITY_HIGH     = 3,    // Управление приводами
    PRIORITY_MEDIUM   = 2,    // Кинематика, траектории
    PRIORITY_LOW      = 1,    // Обработка команд
    PRIORITY_IDLE     = 0     // UI, Логирование
  };

  // Статус задачи
  enum TaskStatus {
    TASK_READY,
    TASK_RUNNING,
    TASK_SUSPENDED,
    TASK_COMPLETED,
    TASK_ERROR
  };

  // Структура задачи
  struct Task {
    // Идентификатор задачи
    uint8_t id;

    // Функция задачи (возвращает true если нужно продолжать выполнение)
    TaskFunc function;

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
             status(TASK_READY), execution_count(0),
             total_execution_time(0), max_execution_time(0),
             enabled(true), one_shot(false) {}
  };

  // Конструктор
  Sheduler();

  // Инициализация
  void init();

  // Добавление задачи
  bool addTask(TaskFunc func, void* context,
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

  // Распечатать статистику выполнения задач
  void printStatistics() const;

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
  void updateStatistics(uint8_t task_index, uint32_t execution_time);
};
