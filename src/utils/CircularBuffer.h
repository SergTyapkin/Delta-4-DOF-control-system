#pragma once

#include <cstdint>
#include <cstring>

template<typename T, size_t N>
class CircularBuffer {
private:
  T buffer_[N];
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t count_ = 0;
  bool full_ = false;

public:
  CircularBuffer() = default;

  // Добавление элемента
  bool push(const T& item) {
    if (full_) {
      return false; // Буфер полон
    }

    buffer_[head_] = item;
    head_ = (head_ + 1) % N;
    count_++;

    if (head_ == tail_) {
      full_ = true;
    }

    return true;
  }

  // Извлечение элемента
  bool pop(T& item) {
    if (isEmpty()) {
      return false;
    }

    item = buffer_[tail_];
    tail_ = (tail_ + 1) % N;
    count_--;
    full_ = false;

    return true;
  }

  // Просмотр следующего элемента без извлечения
  bool peek(T& item) const {
    if (isEmpty()) {
      return false;
    }

    item = buffer_[tail_];
    return true;
  }

  // Очистка буфера
  void clear() {
    head_ = 0;
    tail_ = 0;
    count_ = 0;
    full_ = false;
  }

  // Проверка на пустоту
  bool isEmpty() const {
    return (!full_ && (head_ == tail_));
  }

  // Проверка на полноту
  bool isFull() const {
    return full_;
  }

  // Количество элементов
  size_t size() const {
    return count_;
  }

  // Ёмкость буфера
  size_t capacity() const {
    return N;
  }

  // Доступ по индексу (только для чтения)
  const T& operator[](size_t index) const {
    size_t actual_index = (tail_ + index) % N;
    return buffer_[actual_index];
  }
};
