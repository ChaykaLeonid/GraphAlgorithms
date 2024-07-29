#ifndef NAVIGATOR_SRC_INLCUDES_S21_QUEUE_H_
#define NAVIGATOR_SRC_INLCUDES_S21_QUEUE_H_

#include <iostream>

#include "s21_list.h"

namespace s21 {
template <typename T, class container_type = List<T>>
class Queue {
 public:
  using value_type = T;
  using reference = T &;
  using const_reference = const T &;
  using size_type = size_t;

  Queue() : container_(new container_type){};

  Queue(std::initializer_list<value_type> const &items) {
    container_ = new container_type;
    for (auto it = items.begin(); it != items.end(); ++it) {
      this->push(*it);
    }
  };

  Queue(const Queue &q) : container_(new container_type) {
    typename container_type::const_iterator it = q.container_->cbegin();
    while (it != q.container_->cend()) {
      this->push(*it);
      ++it;
    }
  };

  Queue(Queue &&q) : container_(new container_type) {
    this->swap(q);
    q.container_->clear();
  };

  ~Queue() { delete container_; };
  Queue &operator=(const Queue &q) {
    container_->clear();
    typename container_type::const_iterator it = q.container_->cbegin();
    while (it != q.container_->cend()) {
      this->push(*it);
      ++it;
    }
    return *this;
  };

  const_reference front() { return container_->front(); };

  const_reference back() { return container_->back(); };

  bool empty() { return container_->empty(); };

  size_type size() { return container_->size(); };

  void push(const_reference value) { container_->push_back(value); };

  void pop() { container_->pop_front(); };

  void swap(Queue &other) { container_->swap(*(other.container_)); };

 private:
  container_type *container_;
};
}  // namespace s21
#endif  // NAVIGATOR_SRC_INLCUDES_S21_QUEUE_H_
