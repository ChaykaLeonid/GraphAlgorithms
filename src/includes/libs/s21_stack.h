#ifndef NAVIGATOR_SRC_INCLUDES_S21_STACK_H_
#define NAVIGATOR_SRC_INCLUDES_S21_STACK_H_

#include <iostream>

#include "s21_list.h"

namespace s21 {
template <typename T, class container_type = List<T>>
class Stack {
 public:
  using value_type = T;
  using reference = T &;
  using const_reference = const T &;
  using size_type = size_t;

  Stack() : container_(new container_type){};

  Stack(std::initializer_list<value_type> const &items) {
    container_ = new container_type;
    for (auto it = items.begin(); it != items.end(); it++) {
      this->push(*it);
    }
  };

  Stack(const Stack &s) : container_(new container_type) {
    typename container_type::const_iterator it = s.container_->cbegin();
    while (it != s.container_->cend()) {
      this->push(*it);
      ++it;
    }
  };

  Stack(Stack &&s) : container_(new container_type) {
    this->swap(s);
    s.container_->clear();
  };

  ~Stack() { delete container_; };
  Stack &operator=(const Stack &s) {
    container_->clear();
    typename container_type::const_iterator it = s.container_->cbegin();
    while (it != s.container_->cend()) {
      this->push(*it);
      ++it;
    }
    return *this;
  };

  const_reference top() { return container_->back(); };

  bool empty() { return container_->empty(); };

  size_type size() { return container_->size(); };

  void push(const_reference value) { container_->push_back(value); };

  void pop() { container_->pop_back(); };

  void swap(Stack &other) { container_->swap(other); };

 private:
  container_type *container_;
};
}  // namespace s21

#endif  //  NAVIGATOR_SRC_INCLUDES_S21_STACK_H_
