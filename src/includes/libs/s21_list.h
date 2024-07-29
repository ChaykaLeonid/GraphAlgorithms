#ifndef NAVIGATOR_SRC_INCLUDES_S21_LIST_H_
#define NAVIGATOR_SRC_INCLUDES_S21_LIST_H_
#include <limits>

namespace s21 {
template <typename T>
class List {
 public:
  using value_type = T;
  using reference = T&;
  using const_reference = const T&;

  class Node {
   public:
    Node() : content_(0), next_(this), prev_(this){};
    Node(value_type value, Node* next, Node* prev)
        : content_(value), next_(next), prev_(prev){};
    Node(const Node& other)
        : content_(other.get_content()),
          next_(other.get_next()),
          prev_(other.get_prev()){};
    Node(Node&& node)
        : content_(node.get_content()),
          next_(node.get_next()),
          prev_(node.get_prev()) {
      node.set_content(0);
      node.set_next(nullptr);
      node.set_prev(nullptr);
    };
    ~Node(){};
    Node& operator=(const Node& other) {
      next_ = other.get_next();
      prev_ = other.get_prev();
      content_ = other.get_content();
    };
    void set_content(const value_type& value) { content_ = value; };
    reference get_content_ref() { return content_; };
    const_reference get_content() const { return content_; };
    void set_next(Node* next) { next_ = next; };
    void set_next(const Node& next) { next_ = &next; };
    Node* get_next() const { return next_; };
    void set_prev(Node* prev) { prev_ = prev; };
    void set_prev(const Node& prev) { prev_ = &prev; };
    Node* get_prev() const { return prev_; };

   private:
    value_type content_;
    Node* next_;
    Node* prev_;
  };

  class ListIterator {
   public:
    ListIterator(Node& node) : address_(&node){};
    ListIterator(Node* node_ptr) : address_(node_ptr){};
    ListIterator(const ListIterator& other) : address_(other.address_){};
    ListIterator(ListIterator&& other) : address_(other.address_) {
      other.address_ = nullptr;
    };
    ~ListIterator(){};
    ListIterator& operator=(const ListIterator& other) {
      address_ = other.address_;
      return *this;
    };

    Node* node() const { return address_; };
    reference operator*() { return address_->get_content_ref(); };
    const_reference operator*() const { return address_->get_content(); };

    ListIterator& operator++() {
      address_ = address_->get_next();
      return *this;
    };

    // TODO узнать, нужно ли делать лвалью
    ListIterator operator++(int) {
      ListIterator tmp = *this;
      address_ = address_->get_next();
      return tmp;
    }

    ListIterator& operator--() {
      address_ = address_->get_prev();
      return *this;
    };

    ListIterator operator--(int) {
      ListIterator tmp = *this;
      address_ = address_->get_prev();
      return tmp;
    };

    ListIterator operator+(int n) {
      ListIterator tmp = *this;
      for (int i = 0; i < n; i++) {
        ++tmp;
      }
      return tmp;
    };

    ListIterator operator-(int n) {
      ListIterator tmp = *this;
      for (int i = 0; i < n; i++) {
        --tmp;
      }
      return tmp;
    };

    ListIterator operator+=(int n) {
      for (int i = 0; i < n; i++) {
        ++*this;
      }
      return *this;
    }

    ListIterator operator-=(int n) {
      for (int i = 0; i < n; i++) {
        --*this;
      }
      return *this;
    }

    bool operator==(const ListIterator& other) {
      return address_ == other.address_;
    }

    bool operator!=(const ListIterator& other) {
      return address_ != other.address_;
    }

   private:
    Node* address_;
  };

  class ListConstIterator {
   public:
    ListConstIterator(Node& node) : address_(&node){};
    ListConstIterator(Node* node_ptr) : address_(node_ptr){};
    ListConstIterator(ListConstIterator& other) : address_(other.address_){};
    ListConstIterator(ListConstIterator&& other) : address_(other.address_) {
      other.address_ = nullptr;
    };

    ~ListConstIterator(){};
    ListConstIterator& operator=(const ListConstIterator& other) {
      address_ = other.address_;
      return *this;
    };

    Node* node() const { return address_; };
    const_reference operator*() const { return address_->get_content(); };
    ListConstIterator operator++() {
      address_ = address_->get_next();
      return *this;
    };

    ListConstIterator operator++(int) {
      ListConstIterator tmp = *this;
      address_ = address_->get_next();
      return tmp;
    };

    ListConstIterator operator--() {
      address_ = address_->get_prev();
      return *this;
    };

    ListConstIterator operator--(int) {
      ListConstIterator tmp = *this;
      address_ = address_->get_prev();
      return tmp;
    };

    ListConstIterator operator+(int n) {
      ListConstIterator tmp = *this;
      for (int i = 0; i < n; i++) ++tmp;
      return tmp;
    };

    ListConstIterator operator-(int n) {
      ListConstIterator tmp = *this;
      for (int i = 0; i < n; i++) --tmp;
      return tmp;
    };

    ListConstIterator operator+=(int n) {
      for (int i = 0; i < n; i++) ++this;
      return *this;
    };

    ListConstIterator operator-=(int n) {
      for (int i = 0; i < n; i++) --this;
      return *this;
    };

    bool operator==(const ListConstIterator other) {
      return address_ == other.address_;
    };

    bool operator!=(const ListConstIterator other) {
      return address_ != other.address_;
    };

   private:
    Node* address_;
  };

  using iterator = ListIterator;
  using const_iterator = ListConstIterator;
  using size_type = size_t;

  List(){};

  List(size_type n) {
    for (int i = 0; i < n; ++i) push_back(0);
  };

  List(std::initializer_list<value_type> const& items) {
    for (auto it = items.begin(); it != items.end(); ++it) push_back(*it);
  };

  List(const List& l) {
    for (auto it = l.cbegin(); it != l.cend(); ++it) push_back(*it);
  };

  List(List&& l) { this->swap(l); };

  ~List() {
    this->clear();
    delete null_node_;
  };

  List& operator=(const List& l) {
    this->clear();
    for (auto i = l.cbegin(); i != l.cend(); i++) this->push_back(*i);
    return *this;
  };

  const_reference front() { return *this->begin(); };

  const_reference back() { return *(this->end() - 1); };

  const_iterator cbegin() const {
    const_iterator it(null_node_);
    if (size_ > 0) ++it;
    return it;
  };

  const_iterator cend() const { return const_iterator(null_node_); };

  iterator begin() {
    iterator it(null_node_);
    if (size_ > 0) ++it;
    return it;
  };

  iterator end() { return iterator(null_node_); };

  bool empty() { return !(bool)size_; };

  size_type size() { return size_; };

  size_type max_size() const {
    return std::numeric_limits<size_type>::max() / (2 * sizeof(Node));
  };

  void clear() {
    if (size_ > 0) {
      auto it = this->begin();
      while (it != this->end()) {
        iterator tmp = it + 1;
        this->erase(it);
        it = tmp;
      }
    }
  };

  iterator insert(iterator pos, const_reference value) {
    Node* next = pos.node();
    Node* prev = (pos - 1).node();
    Node* tmp = new Node(value, next, prev);
    prev->set_next(tmp);
    next->set_prev(tmp);
    ++size_;
    return iterator(tmp);
  };

  void erase(iterator pos) {
    Node* next = (pos + 1).node();
    Node* prev = (pos - 1).node();
    next->set_prev(prev);
    prev->set_next(next);
    --size_;
    delete pos.node();
  };

  void push_back(const_reference value) { insert(this->end(), value); };

  void pop_back() {
    if (size_) erase(this->end() - 1);
  };

  void push_front(const_reference value) { insert(this->begin(), value); };

  void pop_front() {
    if (size_) erase(this->begin());
  };

  void swap(List& other) {
    List<Node*> this_addresses;
    List<Node*> other_addresses;
    this->extract_addresses(this_addresses);
    other.extract_addresses(other_addresses);
    this->insert_addresses(other_addresses);
    other.insert_addresses(this_addresses);

    size_type size_tmp = size_;
    size_ = other.size();
    other.size_ = size_tmp;
  };

  void merge(List& other) {
    auto it_1 = this->begin();
    auto it_2 = other.begin();
    while (it_2 != other.end()) {
      if (*it_2 <= *it_1) {
        auto tmp = it_2 + 1;
        node_hand_over(it_2, it_1, other, *this);
        it_2 = tmp;
      } else if (it_1 == this->end()) {
        auto tmp = it_2 + 1;
        node_hand_over(it_2, it_1, other, *this);
        it_2 = tmp;
        ++it_1;
      } else {
        ++it_1;
      }
    }
  };

  void splice(const_iterator pos, List& other) {
    if (!other.empty()) {
      List<Node*> other_adresses, zero;
      other.extract_addresses(other_adresses);
      other.size_ = 0;
      other.insert_addresses(zero);

      Node* next = pos.node();
      Node* prev = (pos - 1).node();
      next->set_prev(other_adresses.back());
      prev->set_next(other_adresses.front());

      Node* first = other_adresses.front();
      Node* last = other_adresses.back();
      first->set_prev(prev);
      last->set_next(next);
    }
  };

  void reverse() {
    iterator it = this->end();
    do {
      switch_next_prev(it);
      --it;
    } while (it != this->end());
  };

  void unique() {
    auto it_1 = this->begin();
    auto it_2 = it_1 + 1;
    while (it_2 != this->end()) {
      if (*it_1 == *it_2) {
        auto tmp = it_1;
        ++it_1;
        erase(tmp);
      } else {
        ++it_1;
      }
      ++it_2;
    }
  };

  void sort() {
    if (size_ > 1) {
      auto it_compared = this->begin();
      auto it_switch = it_compared + 1;
      while (it_switch != this->end()) {
        if (*it_switch < *it_compared) {
          auto tmp_switch = it_switch + 1;
          while (*it_switch <= *it_compared && it_compared != this->end())
            --it_compared;
          ++it_compared;
          node_hand_over(it_switch, it_compared, *this, *this);
          it_switch = tmp_switch;
          it_compared = it_switch - 1;
        } else {
          ++it_compared;
          ++it_switch;
        }
      }
    }
  };

 private:
  size_type size_ = 0;
  Node* null_node_ = new Node;

  void extract_addresses(List<Node*>& dest) {
    if (size_ > 0) {
      dest.push_back(null_node_->get_next());
      dest.push_back(null_node_->get_prev());
    }
  };

  void insert_addresses(List<Node*>& source) {
    if (source.empty()) {
      null_node_->set_next(null_node_);
      null_node_->set_prev(null_node_);
    } else {
      Node* tmp = source.front();
      tmp->set_prev(null_node_);
      null_node_->set_next(tmp);
      tmp = source.back();
      tmp->set_next(null_node_);
      null_node_->set_prev(tmp);
    }
  };

  void switch_next_prev(iterator& it) {
    Node* node = it.node();
    Node* buffer = node->get_next();
    node->set_next(node->get_prev());
    node->set_prev(buffer);
  };

  void node_hand_over(iterator& it_source, iterator& it_dest, List& source_list,
                      List& dest_list) {
    Node* source = it_source.node();
    Node* source_next = source->get_next();
    Node* source_prev = source->get_prev();
    source_next->set_prev(source_prev);
    source_prev->set_next(source_next);

    Node* dest_next = it_dest.node();
    Node* dest_prev = dest_next->get_prev();
    dest_next->set_prev(source);
    dest_prev->set_next(source);
    source->set_next(dest_next);
    source->set_prev(dest_prev);

    --source_list.size_;
    ++dest_list.size_;
  };
};
}  // namespace s21

#endif  // NAVIGATOR_SRC_INCLUDES_S21_LIST_H_
