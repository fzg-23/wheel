#ifndef PS_VECTOR_H
#define PS_VECTOR_H

#include <esp_heap_caps.h>
#include <cstring>
#include <stdexcept>
#include <type_traits>  // std::is_trivially_destructible

template<typename T>
class ps_vector {
private:
  T* data_;
  size_t size_;
  size_t capacity_;

  void resize(size_t new_capacity) {
    if (new_capacity <= capacity_)
      return;

    T* new_data = static_cast<T*>(
      heap_caps_malloc(new_capacity * sizeof(T), MALLOC_CAP_SPIRAM));
    if (!new_data)
      throw std::bad_alloc();

    if (data_) {
      std::memcpy(new_data, data_, size_ * sizeof(T));
      heap_caps_free(data_);
    }

    data_ = new_data;
    capacity_ = new_capacity;
  }

public:
  // 默认构造函数
  ps_vector()
    : data_(nullptr), size_(0), capacity_(0) {}

  // 复制构造函数
  ps_vector(const ps_vector& other)
    : data_(nullptr), size_(0), capacity_(0) {
    for (size_t i = 0; i < other.size_; ++i) {
      push_back(other.data_[i]);
    }
  }

  // 移动构造函数
  ps_vector(ps_vector&& other) noexcept
    : data_(other.data_), size_(other.size_), capacity_(other.capacity_) {
    other.data_ = nullptr;
    other.size_ = 0;
    other.capacity_ = 0;
  }

  // 复制赋值运算符
  ps_vector& operator=(const ps_vector& other) {
    if (this == &other) return *this;
    clear();
    for (size_t i = 0; i < other.size_; ++i) {
      push_back(other.data_[i]);
    }
    return *this;
  }

  //移动赋值运算符
  ps_vector& operator=(ps_vector&& other) noexcept {
    if (this == &other) return *this;
    clear();
    data_ = other.data_;
    size_ = other.size_;
    capacity_ = other.capacity_;
    other.data_ = nullptr;
    other.size_ = 0;
    other.capacity_ = 0;
    return *this;
  }

  // 析构函数
  ~ps_vector() {
    clear();
    if (data_) {
      heap_caps_free(data_);
    }
  }

  // 添加元素
  void push_back(const T& value) {
    if (size_ == capacity_) {
      size_t new_capacity = capacity_ == 0 ? 1 : capacity_ * 2;
      resize(new_capacity);
    }
    new (&data_[size_]) T(value);
    ++size_;
  }

  void emplace_back(T&& value) {
    if (size_ == capacity_) {
      size_t new_capacity = capacity_ == 0 ? 1 : capacity_ * 2;
      resize(new_capacity);
    }
    new (&data_[size_]) T(std::move(value));
    ++size_;
  }

  // 删除元素
  void pop_back() {
    if (size_ == 0)
      throw std::underflow_error("Vector is empty.");
    data_[--size_].~T();
  }

  // 插入元素
  void insert(size_t index, const T& value) {
    if (index > size_)
      throw std::out_of_range("Index out of range.");
    if (size_ == capacity_) {
      size_t new_capacity = capacity_ == 0 ? 1 : capacity_ * 2;
      resize(new_capacity);
    }
    // 推回
    for (size_t i = size_; i > index; --i) {
      data_[i] = data_[i - 1];
    }
    data_[index] = value;
    ++size_;
  }

  // 删除元素
  void erase(size_t index) {
    if (index >= size_)
      throw std::out_of_range("Index out of range.");
    for (size_t i = index; i < size_ - 1; ++i) {
      data_[i] = data_[i + 1];
    }
    --size_;
  }

  // 索引运算符
  T& operator[](size_t index) {
    if (index >= size_)
      throw std::out_of_range("Index out of range.");
    return data_[index];
  }

  const T& operator[](size_t index) const {
    if (index >= size_)
      throw std::out_of_range("Index out of range.");
    return data_[index];
  }

  // 范围安全索引访问
  T& at(size_t index) {
    if (index >= size_)
      throw std::out_of_range("Index out of range.");
    return data_[index];
  }

  const T& at(size_t index) const {
    if (index >= size_)
      throw std::out_of_range("Index out of range.");
    return data_[index];
  }

  // 第一个元素
  T& front() {
    if (size_ == 0)
      throw std::out_of_range("Vector is empty.");
    return data_[0];
  }

  const T& front() const {
    if (size_ == 0)
      throw std::out_of_range("Vector is empty.");
    return data_[0];
  }

  // 最后一个元素
  T& back() {
    if (size_ == 0)
      throw std::out_of_range("Vector is empty.");
    return data_[size_ - 1];
  }

  const T& back() const {
    if (size_ == 0)
      throw std::out_of_range("Vector is empty.");
    return data_[size_ - 1];
  }

  // 数据指针
  T* data() {
    return data_;
  }

  const T* data() const {
    return data_;
  }

  // 尺寸
  size_t size() const {
    return size_;
  }

  // 容量
  size_t capacity() const {
    return capacity_;
  }

  // 检查向量是否为空
  bool empty() const {
    return size_ == 0;
  }

  // 容量预留
  void reserve(size_t new_cap) {
    if (new_cap > capacity_) {
      resize(new_cap);
    }
  }

  // 产能减少
  void shrink_to_fit() {
    resize(size_);
  }

  void clear() {
    // 如果它是原始类型（可简单破坏），则省略析构函数调用。
    if constexpr (!std::is_trivially_destructible<T>::value) {
      for (size_t i = 0; i < size_; ++i) {
        data_[i].~T();  // 仅在需要析构函数时调用
      }
    }
    size_ = 0;
  }

  // 迭代器（用于使用基于范围的 for 语句）
  T* begin() {
    return data_;
  }

  const T* begin() const {
    return data_;
  }

  T* end() {
    return data_ + size_;
  }

  const T* end() const {
    return data_ + size_;
  }

  T* rbegin() {
    return data_ + size_ - 1;
  }

  const T* rbegin() const {
    return data_ + size_ - 1;
  }

  T* rend() {
    return data_ - 1;
  }

  const T* rend() const {
    return data_ - 1;
  }
};

#endif  // PS_VECTOR_H
