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
  // 기본 생성자
  ps_vector()
    : data_(nullptr), size_(0), capacity_(0) {}

  // 복사 생성자
  ps_vector(const ps_vector& other)
    : data_(nullptr), size_(0), capacity_(0) {
    for (size_t i = 0; i < other.size_; ++i) {
      push_back(other.data_[i]);
    }
  }

  // 이동 생성자
  ps_vector(ps_vector&& other) noexcept
    : data_(other.data_), size_(other.size_), capacity_(other.capacity_) {
    other.data_ = nullptr;
    other.size_ = 0;
    other.capacity_ = 0;
  }

  // 복사 대입 연산자
  ps_vector& operator=(const ps_vector& other) {
    if (this == &other) return *this;
    clear();
    for (size_t i = 0; i < other.size_; ++i) {
      push_back(other.data_[i]);
    }
    return *this;
  }

  // 이동 대입 연산자
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

  // 소멸자
  ~ps_vector() {
    clear();
    if (data_) {
      heap_caps_free(data_);
    }
  }

  // 요소 추가
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

  // 요소 제거
  void pop_back() {
    if (size_ == 0)
      throw std::underflow_error("Vector is empty.");
    data_[--size_].~T();
  }

  // 요소 삽입
  void insert(size_t index, const T& value) {
    if (index > size_)
      throw std::out_of_range("Index out of range.");
    if (size_ == capacity_) {
      size_t new_capacity = capacity_ == 0 ? 1 : capacity_ * 2;
      resize(new_capacity);
    }
    // 뒤로 밀기
    for (size_t i = size_; i > index; --i) {
      data_[i] = data_[i - 1];
    }
    data_[index] = value;
    ++size_;
  }

  // 요소 삭제
  void erase(size_t index) {
    if (index >= size_)
      throw std::out_of_range("Index out of range.");
    for (size_t i = index; i < size_ - 1; ++i) {
      data_[i] = data_[i + 1];
    }
    --size_;
  }

  // 인덱스 연산자
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

  // 범위 안전 인덱스 접근
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

  // 첫 번째 요소
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

  // 마지막 요소
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

  // 데이터 포인터
  T* data() {
    return data_;
  }

  const T* data() const {
    return data_;
  }

  // 크기
  size_t size() const {
    return size_;
  }

  // 용량
  size_t capacity() const {
    return capacity_;
  }

  // 벡터 비어있음 확인
  bool empty() const {
    return size_ == 0;
  }

  // 용량 예약
  void reserve(size_t new_cap) {
    if (new_cap > capacity_) {
      resize(new_cap);
    }
  }

  // 용량 축소
  void shrink_to_fit() {
    resize(size_);
  }

  void clear() {
    // 기본 타입 (trivially destructible)일 경우 소멸자 호출을 생략
    if constexpr (!std::is_trivially_destructible<T>::value) {
      for (size_t i = 0; i < size_; ++i) {
        data_[i].~T();  // 소멸자가 필요한 경우에만 호출
      }
    }
    size_ = 0;
  }

  // 반복자 (범위 기반 for문을 사용하기 위한)
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
