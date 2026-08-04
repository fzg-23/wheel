#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>  // millis() 및 micros() 함수 사용을 위한 헤더

class Timer {
public:
  enum class TimerType {
    Millis,  // millis 기반 타이머
    Micros   // micros 기반 타이머
  };

private:
  TimerType timer_type_;     // 타이머 타입 (Millis 또는 Micros)
  unsigned long start_time_; // 타이머 시작 시간

public:
  // 생성자: 타이머 타입 설정
  Timer(TimerType type) : timer_type_(type), start_time_(0) {}

  // 타이머 시작
  void start() {
    start_time_ = (timer_type_ == TimerType::Millis) ? millis() : micros();
  }

  // 경과 시간 반환
  unsigned long getDuration() const {
    unsigned long current_time = (timer_type_ == TimerType::Millis) ? millis() : micros();
    return current_time - start_time_;
  }

  // 타이머 리셋
  void reset() {
    start_time_ = 0;
  }
};

#endif  // TIMER_H
