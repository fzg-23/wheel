#ifndef TIMER_H
#define TIMER_H

#include <Arduino.h>  // 使用 millis() 和 micros() 函数的标头

class Timer {
public:
  enum class TimerType {
    Millis,  // 基于毫秒的计时器
    Micros   //基于微秒的定时器
  };

private:
  TimerType timer_type_;     // 定时器类型（毫利斯或微米）
  unsigned long start_time_; // 定时器开始时间

public:
  // 构造函数：设置定时器类型
  Timer(TimerType type) : timer_type_(type), start_time_(0) {}

  // 计时器开始
  void start() {
    start_time_ = (timer_type_ == TimerType::Millis) ? millis() : micros();
  }

  // 返回经过的时间
  unsigned long getDuration() const {
    unsigned long current_time = (timer_type_ == TimerType::Millis) ? millis() : micros();
    return current_time - start_time_;
  }

  // 定时器重置
  void reset() {
    start_time_ = 0;
  }
};

#endif  // TIMER_H
