#ifndef HRCONTROLLER_H
#define HRCONTROLLER_H

#include "HardwareSerial.h"
#include <ESP32Servo.h>
#include "Params.h"

class HRController {
public:
  // 构造函数
  HRController() {}

  // 伺服销连接
  void attachServos(int left_pin, int right_pin) {
    left_servo.attach(left_pin);
    right_servo.attach(right_pin);
  }

  // 舵机角度设定
  void controlHipServos(const Eigen::Vector2f theta_hips_) {
    theta_hips = theta_hips_;

    int angle_RH = constrain(RH_SERVO_C - theta_hips(0) * 180 / M_PI, RH_SERVO_MIN, RH_SERVO_MAX);
    int angle_LH = constrain(LH_SERVO_C - theta_hips(1) * 180 / M_PI, LH_SERVO_MIN, LH_SERVO_MAX);

    right_servo.write(angle_RH);   
    left_servo.write(angle_LH);
  }

  void printHipAngles() {
    Serial.print("theta_RH:");
    Serial.print(theta_hips(0));
    Serial.print(" theta_LH:");
    Serial.print(theta_hips(1));
  }

private:
  //伺服角度范围
  const int LH_SERVO_MIN = 62;   // 左舵机最小角度
  const int LH_SERVO_MAX = 162;  // 左舵机最大角度
  const int LH_SERVO_C = 92;     // 左伺服中位

  const int RH_SERVO_MIN = 18;   // 右舵机最小角度
  const int RH_SERVO_MAX = 118;  // 右舵机最大角度
  const int RH_SERVO_C = 88;     // 右伺服中值

  Servo left_servo;
  Servo right_servo;

  Eigen::Vector2f theta_hips;
};

#endif  // HRCONTROLLER_H
