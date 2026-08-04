#ifndef HRCONTROLLER_H
#define HRCONTROLLER_H

#include "HardwareSerial.h"
#include <ESP32Servo.h>
#include "Params.h"

class HRController {
public:
  // 생성자
  HRController() {}

  // 서보 핀 연결
  void attachServos(int left_pin, int right_pin) {
    left_servo.attach(left_pin);
    right_servo.attach(right_pin);
  }

  // 서보 각도 설정
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
  // 서보 각도 범위
  const int LH_SERVO_MIN = 60;   // 왼쪽 서보 최소각도
  const int LH_SERVO_MAX = 160;  // 왼쪽 서보 최대각도
  const int LH_SERVO_C = 90;     // 왼쪽 서보 중앙값

  const int RH_SERVO_MIN = 20;   // 오른쪽 서보 최소각도
  const int RH_SERVO_MAX = 120;  // 오른쪽 서보 최대각도
  const int RH_SERVO_C = 90;     // 오른쪽 서보 중앙값

  Servo left_servo;
  Servo right_servo;

  Eigen::Vector2f theta_hips;
};

#endif  // HRCONTROLLER_H
