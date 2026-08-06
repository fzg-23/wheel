#ifndef HRCONTROLLER_H
#define HRCONTROLLER_H

#include <Wire.h>
#include "Params.h"

class HRController {
public:
  // 构造函数
  HRController() {}

  // 伺服销连接
  bool begin() {
    return Wire.begin(LU9685_SDA_PIN, LU9685_SCL_PIN, 100000);
  }

  // 舵机角度设定
  void controlHipServos(const Eigen::Vector2f theta_hips_) {
    theta_hips = theta_hips_;

    int angle_RH = constrain(RH_SERVO_C - theta_hips(0) * 180 / M_PI, RH_SERVO_MIN, RH_SERVO_MAX);
    int angle_LH = constrain(LH_SERVO_C - theta_hips(1) * 180 / M_PI, LH_SERVO_MIN, LH_SERVO_MAX);

    writeServoAngle(RH_SERVO_CHANNEL, angle_RH);
    writeServoAngle(LH_SERVO_CHANNEL, angle_LH);
  }

  void printHipAngles() {
    Serial.print("theta_RH:");
    Serial.print(theta_hips(0));
    Serial.print(" theta_LH:");
    Serial.print(theta_hips(1));
  }

private:
  // 伺服角度范围
  const int LH_SERVO_MIN = 88;
  const int LH_SERVO_MAX = 171;
  const int LH_SERVO_C = 101;

  const int RH_SERVO_MIN = 15;
  const int RH_SERVO_MAX = 98;
  const int RH_SERVO_C = 85;

  bool writeServoAngle(uint8_t channel, uint8_t angle) {
    Wire.beginTransmission(LU9685_ADDRESS);
    Wire.write(channel);
    Wire.write(angle);
    return Wire.endTransmission() == 0;
  }

  Eigen::Vector2f theta_hips;
};

#endif  // HRCONTROLLER_H
