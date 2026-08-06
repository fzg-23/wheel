#ifndef RECEIVER_H
#define RECEIVER_H

#include "HardwareSerial.h"
#include <sbus.h>
#include "Params.h"

class Receiver {
private:
  // command channel
  const int CH_ROLL = 0;    // Roll Channel
  const int CH_HEIGHT = 1;  // Height Channel
  const int CH_VEL = 2;     // Velocity Channel
  const int CH_YAW = 3;     // Yaw angular velocity Channel
  const int CH_ISRUN = 4;   // ISRUN Channel
  const int CH_RESET = 5;   // Estimator Reset

  // Receiver data min/max
  const int SBUS_MIN = 172;   // SBUS 最小值
  const int SBUS_MAX = 1810;  // SBUS最大

  bfs::SbusRx sbus_rx;
  bfs::SbusData data;
  float h_d, phi_d, v_d, dpsi_d;

public:
  // 构造函数
  Receiver(HardwareSerial& serial)
    : sbus_rx(&serial, SBUS_RX_PIN, -1, true) {
      h_d = HEIGHT_MAX;
      phi_d = 0;
      v_d = 0;
      dpsi_d = 0;
    }

  // SBUS初始化
  void begin() {
    sbus_rx.Begin();
  }

  // 读取SBUS数据
  bool readData() {
    return sbus_rx.Read();
  }

  void updateData() {
    data = sbus_rx.data();
  }

  // 检查状态（开/关）
  bool isRun() {
    return (data.ch[CH_ISRUN] == SBUS_MAX);  // 分段阅读_第 1810 章
  }

  bool isReset() {
    return (data.ch[CH_RESET] == SBUS_MAX);  // 分段阅读_第 1810 章
  }

  // 获取频道数据
  void updateDesiredStates() {
    h_d = mapToRange(data.ch[CH_HEIGHT], HEIGHT_MIN, HEIGHT_MAX);
    phi_d = mapToRange(data.ch[CH_ROLL], PHI_MIN, PHI_MAX);
    v_d = mapToRange(data.ch[CH_VEL], -VEL_MAX, VEL_MAX);
    dpsi_d = -mapToRange(data.ch[CH_YAW], -YAW_MAX, YAW_MAX);
  }

  float getDesiredHeight() {
    return h_d;
  }

  float getDesiredRoll() {
    return phi_d;
  }

  float getDesiredVel() {
    return v_d;
  }

  float getDesiredYawVel() {
    return dpsi_d;
  }

  void printDesiredDatas() {
    Serial.print("h_d: ");
    Serial.print(h_d);
    Serial.print(",phi_d: ");
    Serial.print(phi_d);
    Serial.print(",v_d: ");
    Serial.print(v_d);
    Serial.print(",dpsi_d: ");
    Serial.print(dpsi_d);
  }

  // 值映射函数
  float mapToRange(int value, float out_min, float out_max) {
    float scaled_value = (static_cast<float>(value) - static_cast<float>(SBUS_MIN))
                         / (static_cast<float>(SBUS_MAX) - static_cast<float>(SBUS_MIN));
    float mapped_value = scaled_value * (out_max - out_min) + out_min;
    return mapped_value;  // 返回双精度结果
  }
};

#endif  // RECEIVER_H
