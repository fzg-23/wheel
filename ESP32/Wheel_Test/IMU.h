#ifndef IMU_H
#define IMU_H

#include "Params.h"
#include <Wire.h>

class IMU {
public:
  // 传感器的测量值
  int16_t Tmp;
  Eigen::Matrix<int16_t, 3, 1> acc_raw_vec, gyr_raw_vec;
  Eigen::Matrix<float, 3, 1> acc_vec, gyr_vec;
  Eigen::Matrix<float, 3, 1> acc_vec_prev, gyr_vec_prev;
  float temperature;  // 温度（摄氏度）

  // 校准值
  Eigen::Vector3f gyro_bias{ -0.118868900f, -0.0329653600f, 0.0327140395f };    // (rad/s)
  Eigen::Vector3f accel_bias{ 0.6031676618f, -0.3592370879f, -0.9186593854f };  // (m/s^2)

  // 构造函数
  IMU() {}

  bool begin() {
    Wire.begin(SDA_PIN, SCL_PIN, 400000);  // SDA、SCL 引脚和时钟速度设置
    // clock frequency: 400kHz
    Wire.beginTransmission(0x68);          // I2C地址

    // 1. 禁用睡眠模式（PWR_MGMT_1寄存器）
    Wire.write(0x6B);  // PWR_MGMT_1 寄存器
    Wire.write(0);     // 禁用睡眠模式
    if (Wire.endTransmission(true) != 0) {
      Serial.println("[Error] Failed to initialize MPU6050. Check connections!");
      return false;
    }

    // 2. DLPF设置（CONFIG寄存器）
    Wire.beginTransmission(0x68);  // I2C地址
    Wire.write(0x1A);              //配置寄存器
    Wire.write(0x02);              //DLPF 94Hz 设置
    //Wire.write(0x05);              // DLPF 10Hz 设置
    if (Wire.endTransmission(true) != 0) {
      Serial.println("[Error] Failed to set DLPF. Check connections!");
      return false;
    }

    return true;
  }


  void setZero() {
    acc_raw_vec.setZero();
    gyr_raw_vec.setZero();
    acc_vec.setZero();
    gyr_vec.setZero();
    acc_vec_prev.setZero();
    gyr_vec_prev.setZero();
    temperature = 0;  // 温度（摄氏度）
  }

  void getIMUMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z.segment<3>(0) = acc_vec;
    z.segment<3>(3) = gyr_vec;
  }

  bool readData() {
    Wire.beginTransmission(0x68);  // I2C地址
    Wire.write(0x3B);              // 启动寄存器（ACCEL_XOUT_H）
    if (Wire.endTransmission(false) != 0) {
      return false;  // 数据请求失败
    }

    // 请求 14 个字节
    Wire.requestFrom(0x68, 14, true);
    if (Wire.available() < 14) {
      return false;  // 数据接收失败
    }

    // 数据读取与处理
    uint8_t buffer[14];
    for (int i = 0; i < 14; i++) {
      buffer[i] = Wire.read();
    }

    // 加速度数据
    acc_raw_vec << (buffer[0] << 8 | buffer[1]),
      (buffer[2] << 8 | buffer[3]),
      (buffer[4] << 8 | buffer[5]);
    // 温度数据
    Tmp = buffer[6] << 8 | buffer[7];
    // 陀螺仪数据
    gyr_raw_vec << (buffer[8] << 8 | buffer[9]),
      (buffer[10] << 8 | buffer[11]),
      (buffer[12] << 8 | buffer[13]);

    // 单位换算及修正
    acc_vec = (acc_raw_vec.cast<float>() / 16384.0f) * 9.80665f - accel_bias;
    gyr_vec = (gyr_raw_vec.cast<float>() / 131.0f) * M_PI / 180 - gyro_bias;

    applyFilters();

    // 温度转换
    temperature = Tmp / 340.0f + 36.53f;

    return true;
  }


  // 过滤器应用功能
  void applyFilters() {
    float alphaLPF = cutoffFrequencyLPF(10);   // LPF参考频率10Hz
    float alphaHPF = cutoffFrequencyHPF(0.5);  // HPF参考频率0.5Hz

    // 将LPF应用于加速
    // lowPassFilter(acc_vec, acc_vec_prev, alphaLPF);

    // 将 HPF 应用于陀螺仪
    // highPassFilter(gyr_vec, gyr_vec_prev, alphaHPF);
  }

  void lowPassFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevOutput, const float alpha) {
    input = alpha * input + (1 - alpha) * prevOutput;
    prevOutput = input;
  }

  void highPassFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevOutput, const float alpha) {
    Eigen::Vector3f temp = input;
    input = alpha * (prevOutput + input - prevOutput);
    prevOutput = temp;
  }

  float cutoffFrequencyLPF(float f_c) {
    float tau = 1.0 / (2.0 * M_PI * f_c);
    return dt / (tau + dt);
  }

  float cutoffFrequencyHPF(float f_c) {
    float tau = 1.0 / (2.0 * M_PI * f_c);
    return tau / (tau + dt);
  }

  // 将数据输出到串行绘图仪的功能
  void printData() {
    Serial.print("Accel_X:");
    Serial.print(acc_vec(0), 5);
    Serial.print(" ");
    Serial.print("Accel_Y:");
    Serial.print(acc_vec(1), 5);
    Serial.print(" ");
    Serial.print("Accel_Z:");
    Serial.print(acc_vec(2), 5);
    Serial.print(" ");
    Serial.print("Gyro_X:");
    Serial.print(gyr_vec(0), 5);
    Serial.print(" ");
    Serial.print("Gyro_Y:");
    Serial.print(gyr_vec(1), 5);
    Serial.print(" ");
    Serial.print("Gyro_Z:");
    Serial.print(gyr_vec(2), 5);
    Serial.print(" ");
    Serial.print("Temperature:");
    Serial.print(temperature, 5);  // 换行符
  }
};

#endif  // IMU_H
