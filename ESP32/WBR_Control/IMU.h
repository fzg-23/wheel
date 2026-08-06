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
  Eigen::Matrix<float, 3, 1> acc_vec_prev_input, gyr_vec_prev_input;
  Eigen::Matrix<float, 3, 1> acc_vec_prev_prev_input, acc_vec_prev_prev_output;
  Eigen::Matrix<float, 3, 1> gyr_vec_prev_prev_input, gyr_vec_prev_prev_output;

  float temperature;  // 温度（摄氏度）

  // 校准值
  // Eigen::Vector3f gyro_bias{ -0.1230688696f, -0.0304898514f,	0.01379522641f};    // (rad/s)
  // Eigen::Vector3f accel_bias{ 0.1981802f,	-0.333633411f,	-0.36471631818f};  // (m/s^2)
  Eigen::Vector3f gyro_bias{-0.012987159f, 0.024731049f,
                            0.000443216f};  // rad/s
  Eigen::Vector3f accel_offset_raw{497.735f, 77.190f, 813.265f};
  Eigen::Vector3f accel_sensitivity{16362.115f, 16382.780f,
                                    16732.935f};  // raw/g
  // Eigen::Vector3f gyro_bias{ 0.f, 0.f, 0.f};    // (rad/s)
  // Eigen::Vector3f accel_bias{ 0.f,	0.f,	0.f};  // (m/s^2)
  

  // 构造函数
  IMU() {}
  bool begin() {
    Wire1.begin(SDA_PIN, SCL_PIN, 400000);  // Dedicated MPU6050 I2C bus
    // clock frequency: 400kHz
    Wire1.beginTransmission(0x68);  // I2C地址

    // 1. 禁用睡眠模式（PWR_MGMT_1寄存器）
    Wire1.write(0x6B);  // PWR_MGMT_1 寄存器
    Wire1.write(0x00);     // 禁用睡眠模式
    if (Wire1.endTransmission(true) != 0) {
      Serial.println("[Error] Failed to initialize MPU6050. Check connections!");
      return false;
    }

    // 2. DLPF设置（CONFIG寄存器）
    Wire1.beginTransmission(0x68);  // I2C地址
    Wire1.write(0x1A);              // 配置寄存器
    Wire1.write(0x00);              // DLPF 关闭设置
    // Wire.write(0x02);              // DLPF 94Hz 设置
    // Wire.write(0x05);  // DLPF 10Hz 设置
    if (Wire1.endTransmission(true) != 0) {
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
    acc_vec_prev_input.setZero();
    gyr_vec_prev_input.setZero();
    acc_vec_prev_prev_input.setZero();
    acc_vec_prev_prev_output.setZero();
    gyr_vec_prev_prev_input.setZero();
    gyr_vec_prev_prev_output.setZero();
    temperature = 0;
  }

  void getIMUMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z.segment<3>(0) = acc_vec;
    z.segment<3>(3) = gyr_vec;
  }

  bool readData() {
    Wire1.beginTransmission(0x68);  // I2C地址
    Wire1.write(0x3B);              // 启动寄存器（ACCEL_XOUT_H）
    if (Wire1.endTransmission(false) != 0) {
      return false;  // 数据请求失败
    }

    // 请求 14 个字节
    Wire1.requestFrom(static_cast<uint8_t>(0x68), static_cast<size_t>(14), true);
    if (Wire1.available() < 14) {
      return false;  // 数据接收失败
    }

    // 数据读取与处理
    uint8_t buffer[14];
    for (int i = 0; i < 14; i++) {
      buffer[i] = Wire1.read();
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
    acc_vec = (acc_raw_vec.cast<float>() - accel_offset_raw)
                  .cwiseQuotient(accel_sensitivity) *
              9.80665f;
    gyr_vec = (gyr_raw_vec.cast<float>() / 131.0f) * M_PI / 180 - gyro_bias;

    // MPU mounting to robot body frame: robot +X = sensor +X,
    // robot +Y (left) = sensor -Y, therefore robot +Z = sensor -Z.
    // This is a proper 180-degree rotation about the X axis and must be
    // applied identically to acceleration and angular velocity.
    acc_vec.y() = -acc_vec.y();
    acc_vec.z() = -acc_vec.z();
    gyr_vec.y() = -gyr_vec.y();
    gyr_vec.z() = -gyr_vec.z();

    applyFilters();

    // 温度转换
    temperature = Tmp / 340.0f + 36.53f;

    return true;
  }


  // 过滤器应用功能
  void applyFilters() {
    // 将LPF应用于加速
    // lowPassFilter(acc_vec, acc_vec_prev, 5.f);
    // 将 HPF 应用于陀螺仪
    // highPassFilter(gyr_vec, gyr_vec_prev_input, gyr_vec_prev, 5.f);


    // lowPassFilter(acc_vec, acc_vec_prev, 5.f);
    // lowPassFilter(gyr_vec, gyr_vec_prev, 5.f);
    // highPassFilter(acc_vec, acc_vec_prev_input, acc_vec_prev, 15.f);
    // highPassFilter(gyr_vec, gyr_vec_prev_input, gyr_vec_prev, 15.f);

    // notchFilter(acc_vec, acc_vec_prev_input, acc_vec_prev_prev_input,
    //             acc_vec_prev, acc_vec_prev_prev_output, 12.0f, 5.0f);
    // notchFilter(gyr_vec, gyr_vec_prev_input, gyr_vec_prev_prev_input,
    //             gyr_vec_prev, gyr_vec_prev_prev_output, 12.0f, 5.0f);

    // notchFilter(acc_vec, acc_vec_prev_input, acc_vec_prev_prev_input,
    //             acc_vec_prev, acc_vec_prev_prev_output, 20.0f, 5.0f);
    // notchFilter(gyr_vec, gyr_vec_prev_input, gyr_vec_prev_prev_input,
    //             gyr_vec_prev, gyr_vec_prev_prev_output, 20.0f, 5.0f);
  }

  void lowPassFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevOutput, float cutoffFreq) {
    // 过滤系数计算
    float RC = 1.0f / (2.0f * M_PI * cutoffFreq);  // Time constant
    float alpha = dt / (dt + RC);
    // float cut_off_freq = exp(-2.0f * M_PI * cutoffFreq * dt);  // cut off frequency

    // 应用低通滤波器
    input = alpha * input + (1.0f - alpha) * prevOutput;
    // input = cut_off_freq * input + (1.0f - cut_off_freq) * prevOutput;
    prevOutput = input;
  }

  void highPassFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevInput, Eigen::Vector3f& prevOutput, float cutoffFreq) {
    // 过滤系数计算
    float RC = 1.0f / (2.0f * M_PI * cutoffFreq);  // Time constant
    float wc = 1 / RC;
    float alpha = RC / (dt + RC);
    // float alpha = exp(-wc*dt);


    // 高通滤波器的应用
    Eigen::Vector3f temp = input;                      // 暂时保存当前输入值
    input = alpha * (prevOutput + input - prevInput);  // 高通滤波器的应用
    //Eigen::Vector3f 温度 = 输入；                             // 暂时存储当前输入值
    // 输入 = cut_off_freq * prevOutput + (输入 - prevInput);  // 应用 HPF
    prevInput = temp;                                  // 更新之前的输入
    prevOutput = input;                                // 更新之前的输出
  }

  void notchFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevInput, Eigen::Vector3f& prevPrevInput,
                   Eigen::Vector3f& prevOutput, Eigen::Vector3f& prevPrevOutput, float targetFreq, float Q) {
    // 过滤系数计算
    float omega = 2.0f * M_PI * targetFreq * dt;  // 角频率（弧度）
    float alpha = sin(omega) / (2.0f * Q);        // 根据 Q 因子的衰减系数
    float cosOmega = cos(omega);

    float b0 = 1.0f;
    float b1 = -2.0f * cosOmega;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosOmega;
    float a2 = 1.0f - alpha;

    // 系数归一化
    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;

    // 应用陷波滤波器
    Eigen::Vector3f temp = input;
    input = b0 * input + b1 * prevInput + b2 * prevPrevInput
            - a1 * prevOutput - a2 * prevPrevOutput;

    // 之前的状态更新
    prevPrevInput = prevInput;
    prevInput = temp;
    prevPrevOutput = prevOutput;
    prevOutput = input;
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
    Serial.print(temperature, 5);  //换行符
  }
};

#endif  // IMU_H
