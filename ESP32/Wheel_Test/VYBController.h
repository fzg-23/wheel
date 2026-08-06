#ifndef VYB_CONTROLLER_H
#define VYB_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include "Params.h"
#include "MGServo.h"

/**
 * @class VYBController
 * @brief 处理控制系统基于 LQR 的行为的类。
 */
class VYBController {
private:
  MGServo& ServoRW;  ///< 右轮舵机
  MGServo& ServoLW;  ///< 左轮舵机

  std::vector<Eigen::Matrix<float, 2, 4>> Ks;  ///< LQR 增益矩阵向量
  Eigen::Matrix<float, 2, 4> K;                ///< 当前使用的 LQR 增益
  Eigen::Matrix<float, 2, 1> u;                ///< 控制输入向量

  float iq_factor;         ///< 电流转换系数 (A/LSB)
  float torque_constant;   ///< 扭矩常数 (Nm/A)
  float saturation;        ///< input saturation
  const int RW_bias = 12;  ///< 右轮电机偏置值

public:
  /**
   * @brief构造函数：VYBController初始化
   * @param ServoRW_ 右轮伺服对象引用
   * @param ServoLW_ 左轮伺服对象的引用。
   */
  VYBController(MGServo& ServoRW_, MGServo& ServoLW_)
    : ServoRW(ServoRW_), ServoLW(ServoLW_) {
    // 初始化电流和扭矩常数
    iq_factor = 0.01611328f;  // (A/LSB) 33 / 2048
    torque_constant = 0.07f;  // (Nm/A)
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;


    // 初始化LQR增益（插入硬编码数据）
    Eigen::Matrix<float, 2, 4> mat;
    //////////////////////////////////////////////////////////
    mat << 1.08462239f, 0.12305272f, 0.19382449f, -0.19479993f,
      -1.10877978f, -0.12702366f, -0.19782372f, -0.19250106f;
    Ks.push_back(mat);

    mat << 1.13454252f, 0.12908051f, 0.19377222f, -0.19484792f,
      -1.15994509f, -0.13330291f, -0.19778948f, -0.19257405f;
    Ks.push_back(mat);

    mat << 1.18247271f, 0.13517704f, 0.19376463f, -0.19485655f,
      -1.20911107f, -0.13965707f, -0.19780903f, -0.19259824f;
    Ks.push_back(mat);

    mat << 1.22822571f, 0.14133210f, 0.19380100f, -0.19483537f,
      -1.25607926f, -0.14607383f, -0.19787917f, -0.19258459f;
    Ks.push_back(mat);

    mat << 1.27187628f, 0.14753870f, 0.19387452f, -0.19478663f,
      -1.30092440f, -0.15254568f, -0.19799221f, -0.19253632f;
    Ks.push_back(mat);

    mat << 1.31356399f, 0.15379065f, 0.19397781f, -0.19471129f,
      -1.34378880f, -0.15906634f, -0.19814025f, -0.19245517f;
    Ks.push_back(mat);

    mat << 1.35344225f, 0.16008252f, 0.19410421f, -0.19461047f,
      -1.38482905f, -0.16563050f, -0.19831622f, -0.19234296f;
    Ks.push_back(mat);

    mat << 1.39166084f, 0.16640989f, 0.19424814f, -0.19448583f,
      -1.42419769f, -0.17223381f, -0.19851408f, -0.19220205f;
    Ks.push_back(mat);

    mat << 1.42835960f, 0.17276955f, 0.19440513f, -0.19433973f,
      -1.46203648f, -0.17887311f, -0.19872882f, -0.19203546f;
    Ks.push_back(mat);

    mat << 1.46366649f, 0.17915983f, 0.19457175f, -0.19417517f,
      -1.49847430f, -0.18554664f, -0.19895636f, -0.19184681f;
    Ks.push_back(mat);

    mat << 1.49769806f, 0.18558107f, 0.19474555f, -0.19399568f,
      -1.53362759f, -0.19225457f, -0.19919351f, -0.19164030f;
    Ks.push_back(mat);

    mat << 1.53056218f, 0.19203646f, 0.19492502f, -0.19380546f,
      -1.56760289f, -0.19899973f, -0.19943784f, -0.19142077f;
    Ks.push_back(mat);

    mat << 1.56236614f, 0.19853400f, 0.19510961f, -0.19361074f,
      -1.60050406f, -0.20578947f, -0.19968755f, -0.19119521f;
    Ks.push_back(mat);

    mat << 1.59324034f, 0.20509105f, 0.19530021f, -0.19342572f,
      -1.63245203f, -0.21263951f, -0.19994099f, -0.19097907f;
    Ks.push_back(mat);
    /////////////////////////////////////////////////////////////////
  }

  /**
   * @brief 返回当前控制输入向量
   * @return Eigen::Matrix<float, 2, 1> u
   */
  Eigen::Matrix<float, 2, 1> getInputVector() {
    return u;
  }

  /**
   * @brief 进行电机速度测量并将其反映在测量向量中
   * @param z 用于存储电机速度测量值的向量。
   */
  void getMotorSpeedMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z(6) = ServoRW.getMotorSpeed() * M_PI / 180;
    z(7) = ServoLW.getMotorSpeed() * M_PI / 180;
  }

  /**
  * @brief 电机电流测量值更新
  * @param iq_vec 用于存储电机电流测量值的向量
  */
  void getMotorCurrentMeasurement(Eigen::Matrix<float, 2, 1>& iq_vec) {
    iq_vec << ServoRW.getMotorIq(), ServoLW.getMotorIq();
  }

  /**
  * @brief 电机电流原始测量值更新
  * @param iq_raw_vec 用于存储电机电流测量值的向量
  */
  void getMotorCurrentMeasurement(Eigen::Matrix<int16_t, 2, 1>& iq_raw_vec) {
    iq_raw_vec << ServoRW.getMotorIqRaw(), ServoLW.getMotorIqRaw();
  }

  /**
   * @brief 根据当前高度计算LQR增益K
   * @param h 当前高度（米）
   */
  void computeGainK(const float h) {
    float temp = (h - HEIGHT_MIN) / 0.01;  // 将截面每10mm分成1段
    int idx = static_cast<int>(temp);      // 计算区间的整数索引

    if (idx >= 0 && idx < static_cast<int>(Ks.size()) - 1) {
      // 插值比计算
      float ratio = temp - idx;  // 当前位置在该部分中的比例

      // 执行插值
      K = Ks.at(idx) * (1.0f - ratio) + Ks.at(idx + 1) * ratio;
    } else if (idx < 0) {
      // 如果 h 小于或等于 HEIGHT_MIN，则使用最小值
      K = Ks.front();
    } else {
      // 如果 h 超出范围，则使用最大值
      K = Ks.back();
    }
  }

  /**
   * @brief 根据状态向量计算控制输入向量
   * @param x_d 目标状态向量
   * @param x 当前状态向量
   */
  void computeInput(Eigen::Matrix<float, 4, 1>& x_d, Eigen::Matrix<float, 4, 1>& x) {
    u = K * (x_d - x) / 2;

    // Input saturation
    for (int j = 0; j < 2; j++) {
      if (u(j) > saturation) {
        u(j) = saturation;
      } else if (u(j) < -saturation) {
        u(j) = -saturation;
      }
    }
  }

  /**
 * @brief 将计算出的控制命令发送到伺服器
 */
  void sendControlCommand() {
    float u_RW = u(0) / (iq_factor * torque_constant);
    float u_LW = u(1) / (iq_factor * torque_constant);

    // 调整右轮电机摩擦引起的扭矩问题。
    if (u_RW < 0) {
      u_RW -= RW_bias;
    } else if (u_RW > 0) {
      u_RW += RW_bias;
    }

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }

  /**
 * @brief 直接向舵机传输控制命令
 * @param iq_inputs 两个轮子的控制信号（右轮、左轮）
 */
  void sendDirectControlCommand(Eigen::Matrix<int16_t, 2, 1> iq_inputs) {
    int16_t u_RW = iq_inputs(0);
    int16_t u_LW = iq_inputs(1);

    ServoRW.sendTorqueControlCommand(u_RW);
    ServoLW.sendTorqueControlCommand(u_LW);
  }
};

#endif  // VYB_CONTROLLER_H
