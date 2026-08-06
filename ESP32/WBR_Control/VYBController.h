#ifndef VYB_CONTROLLER_H
#define VYB_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <vector>
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

  std::vector<Eigen::Matrix<float, 2, 4>> Ks;  ///增益表
  Eigen::Matrix<float, 2, 4> K;                ///当前选中增益
  Eigen::Matrix<float, 2, 1> u;                ///输出

  float iq_factor;        ///电流与电机命令之间转化系数
  float torque_constant;  ///电流与扭矩之间转化系数
  //左右轮实测换算系数(一个电机命令产生多少扭矩)
  float LW_factor = 0.001043224f;
  float RW_factor = 0.000857902f;

  float AngleFixRate = 0.1;

  float saturation;  ///输出限幅
  Eigen::Matrix<float, 2, 1> saturation_vec;//限幅矩阵
  const int RW_bias = 12;  ///死区补偿

public:
  float theta_d;
  /**
   * @brief构造函数：VYBController初始化
   * @param ServoRW_ 右轮伺服对象引用
   * @param ServoLW_ 左轮伺服对象的引用。
   */
  VYBController(MGServo& ServoRW_, MGServo& ServoLW_)
    : ServoRW(ServoRW_), ServoLW(ServoLW_) {
    theta_d = 0.f;
    // 初始化电流和扭矩常数
    iq_factor = 0.001611328f;  // (A/LSB) 3.3 / 2048
    torque_constant = 0.7f;    // (Nm/A) * reduction ratio
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;
    // saturation_vec << MAX_TORQUE_COMMAND * RW_factor, MAX_TORQUE_COMMAND * LW_factor;
    saturation_vec << MAX_TORQUE, MAX_TORQUE;
    //初始化限幅矩阵[MAX_TORQUE]
    //             [MAX_TORQUE];



    // 原 LQR 增益表（保留供回退，不参与编译）
    Eigen::Matrix<float, 2, 4> mat;
#if 0
    //////////////////////////////////////////////////////////
    mat << 1.35316904f, 0.14009245f, 0.23358589f, -0.12161902f,
      -1.36517013f, -0.14243867f, -0.23405192f, -0.12179830f;
    Ks.push_back(mat);

    mat << 1.40047216f, 0.14638508f, 0.23369175f, -0.12160757f,
      -1.41285639f, -0.14885450f, -0.23407320f, -0.12184087f;
    Ks.push_back(mat);

    mat << 1.44602994f, 0.15279748f, 0.23382116f, -0.12158976f,
      -1.45881851f, -0.15539539f, -0.23413544f, -0.12186834f;
    Ks.push_back(mat);

    mat << 1.48968650f, 0.15931475f, 0.23397350f, -0.12156787f,
      -1.50289679f, -0.16204545f, -0.23423617f, -0.12188368f;
    Ks.push_back(mat);

    mat << 1.53149557f, 0.16592279f, 0.23414424f, -0.12154222f,
      -1.54514525f, -0.16879039f, -0.23436934f, -0.12188799f;
    Ks.push_back(mat);

    mat << 1.57156576f, 0.17260877f, 0.23432853f, -0.12151281f,
      -1.58567342f, -0.17561742f, -0.23452873f, -0.12188200f;
    Ks.push_back(mat);

    mat << 1.61001973f, 0.17936153f, 0.23452203f, -0.12147964f,
      -1.62460463f, -0.18251548f, -0.23470875f, -0.12186644f;
    Ks.push_back(mat);

    mat << 1.64697959f, 0.18617179f, 0.23472118f, -0.12144289f,
      -1.66206114f, -0.18947534f, -0.23490463f, -0.12184212f;
    Ks.push_back(mat);

    mat << 1.68256117f, 0.19303219f, 0.23492316f, -0.12140290f,
      -1.69815838f, -0.19648966f, -0.23511244f, -0.12180996f;
    Ks.push_back(mat);

    mat << 1.71687199f, 0.19993741f, 0.23512588f, -0.12136017f,
      -1.73300286f, -0.20355303f, -0.23532897f, -0.12177102f;
    Ks.push_back(mat);

    mat << 1.75001117f, 0.20688435f, 0.23532789f, -0.12131532f,
      -1.76669221f, -0.21066222f, -0.23555173f, -0.12172640f;
    Ks.push_back(mat);

    mat << 1.78207150f, 0.21387275f, 0.23552830f, -0.12126915f,
      -1.79931713f, -0.21781670f, -0.23577883f, -0.12167735f;
    Ks.push_back(mat);

    mat << 1.81314599f, 0.22090653f, 0.23572687f, -0.12122289f,
      -1.83096725f, -0.22501991f, -0.23600886f, -0.12162555f;
    Ks.push_back(mat);

    mat << 1.84334794f, 0.22799757f, 0.23592427f, -0.12117958f,
      -1.86174765f, -0.23228239f, -0.23624048f, -0.12157472f;
    Ks.push_back(mat);
#endif

    // 当前 LQR 增益表：m_Body=948.127 g，h=0.07:0.01:0.20 m，Ts=0.008 s
    // Q=diag([100, 0, 20, 5])，R=diag([150, 150])
    mat << 1.06669590f, 0.11065670f, 0.23233587f, -0.11821313f,
      -1.06761043f, -0.11154472f, -0.23172512f, -0.11892476f;
    Ks.push_back(mat);

    mat << 1.10137033f, 0.11604147f, 0.23249065f, -0.11824224f,
      -1.10227047f, -0.11692886f, -0.23188677f, -0.11893595f;
    Ks.push_back(mat);

    mat << 1.13518804f, 0.12155486f, 0.23266869f, -0.11825619f,
      -1.13608751f, -0.12245005f, -0.23206850f, -0.11893815f;
    Ks.push_back(mat);

    mat << 1.16797418f, 0.12717946f, 0.23286789f, -0.11825915f,
      -1.16888007f, -0.12808539f, -0.23227121f, -0.11893132f;
    Ks.push_back(mat);

    mat << 1.19971506f, 0.13290032f, 0.23308362f, -0.11825186f,
      -1.20063316f, -0.13381866f, -0.23249101f, -0.11891523f;
    Ks.push_back(mat);

    mat << 1.23044791f, 0.13870439f, 0.23331115f, -0.11823463f,
      -1.23138390f, -0.13963663f, -0.23272330f, -0.11888991f;
    Ks.push_back(mat);

    mat << 1.26022866f, 0.14458059f, 0.23354631f, -0.11820790f,
      -1.26118836f, -0.14552840f, -0.23296382f, -0.11885584f;
    Ks.push_back(mat);

    mat << 1.28911889f, 0.15051980f, 0.23378573f, -0.11817244f,
      -1.29010822f, -0.15148514f, -0.23320894f, -0.11881394f;
    Ks.push_back(mat);

    mat << 1.31717975f, 0.15651481f, 0.23402684f, -0.11812937f,
      -1.31820460f, -0.15750006f, -0.23345573f, -0.11876554f;
    Ks.push_back(mat);

    mat << 1.34446908f, 0.16256046f, 0.23426785f, -0.11808004f,
      -1.34553522f, -0.16356845f, -0.23370196f, -0.11871227f;
    Ks.push_back(mat);

    mat << 1.37104064f, 0.16865384f, 0.23450773f, -0.11802594f,
      -1.37215362f, -0.16968797f, -0.23394605f, -0.11865598f;
    Ks.push_back(mat);

    mat << 1.39694591f, 0.17479501f, 0.23474614f, -0.11796877f,
      -1.39811100f, -0.17585935f, -0.23418700f, -0.11859884f;
    Ks.push_back(mat);

    mat << 1.42224188f, 0.18098861f, 0.23498344f, -0.11791125f,
      -1.42346352f, -0.18208796f, -0.23442430f, -0.11854408f;
    Ks.push_back(mat);

    mat << 1.44701623f, 0.18724797f, 0.23522056f, -0.11786094f,
      -1.44829559f, -0.18838740f, -0.23465749f, -0.11849987f;
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
  //读取左右电机速度，送入EKF
  void getMotorSpeedMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z(6) = ServoRW.getMotorSpeed() * M_PI / 180;
    z(7) = ServoLW.getMotorSpeed() * M_PI / 180;
  }

  /**
  * @brief 电机电流测量值更新
  * @param iq_vec 用于存储电机电流测量值的向量
  */
 //读取电机电流
  void getMotorCurrentMeasurement(Eigen::Matrix<float, 2, 1>& iq_vec) {
    iq_vec << ServoRW.getMotorIq(), ServoLW.getMotorIq();
  }

  /**
   * @brief 根据当前高度计算LQR增益K
   * @param h 当前高度（米）
   */
  //根据当前高度计算LQR增益K
  void computeGainK(const float h) {
    float temp = (h - HEIGHT_MIN) / 0.01;  //高度间隔10mm
    int idx = static_cast<int>(temp);      //把temp转化为整数

    if (idx >= 0 && idx < static_cast<int>(Ks.size()) - 1) {
      //如果高度处于增益表范围内，就在相邻两组之间插值
      float ratio = temp - idx;  // 当前位置在区间内所占的比例

      // 插值计算
      K = Ks.at(idx) * (1.0f - ratio) + Ks.at(idx + 1) * ratio;
    } else if (idx < 0) {
      // 使用第一组增益
      K = Ks.front();
    } else {
      // 使用最后一组增益
      K = Ks.back();
    }
  }

  /**
   * @brief 根据状态向量计算控制输入向量
   * @param x_d 目标状态向量
   * @param x 当前状态向量
   */
  void computeInput(Eigen::Matrix<float, 4, 1>& x_d, Eigen::Matrix<float, 4, 1>& x) {

    // if (x_d(0) - x(0) < 0) {
    //   theta_d -= AngleFixRate * dt;
    // } else {
    //   theta_d += AngleFixRate * dt;
    // }
    // x_d(0) = theta_d;

    u = K * (x_d - x);

    // // Input saturation
    // for (int j = 0; j < 2; j++) {
    //   if (u(j) > saturation) {
    //     u(j) = saturation;
    //   } else if (u(j) < -saturation) {
    //     u(j) = -saturation;
    //   }
    // }
    // Input saturation
    for (int j = 0; j < 2; j++) {
      if (u(j) > saturation_vec(j)) {
        u(j) = saturation_vec(j);
      } else if (u(j) < -saturation_vec(j)) {
        u(j) = -saturation_vec(j);
      }
    }
  }

  /**
   * @brief 将计算出的控制命令发送到伺服器
   */
    //发送平衡控制指令
  void sendControlCommand() {
    // float u_RW = u(0) / (iq_factor * torque_constant);
    // float u_LW = u(1) / (iq_factor * torque_constant);

    //// 调整右轮电机摩擦引起的扭矩问题
    // if (u_RW < 0) {
    //   u_RW -= RW_bias;
    // } else if (u_RW > 0) {
    //   u_RW += RW_bias;
    // }

    float u_RW = u(0) / RW_factor;
    float u_LW = u(1) / LW_factor;

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }

  void sendDirectControlCommand(Eigen::Matrix<float, 2, 1> u_) {
    float u_RW = u_(0) / (iq_factor * torque_constant);
    float u_LW = u_(1) / (iq_factor * torque_constant);

    // 调整右轮电机摩擦引起的扭矩问题。
    if (u_RW < 0) {
      u_RW -= RW_bias;
    } else if (u_RW > 0) {
      u_RW += RW_bias;
    }

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }
//请求读取电机状态
  void sendReadStateCommand() {
    ServoRW.sendCommandReadMotorState2();
    ServoLW.sendCommandReadMotorState2();
  }
};

#endif  // VYB_CONTROLLER_H
