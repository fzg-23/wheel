#ifndef PARAMS_H
#define PARAMS_H

#include <math.h>
#include <ArduinoEigenDense.h>

// 单位转换宏功能
#define MM_TO_M(mm) ((mm)*1e-3)
#define G_TO_KG(g) ((g)*1e-3)
#define GMM2_TO_KGM2(gmm2) ((gmm2)*1e-9)

// 引脚号定义
#define LH_PIN 13  // 左伺服销
#define RH_PIN 21  // 右伺服销

#define SBUS_RX_PIN 15  // SBUS接收引脚

#define RS485_DE_RE 1    // DE/RE 控制引脚
#define RS485_TX_PIN 40  // DI (TX) 引脚
#define RS485_RX_PIN 42  // RO (RX) 引脚

#define SDA_PIN 8   // SDA MPU6050引脚
#define SCL_PIN 17  //SCL MPU6050引脚

// 范围设置（高度和 phi）
const float HEIGHT_MIN = 0.07;  // 最低高度（米）
const float HEIGHT_MAX = 0.2;   // 最大高度（米）

const float PHI_MIN = -30.0;  // phi最小值（度）
const float PHI_MAX = 30.0;   // phi 最大值（度）

const float VEL_MAX = 1;  // 最大速度（米/秒）
const float YAW_MAX = 1.5;  // 最大偏航角速度（rad/s）

const float MAX_TORQUE_COMMAND = 100.000;  //最大扭矩指令

// 输入热点信息->只需输入信息即可连接Wi-Fi。
// const char* ssid = "正彬";       // 热点名称
//const char* 密码 = "james0928";  // 热点密码
// const char* ssid = "Woodaengtang";       // 热点名称
// const char* 密码 = "jonghyun1234";  // 热点密码
const char* ssid = "OSB";       // 热点名称
const char* password = "12345678";  // 热点密码


const float dt = 0.003;  // sampling time

// mm -> m 单位转换函数（向量）
template<typename T>
Eigen::Matrix<T, 3, 1> mmToMVector(const Eigen::Matrix<T, 3, 1>& vec) {
  return vec * static_cast<T>(1e-3);
}

// gmm^2 -> kgm^2 单位转换函数（矩阵）
template<typename T>
Eigen::Matrix<T, 3, 3> gmm2ToKgm2Matrix(const Eigen::Matrix<T, 3, 3>& mat) {
  return mat * static_cast<T>(1e-9);
}

// 属性结构定义
struct Properties {
  float a, b, l1, l2, l3, l4, l5, L, R;

  // 惯性和质量相关变量
  float m_Body;
  Eigen::Matrix<float, 3, 1> CoM_Body;
  Eigen::Matrix<float, 3, 3> I_Body;

  float m_TAR;  // Thigh Link Active Right
  Eigen::Matrix<float, 3, 1> CoM_TAR;
  Eigen::Matrix<float, 3, 3> I_TAR;

  float m_TAL;  // Thigh Link Active Left
  Eigen::Matrix<float, 3, 1> CoM_TAL;
  Eigen::Matrix<float, 3, 3> I_TAL;

  float m_TPR;  // Thigh Link Passive Right
  Eigen::Matrix<float, 3, 1> CoM_TPR;
  Eigen::Matrix<float, 3, 3> I_TPR;

  float m_TPL;  // Thigh Link Passive Left
  Eigen::Matrix<float, 3, 1> CoM_TPL;
  Eigen::Matrix<float, 3, 3> I_TPL;

  float m_CR;  // Calf Link Right
  Eigen::Matrix<float, 3, 1> CoM_CR;
  Eigen::Matrix<float, 3, 3> I_CR;

  float m_CL;  // Calf Link Left
  Eigen::Matrix<float, 3, 1> CoM_CL;
  Eigen::Matrix<float, 3, 3> I_CL;

  float m_RW;  // Wheel Right
  Eigen::Matrix<float, 3, 1> CoM_RW;
  Eigen::Matrix<float, 3, 3> I_RW;

  float m_LW;  // Wheel Left
  Eigen::Matrix<float, 3, 1> CoM_LW;
  Eigen::Matrix<float, 3, 3> I_LW;
};

// 属性默认设置功能
inline Properties createDefaultProperties() {
  Properties props = {
    0.075 * cos(M_PI / 6.0),  // a
    0.075 * sin(M_PI / 6.0),  // b
    0.106,                    // l1
    0.077,                    // l2
    0.050,                    // l3
    0.137,                    // l4
    0.008,                    // l5
    0.123,                    // L
    0.0762                    // R
  };

  // Mainbody
  props.m_Body = G_TO_KG(1414.04421338f);  // 如果 G_TO_KG 函数中的返回值为浮点型，则添加 f。
  props.CoM_Body = mmToMVector(Eigen::Matrix<float, 3, 1>(18.19659659f, -0.24888409f, 35.42269696f));
  props.I_Body = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 4191453.72496564f, 19611.02661734f, 183001.27422549f,
                                   19611.02661734f, 6619178.75424983f, 6399.95018693f,
                                   183001.27422549f, 6399.95018693f, 7735669.49472275f)
                                    .finished());

  // Calf Link Left
  props.m_CL = G_TO_KG(338.23782393f);
  props.CoM_CL = mmToMVector(Eigen::Matrix<float, 3, 1>(169.71073180f, -3.92577446f, 7.13663078f));
  props.I_CL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 108187.53374990f, -5688.74489609f, -34349.39868930f,
                                 -5688.74489609f, 849624.61514512f, -272.20346797f,
                                 -34349.39868930f, -272.20346797f, 820318.28517488f)
                                  .finished());

  // Calf Link Right
  props.m_CR = G_TO_KG(338.23782393f);
  props.CoM_CR = mmToMVector(Eigen::Matrix<float, 3, 1>(169.71091698f, 3.92574726f, 7.13537305f));
  props.I_CR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 108187.86830599f, 5689.13762061f, -34348.43584610f,
                                 5689.13762061f, 849621.31645905f, 266.26892472f,
                                 -34348.43584610f, 266.26892472f, 820314.86080829f)
                                  .finished());

  // Thigh Link Active Left
  props.m_TAL = G_TO_KG(42.41994494f);
  props.CoM_TAL = mmToMVector(Eigen::Matrix<float, 3, 1>(-48.46903323f, 4.61143087f, 2.04282631f));
  props.I_TAL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5006.89568398f, 6444.24555368f, -846.35023128f,
                                  6444.24555368f, 48849.83835373f, 404.38523501f,
                                  -846.35023128f, 404.38523501f, 49929.39396428f)
                                   .finished());

  // Thigh Link Active Right
  props.m_TAR = G_TO_KG(42.41994494f);
  props.CoM_TAR = mmToMVector(Eigen::Matrix<float, 3, 1>(-48.46902377f, -4.61143086f, 2.04281842f));
  props.I_TAR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5006.89699630f, -6444.24742526f, -846.35096499f,
                                  -6444.24742526f, 48849.85593152f, -404.38374370f,
                                  -846.35096499f, -404.38374370f, 49929.41023364f)
                                   .finished());

  // Thigh Link Passive Left
  props.m_TPL = G_TO_KG(39.26139565f);
  props.CoM_TPL = mmToMVector(Eigen::Matrix<float, 3, 1>(-77.21916722f, 10.27201342f, -3.86042407f));
  props.I_TPL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5253.53554832f, 5957.24686321f, 2247.31583435f,
                                  5957.24686321f, 60185.49451452f, -799.05607096f,
                                  2247.31583435f, -799.05607096f, 60398.22510949f)
                                   .finished());

  // Thigh Link Passive Right
  props.m_TPR = G_TO_KG(39.26139565f);
  props.CoM_TPR = mmToMVector(Eigen::Matrix<float, 3, 1>(-77.21916722f, -10.27201342f, -3.86042407f));
  props.I_TPR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5253.53554222f, -5957.24686336f, 2247.31583389f,
                                  -5957.24686336f, 60185.49451538f, 799.05607092f,
                                  2247.31583389f, 799.05607092f, 60398.22511646f)
                                   .finished());

  // Wheel Left
  props.m_LW = G_TO_KG(237.11770281f);
  props.CoM_LW = mmToMVector(Eigen::Matrix<float, 3, 1>(-0.00000687f, 0.43740164f, -0.00000028f));
  props.I_LW = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 377169.50236306f, -0.00684531f, -0.00114686f,
                                 -0.00684531f, 723999.02807526f, 0.00006205f,
                                 -0.00114686f, 0.00006205f, 377169.52206100f)
                                  .finished());

  // Wheel Right
  props.m_RW = G_TO_KG(214.11770281f);
  props.CoM_RW = mmToMVector(Eigen::Matrix<float, 3, 1>(-0.00000761f, 0.48438625f, -0.00000031f));
  props.I_RW = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 334104.60295742f, -0.00692262f, -0.00045120f,
                                 -0.00692262f, 640742.79507983f, 0.00005987f,
                                 -0.00045120f, 0.00005987f, 334104.65585037f)
                                  .finished());


  return props;
}

#endif
