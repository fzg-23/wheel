#ifndef PARAMS_H
#define PARAMS_H

#include <math.h>
#include <ArduinoEigenDense.h>

//单位转换宏功能
#define MM_TO_M(mm) ((mm)*1e-3)
#define G_TO_KG(g) ((g)*1e-3)
#define GMM2_TO_KGM2(gmm2) ((gmm2)*1e-9)

// 波特率设置
#define RS485_BAUDRATE 460800  //必须与电机配套
#define SERIAL_BAUDRATE 115200

//引脚号定义
#define LU9685_SDA_PIN 8
#define LU9685_SCL_PIN 9
#define LU9685_ADDRESS 0x00
#define LH_SERVO_CHANNEL 1
#define RH_SERVO_CHANNEL 2

#define USE_SBUS_RECEIVER 0  // GPIO15/Serial2 is currently used by motor 2
#define USE_WIFI_LOGGER 0     // Serial-control bring-up: do not block on Wi-Fi
#define SBUS_RX_PIN 15       // Update before enabling SBUS

#define RS485_DE_RE 1  // Only used by non-automatic-direction transceivers
#define MOTOR1_RX_PIN 18
#define MOTOR1_TX_PIN 17
#define MOTOR2_RX_PIN 16
#define MOTOR2_TX_PIN 15
#define MOTOR_BAUDRATE 115200

#define SDA_PIN 10  // SDA MPU6050 pin
#define SCL_PIN 11  // SCL MPU6050 pin

// 范围设置（高度和 phi）
const float HEIGHT_MIN = 0.07;  // 最低高度（米）
const float HEIGHT_MAX = 0.2;   // 最大高度（米）

const float PHI_MIN = -15.0;  // phi最小值（度）
const float PHI_MAX = 15.0;   // phi 最大值（度）

const float VEL_MAX = 1;     // 最大速度（米/秒）
const float YAW_MAX = 1;  // 最大偏航角速度（rad/s）

const float MAX_TORQUE_COMMAND = 100.000;  // 最大扭矩指令
// 常量浮点数 MAX_TORQUE = 0.12f；  // 最大扭矩指令
// 常量浮点数 MAX_TORQUE = 0.48f；  // 最大扭矩指令
const float MAX_TORQUE = 0.75f;  // 最大扭矩指令


// 输入热点信息->只需输入信息即可连接Wi-Fi。
// const char* ssid = "正彬";       // 热点名称
// const char* 密码 = "james0928";  // 热点密码
// const char* ssid = "Woodaengtang";       // 热点名称
//const char* 密码 = "jonghyun1234";  // 热点密码

const char* ssid = "OSB";           // 热点名称
const char* password = "12345678";  // 热点密码

const float dt = 0.008;  // sampling time
// const float dt = 0.007;  // sampling time


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
    0.0725                    // R
  };

  // 原动力学参数（保留供回退，不参与编译）
#if 0
  // Mainbody
  // props.m_Body = G_TO_KG(1524.76209213f);  // 如果G_TO_KG函数中的返回值为浮点数，则添加f
  // props.CoM_Body = mmToMVector(Eigen::Matrix<float, 3, 1>(10.81299751f, -0.19574788f, 36.74451323f));
  // props.I_Body = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 4221074.20231635f, 26062.23004946f, 277168.19335956f,
  //                          26062.23004946f, 7437280.54727470f, 6416.10856472f,
  //                          277168.19335956f, 6416.10856472f, 8509987.98611482f)
  //                                   .finished());
  props.m_Body = G_TO_KG(1524.76209213f);  // 如果 G_TO_KG 函数中的返回值为浮点型，则添加 f。
  props.CoM_Body = mmToMVector(Eigen::Matrix<float, 3, 1>(13.71923256f, -0.22808627f, 34.91864017f));
  props.I_Body = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 4274811.10362144f, 21823.60087554f, 202865.50474913f,
                                   21823.60087554f, 7103674.50655196f, 7275.19023018f,
                                   202865.50474913f, 7275.19023018f, 8108785.33349067f)
                                    .finished());

  // Calf Link Left
  props.m_CL = G_TO_KG(319.23782393f);
  props.CoM_CL = mmToMVector(Eigen::Matrix<float, 3, 1>(172.54736867f, -3.72877629f, 7.27850779f));
  props.I_CL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 101346.64715298f, -5353.69484487f, -27779.40548575f,
                                 -5353.69484487f, 703160.42663078f, -258.16665555f,
                                 -27779.40548575f, -258.16665555f, 676774.26748659f)
                                  .finished());


  // Calf Link Right
  props.m_CR = G_TO_KG(319.23782393f);
  props.CoM_CR = mmToMVector(Eigen::Matrix<float, 3, 1>(172.54753946f, 3.72875356f, 7.27718211f));
  props.I_CR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 101347.04479182f, 5354.04268934f, -27779.76191434f,
                                 5354.04268934f, 703157.66050555f, 252.37915614f,
                                 -27779.76191434f, 252.37915614f, 676771.26272286f)
                                  .finished());


  // Thigh Link Active Left
  props.m_TAL = G_TO_KG(42.41994494f);
  props.CoM_TAL = mmToMVector(Eigen::Matrix<float, 3, 1>(-48.49051712f, 4.61247327f, 2.04421822f));
  props.I_TAL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5008.67928627f, 6441.33404437f, -848.78989403f,
                                  6441.33404437f, 48822.93805935f, 404.75116121f,
                                  -848.78989403f, 404.75116121f, 49903.74714712f)
                                   .finished());


  // Thigh Link Active Right
  props.m_TAR = G_TO_KG(42.41994494f);
  props.CoM_TAR = mmToMVector(Eigen::Matrix<float, 3, 1>(-48.49050768f, -4.61247326f, 2.04421032f));
  props.I_TAR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5008.68060146f, -6441.33591254f, -848.79062342f,
                                  -6441.33591254f, 48822.95563592f, -404.74966887f,
                                  -848.79062342f, -404.74966887f, 49903.76341241f)
                                   .finished());


  // Thigh Link Passive Left
  props.m_TPL = G_TO_KG(38.26139565f);
  props.CoM_TPL = mmToMVector(Eigen::Matrix<float, 3, 1>(-77.93299656f, 10.41097168f, -3.75891919f));
  props.I_TPL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5119.23827939f, 5810.74900473f, 2235.15477477f,
                                  5810.74900473f, 58048.93720347f, -778.21403282f,
                                  2235.15477477f, -778.21403282f, 58325.14231362f)
                                   .finished());


  // Thigh Link Passive Right
  props.m_TPR = G_TO_KG(38.26139565f);
  props.CoM_TPR = mmToMVector(Eigen::Matrix<float, 3, 1>(-77.93299656f, -10.41097168f, -3.75891919f));
  props.I_TPR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 5119.23827329f, -5810.74900488f, 2235.15477431f,
                                  -5810.74900488f, 58048.93720431f, 778.21403278f,
                                  2235.15477431f, 778.21403278f, 58325.14232056f)
                                   .finished());

  // Wheel Left
  props.m_LW = G_TO_KG(237.11770281f);
  props.CoM_LW = mmToMVector(Eigen::Matrix<float, 3, 1>(-0.00000687f, 0.43740164f, -0.00000028f));
  props.I_LW = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 352917.56444663f, -0.00684532f, -0.00133537f,
                                 -0.00684532f, 676120.35437132f, 0.00006188f,
                                 -0.00133537f, 0.00006188f, 352917.58100268f)
                                  .finished());


  // Wheel Right
  props.m_RW = G_TO_KG(214.11770281f);
  props.CoM_RW = mmToMVector(Eigen::Matrix<float, 3, 1>(-0.00000761f, 0.48438625f, -0.00000031f));
  props.I_RW = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 312911.58508430f, -0.00692278f, -0.00051598f,
                                 -0.00692278f, 598903.10955900f, 0.00005987f,
                                 -0.00051598f, 0.00005987f, 312911.64073952f)
                                  .finished());
#endif

  // 新动力学参数：/home/fzg/桌面/param(1).md
  // Mainbody
  props.m_Body = G_TO_KG(948.127f);
  props.CoM_Body = mmToMVector(Eigen::Matrix<float, 3, 1>(15.484f, -0.147f, 34.389f));
  props.I_Body = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 2636000.0f, 4369.971f, 70032.962f,
                                   4369.971f, 4630000.0f, 3016.742f,
                                   70032.962f, 3016.742f, 5330000.0f)
                                    .finished());

  // Calf Link Left
  props.m_CL = G_TO_KG(341.503f);
  props.CoM_CL = mmToMVector(Eigen::Matrix<float, 3, 1>(176.363988f, -2.977000f, 6.498837f));
  props.I_CL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 88801.584f, -5916.959f, -17279.432f,
                                 -5916.959f, 516300.000f, -256.839f,
                                 -17279.432f, -256.839f, 491998.416f)
                                  .finished());

  // Calf Link Right
  props.m_CR = G_TO_KG(341.503f);
  props.CoM_CR = mmToMVector(Eigen::Matrix<float, 3, 1>(168.662856f, 3.578000f, 6.154992f));
  props.I_CR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 106143.753f, 8301.241f, -32322.310f,
                                 8301.241f, 895700.000f, 350.165f,
                                 -32322.310f, 350.165f, 864256.247f)
                                  .finished());

  // Thigh Link Active Left
  props.m_TAL = G_TO_KG(19.091f);
  props.CoM_TAL = mmToMVector(Eigen::Matrix<float, 3, 1>(-40.856192f, 3.532000f, 3.316360f));
  props.I_TAL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 2861.914f, 3470.281f, -126.016f,
                                  3470.281f, 20707.064f, 223.834f,
                                  -126.016f, 223.834f, 21158.111f)
                                   .finished());

  // Thigh Link Active Right
  props.m_TAR = G_TO_KG(19.091f);
  props.CoM_TAR = mmToMVector(Eigen::Matrix<float, 3, 1>(-40.856192f, -3.532000f, 3.316360f));
  props.I_TAR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 2861.914f, -3470.281f, -126.018f,
                                  -3470.281f, 20707.060f, -223.833f,
                                  -126.018f, -223.833f, 21158.106f)
                                   .finished());

  // Thigh Link Passive Left
  props.m_TPL = G_TO_KG(19.314f);
  props.CoM_TPL = mmToMVector(Eigen::Matrix<float, 3, 1>(-63.299815f, 9.192000f, -5.740369f));
  props.I_TPL = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 3042.430f, 3456.981f, 90.386f,
                                  3456.981f, 32115.604f, -461.757f,
                                  90.386f, -461.757f, 32052.001f)
                                   .finished());

  // Thigh Link Passive Right
  props.m_TPR = G_TO_KG(19.314f);
  props.CoM_TPR = mmToMVector(Eigen::Matrix<float, 3, 1>(-63.299815f, -9.192000f, -5.740369f));
  props.I_TPR = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 3042.430f, -3456.981f, 90.386f,
                                  -3456.981f, 32115.604f, 461.757f,
                                  90.386f, 461.757f, 32052.001f)
                                   .finished());

  // Wheel Left
  props.m_LW = G_TO_KG(126.059f);
  props.CoM_LW = mmToMVector(Eigen::Matrix<float, 3, 1>(0.0f, 0.537f, 0.0f));
  props.I_LW = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 177500.0f, 0.0f, 0.0f,
                                 0.0f, 339000.0f, 0.0f,
                                 0.0f, 0.0f, 177500.0f)
                                  .finished());

  // Wheel Right
  props.m_RW = G_TO_KG(126.059f);
  props.CoM_RW = mmToMVector(Eigen::Matrix<float, 3, 1>(0.0f, 0.537f, 0.0f));
  props.I_RW = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 177500.0f, 0.0f, 0.0f,
                                 0.0f, 339000.0f, 0.0f,
                                 0.0f, 0.0f, 177500.0f)
                                  .finished());


  return props;
}

#endif
