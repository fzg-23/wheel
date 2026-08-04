#ifndef PARAMS_H
#define PARAMS_H

#include <math.h>
#include <ArduinoEigenDense.h>

// 단위 변환 매크로 함수
#define MM_TO_M(mm) ((mm)*1e-3)
#define G_TO_KG(g) ((g)*1e-3)
#define GMM2_TO_KGM2(gmm2) ((gmm2)*1e-9)

// BaudRate 설정
#define RS485_BAUDRATE 460800  // Motor와 같이 맞춰줘야함
#define SERIAL_BAUDRATE 115200

// 핀 번호 정의
#define LH_PIN 13  // 왼쪽 서보 핀
#define RH_PIN 21  // 오른쪽 서보 핀

#define SBUS_RX_PIN 15  // SBUS 수신 핀

#define RS485_DE_RE 1    // DE/RE 제어 핀
#define RS485_TX_PIN 40  // DI (TX) 핀
#define RS485_RX_PIN 42  // RO (RX) 핀

#define SDA_PIN 8   // SDA MPU6050 핀
#define SCL_PIN 17  // SCL MPU6050 핀

// 범위 설정 (height와 phi)
const float HEIGHT_MIN = 0.07;  // 최소 높이 (m)
const float HEIGHT_MAX = 0.2;   // 최대 높이 (m)

const float PHI_MIN = -15.0;  // phi 최소값 (degree)
const float PHI_MAX = 15.0;   // phi 최대값 (degree)

const float VEL_MAX = 1;     // 최대 속도 (m/s)
const float YAW_MAX = 1;  // 최대 yaw angular velocity (rad/s)

const float MAX_TORQUE_COMMAND = 100.000;  // 최대 torque command
// const float MAX_TORQUE = 0.12f;  // 최대 torque command
// const float MAX_TORQUE = 0.48f;  // 최대 torque command
const float MAX_TORQUE = 0.75f;  // 최대 torque command


// 핫스팟 정보 입력 -> 정보만 입력하면 와이파이 연결 된다.
// const char* ssid = "Jeongbin";       // 핫스팟 이름
// const char* password = "james0928";  // 핫스팟 비밀번호
// const char* ssid = "Woodaengtang";       // 핫스팟 이름
// const char* password = "jonghyun1234";  // 핫스팟 비밀번호

const char* ssid = "OSB";           // 핫스팟 이름
const char* password = "12345678";  // 핫스팟 비밀번호

const float dt = 0.008;  // sampling time
// const float dt = 0.007;  // sampling time


// mm -> m 단위 변환 함수 (벡터)
template<typename T>
Eigen::Matrix<T, 3, 1> mmToMVector(const Eigen::Matrix<T, 3, 1>& vec) {
  return vec * static_cast<T>(1e-3);
}

// gmm^2 -> kgm^2 단위 변환 함수 (행렬)
template<typename T>
Eigen::Matrix<T, 3, 3> gmm2ToKgm2Matrix(const Eigen::Matrix<T, 3, 3>& mat) {
  return mat * static_cast<T>(1e-9);
}

// Properties 구조체 정의
struct Properties {
  float a, b, l1, l2, l3, l4, l5, L, R;

  // 관성 및 질량 관련 변수들
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

// Properties 기본값 설정 함수
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

  // Mainbody
  // props.m_Body = G_TO_KG(1524.76209213f);  // G_TO_KG 함수에서 반환값이 float인 경우 f를 추가
  // props.CoM_Body = mmToMVector(Eigen::Matrix<float, 3, 1>(10.81299751f, -0.19574788f, 36.74451323f));
  // props.I_Body = gmm2ToKgm2Matrix((Eigen::Matrix<float, 3, 3>() << 4221074.20231635f, 26062.23004946f, 277168.19335956f,
  //                          26062.23004946f, 7437280.54727470f, 6416.10856472f,
  //                          277168.19335956f, 6416.10856472f, 8509987.98611482f)
  //                                   .finished());
  props.m_Body = G_TO_KG(1524.76209213f);  // G_TO_KG 함수에서 반환값이 float인 경우 f를 추가
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


  return props;
}

#endif
