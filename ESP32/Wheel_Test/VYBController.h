#ifndef VYB_CONTROLLER_H
#define VYB_CONTROLLER_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include "Params.h"
#include "MGServo.h"

/**
 * @class VYBController
 * @brief 제어 시스템의 LQR 기반 동작을 처리하는 클래스
 */
class VYBController {
private:
  MGServo& ServoRW;  ///< 우측 휠 서보
  MGServo& ServoLW;  ///< 좌측 휠 서보

  std::vector<Eigen::Matrix<float, 2, 4>> Ks;  ///< LQR 게인 행렬들의 벡터
  Eigen::Matrix<float, 2, 4> K;                ///< 현재 사용 중인 LQR 게인
  Eigen::Matrix<float, 2, 1> u;                ///< 제어 입력 벡터

  float iq_factor;         ///< 전류 변환 계수 (A/LSB)
  float torque_constant;   ///< 토크 상수 (Nm/A)
  float saturation;        ///< input saturation
  const int RW_bias = 12;  ///< 우측 휠 모터의 바이어스 값

public:
  /**
   * @brief 생성자: VYBController 초기화
   * @param ServoRW_ 우측 휠 서보 객체 참조
   * @param ServoLW_ 좌측 휠 서보 객체 참조
   */
  VYBController(MGServo& ServoRW_, MGServo& ServoLW_)
    : ServoRW(ServoRW_), ServoLW(ServoLW_) {
    // 전류 및 토크 상수 초기화
    iq_factor = 0.01611328f;  // (A/LSB) 33 / 2048
    torque_constant = 0.07f;  // (Nm/A)
    saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;


    // LQR 게인 초기화 (하드코딩된 데이터 삽입)
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
   * @brief 현재 제어 입력 벡터를 반환
   * @return Eigen::Matrix<float, 2, 1> u
   */
  Eigen::Matrix<float, 2, 1> getInputVector() {
    return u;
  }

  /**
   * @brief 모터 속도 측정을 수행하여 측정 벡터에 반영
   * @param z 모터 속도 측정값을 저장할 벡터
   */
  void getMotorSpeedMeasurement(Eigen::Matrix<float, 8, 1>& z) {
    z(6) = ServoRW.getMotorSpeed() * M_PI / 180;
    z(7) = ServoLW.getMotorSpeed() * M_PI / 180;
  }

  /**
  * @brief 모터 Current 측정값 update
  * @param iq_vec 모터 current 측정값을 저장할 벡터
  */
  void getMotorCurrentMeasurement(Eigen::Matrix<float, 2, 1>& iq_vec) {
    iq_vec << ServoRW.getMotorIq(), ServoLW.getMotorIq();
  }

  /**
  * @brief 모터 Current raw 측정값 update
  * @param iq_raw_vec 모터 current 측정값을 저장할 벡터
  */
  void getMotorCurrentMeasurement(Eigen::Matrix<int16_t, 2, 1>& iq_raw_vec) {
    iq_raw_vec << ServoRW.getMotorIqRaw(), ServoLW.getMotorIqRaw();
  }

  /**
   * @brief 현재 높이에 따라 LQR 게인 K를 계산
   * @param h 현재 높이 (m)
   */
  void computeGainK(const float h) {
    float temp = (h - HEIGHT_MIN) / 0.01;  // 구간을 10mm당 하나씩 나눔
    int idx = static_cast<int>(temp);      // 구간의 정수 인덱스 계산

    if (idx >= 0 && idx < static_cast<int>(Ks.size()) - 1) {
      // 보간 비율 계산
      float ratio = temp - idx;  // 현재 위치가 구간 내에서 차지하는 비율

      // 보간 수행
      K = Ks.at(idx) * (1.0f - ratio) + Ks.at(idx + 1) * ratio;
    } else if (idx < 0) {
      // h가 HEIGHT_MIN 이하일 경우 최소값 사용
      K = Ks.front();
    } else {
      // h가 범위를 벗어날 경우 최대값 사용
      K = Ks.back();
    }
  }

  /**
   * @brief 상태 벡터를 기반으로 제어 입력 벡터를 계산
   * @param x_d 목표 상태 벡터
   * @param x 현재 상태 벡터
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
 * @brief 계산된 제어 명령을 서보에 전송
 */
  void sendControlCommand() {
    float u_RW = u(0) / (iq_factor * torque_constant);
    float u_LW = u(1) / (iq_factor * torque_constant);

    // Right Wheel motor의 마찰로 인해 발생하는 torque 문제를 조정해줌
    if (u_RW < 0) {
      u_RW -= RW_bias;
    } else if (u_RW > 0) {
      u_RW += RW_bias;
    }

    ServoRW.sendTorqueControlCommand(static_cast<int16_t>(u_RW));
    ServoLW.sendTorqueControlCommand(static_cast<int16_t>(u_LW));
  }

  /**
 * @brief 직접 제어 명령을 서보에 전송
 * @param iq_inputs 두 개의 바퀴에 대한 제어 신호 (Right Wheel, Left Wheel)
 */
  void sendDirectControlCommand(Eigen::Matrix<int16_t, 2, 1> iq_inputs) {
    int16_t u_RW = iq_inputs(0);
    int16_t u_LW = iq_inputs(1);

    ServoRW.sendTorqueControlCommand(u_RW);
    ServoLW.sendTorqueControlCommand(u_LW);
  }
};

#endif  // VYB_CONTROLLER_H
