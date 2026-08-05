#ifndef IMU_H
#define IMU_H

#include "Params.h"
#include <Wire.h>

class IMU {
public:
  // 센서의 측정값
  int16_t Tmp;
  Eigen::Matrix<int16_t, 3, 1> acc_raw_vec, gyr_raw_vec;
  Eigen::Matrix<float, 3, 1> acc_vec, gyr_vec;
  Eigen::Matrix<float, 3, 1> acc_vec_prev, gyr_vec_prev;
  Eigen::Matrix<float, 3, 1> acc_vec_prev_input, gyr_vec_prev_input;
  Eigen::Matrix<float, 3, 1> acc_vec_prev_prev_input, acc_vec_prev_prev_output;
  Eigen::Matrix<float, 3, 1> gyr_vec_prev_prev_input, gyr_vec_prev_prev_output;

  float temperature;  // 온도 (섭씨)

  // 캘리브레이션 값
  // Eigen::Vector3f gyro_bias{ -0.1230688696f, -0.0304898514f,	0.01379522641f};    // (rad/s)
  // Eigen::Vector3f accel_bias{ 0.1981802f,	-0.333633411f,	-0.36471631818f};  // (m/s^2)
  Eigen::Vector3f gyro_bias{-0.012987159f, 0.024731049f,
                            0.000443216f};  // rad/s
  Eigen::Vector3f accel_offset_raw{497.735f, 77.190f, 813.265f};
  Eigen::Vector3f accel_sensitivity{16362.115f, 16382.780f,
                                    16732.935f};  // raw/g
  // Eigen::Vector3f gyro_bias{ 0.f, 0.f, 0.f};    // (rad/s)
  // Eigen::Vector3f accel_bias{ 0.f,	0.f,	0.f};  // (m/s^2)
  

  // 생성자
  IMU() {}
  bool begin() {
    Wire1.begin(SDA_PIN, SCL_PIN, 400000);  // Dedicated MPU6050 I2C bus
    // clock frequency: 400kHz
    Wire1.beginTransmission(0x68);  // I2C 주소

    // 1. 슬립 모드 비활성화 (PWR_MGMT_1 레지스터)
    Wire1.write(0x6B);  // PWR_MGMT_1 레지스터
    Wire1.write(0x00);     // 슬립 모드 비활성화
    if (Wire1.endTransmission(true) != 0) {
      Serial.println("[Error] Failed to initialize MPU6050. Check connections!");
      return false;
    }

    // 2. DLPF 설정 (CONFIG 레지스터)
    Wire1.beginTransmission(0x68);  // I2C 주소
    Wire1.write(0x1A);              // CONFIG 레지스터
    Wire1.write(0x00);              // DLPF Off 설정
    // Wire.write(0x02);              // DLPF 94Hz 설정
    // Wire.write(0x05);  // DLPF 10Hz 설정
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
    Wire1.beginTransmission(0x68);  // I2C 주소
    Wire1.write(0x3B);              // 시작 레지스터 (ACCEL_XOUT_H)
    if (Wire1.endTransmission(false) != 0) {
      return false;  // 데이터 요청 실패
    }

    // 14바이트 요청
    Wire1.requestFrom(static_cast<uint8_t>(0x68), static_cast<size_t>(14), true);
    if (Wire1.available() < 14) {
      return false;  // 데이터 수신 실패
    }

    // 데이터 읽기 및 처리
    uint8_t buffer[14];
    for (int i = 0; i < 14; i++) {
      buffer[i] = Wire1.read();
    }

    // 가속도 데이터
    acc_raw_vec << (buffer[0] << 8 | buffer[1]),
      (buffer[2] << 8 | buffer[3]),
      (buffer[4] << 8 | buffer[5]);
    // 온도 데이터
    Tmp = buffer[6] << 8 | buffer[7];
    // 자이로 데이터
    gyr_raw_vec << (buffer[8] << 8 | buffer[9]),
      (buffer[10] << 8 | buffer[11]),
      (buffer[12] << 8 | buffer[13]);

    // 단위 변환 및 보정
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

    // 온도 변환
    temperature = Tmp / 340.0f + 36.53f;

    return true;
  }


  // 필터 적용 함수
  void applyFilters() {
    // 가속도에 LPF 적용
    // lowPassFilter(acc_vec, acc_vec_prev, 5.f);
    // 자이로에 HPF 적용
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
    // 필터 계수 계산
    float RC = 1.0f / (2.0f * M_PI * cutoffFreq);  // Time constant
    float alpha = dt / (dt + RC);
    // float cut_off_freq = exp(-2.0f * M_PI * cutoffFreq * dt);  // cut off frequency

    // LPF 적용
    input = alpha * input + (1.0f - alpha) * prevOutput;
    // input = cut_off_freq * input + (1.0f - cut_off_freq) * prevOutput;
    prevOutput = input;
  }

  void highPassFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevInput, Eigen::Vector3f& prevOutput, float cutoffFreq) {
    // 필터 계수 계산
    float RC = 1.0f / (2.0f * M_PI * cutoffFreq);  // Time constant
    float wc = 1 / RC;
    float alpha = RC / (dt + RC);
    // float alpha = exp(-wc*dt);


    // HPF 적용
    Eigen::Vector3f temp = input;                      // 현재 입력값을 임시 저장
    input = alpha * (prevOutput + input - prevInput);  // HPF 적용
    // Eigen::Vector3f temp = input;                             // 현재 입력값을 임시 저장
    // input = cut_off_freq * prevOutput + (input - prevInput);  // HPF 적용
    prevInput = temp;                                  // 이전 입력값 업데이트
    prevOutput = input;                                // 이전 출력값 업데이트
  }

  void notchFilter(Eigen::Vector3f& input, Eigen::Vector3f& prevInput, Eigen::Vector3f& prevPrevInput,
                   Eigen::Vector3f& prevOutput, Eigen::Vector3f& prevPrevOutput, float targetFreq, float Q) {
    // 필터 계수 계산
    float omega = 2.0f * M_PI * targetFreq * dt;  // 각 주파수 (라디안)
    float alpha = sin(omega) / (2.0f * Q);        // Q-factor에 따른 감쇠 계수
    float cosOmega = cos(omega);

    float b0 = 1.0f;
    float b1 = -2.0f * cosOmega;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cosOmega;
    float a2 = 1.0f - alpha;

    // 계수 정규화
    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;

    // 노치 필터 적용
    Eigen::Vector3f temp = input;
    input = b0 * input + b1 * prevInput + b2 * prevPrevInput
            - a1 * prevOutput - a2 * prevPrevOutput;

    // 이전 상태 업데이트
    prevPrevInput = prevInput;
    prevInput = temp;
    prevPrevOutput = prevOutput;
    prevOutput = input;
  }



  // 데이터를 Serial Plotter에 출력하는 함수
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
    Serial.print(temperature, 5);  // 줄바꿈
  }
};

#endif  // IMU_H
