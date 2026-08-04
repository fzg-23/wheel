#ifndef EKF_H
#define EKF_H

#include <ArduinoEigenDense.h>
#include "Params.h"
#include "POL.h"

class EKF {
private:
  Eigen::Matrix<float, 4, 4> P;       // 단위 행렬
  Eigen::Matrix<float, 4, 4> P_pred;  // 단위 행렬
  const Eigen::DiagonalMatrix<float, 8> R_cov;
  const Eigen::DiagonalMatrix<float, 4> Q_cov;
  Eigen::Matrix<float, 4, 4> F_mat;  // Jacobian of Dynamic model
  Eigen::Matrix<float, 8, 4> H;  // Jacobian of Observation model
  Eigen::Matrix<float, 4, 8> K_mat;  // Kalman Gain

  POL &Pol_ref;

  Eigen::Matrix<float, 4, 1> x, x_pred;  // state
  Eigen::Matrix<float, 8, 1> z, h_obs;   // measurement

public:
  EKF(POL &Pol_)
    : Pol_ref(Pol_),
      R_cov((Eigen::Matrix<float, 8, 1>() << 1.54239e-3f, 1.93287e-3f, 2.63191e-3f, 3.11351e-6f, 4.12642e-6f, 5.37135e-6f, 3.046e-6f, 3.046e-6f).finished().asDiagonal()),  // sensor noise covariance
      Q_cov((Eigen::Matrix<float, 4, 1>() <<  4e-5f, 1e-4f, 4e-5f, 1e-6f).finished().asDiagonal())
      // R_cov((Eigen::Matrix<float, 8, 1>() << 1.54239e-1f, 1.93287e-1f, 2.63191e-1f, 3.11351e-3f, 4.12642e-3f, 5.37135e-3f, 1e-5f, 1e-5f).finished().asDiagonal()),  // sensor noise covariance
      // Q_cov((Eigen::Matrix<float, 4, 1>() << 4e-5f, 1e-4f, 4e-5f, 1e-4f).finished().asDiagonal())
  {
    x.setZero();  // 상태 벡터 초기화
    x_pred.setZero();
    z.setZero();
    h_obs.setZero();
    F_mat.setZero();
    H.setZero();
    K_mat.setZero();
    P_pred = Eigen::Matrix<float, 4, 4>::Identity() * 1e-1;
  }

  void printAllEstimationData() {
    // P matrix
    Serial.println("P:");
    for (int i = 0; i < P.rows(); ++i) {
      for (int j = 0; j < P.cols(); ++j) {
        Serial.print(P(i, j), 6);  // 소수점 6자리까지 출력
        Serial.print(" ");
      }
      Serial.println();
    }

    // P_pred matrix
    Serial.println("P_pred:");
    for (int i = 0; i < P_pred.rows(); ++i) {
      for (int j = 0; j < P_pred.cols(); ++j) {
        Serial.print(P_pred(i, j), 6);  // 소수점 6자리까지 출력
        Serial.print(" ");
      }
      Serial.println();
    }

    // F matrix
    Serial.println("F:");
    for (int i = 0; i < F_mat.rows(); ++i) {
      for (int j = 0; j < F_mat.cols(); ++j) {
        Serial.print(F_mat(i, j), 6);  // 소수점 6자리까지 출력
        Serial.print(" ");
      }
      Serial.println();
    }

    // H matrix
    Serial.println("H:");
    for (int i = 0; i < H.rows(); ++i) {
      for (int j = 0; j < H.cols(); ++j) {
        Serial.print(H(i, j), 6);  // 소수점 6자리까지 출력
        Serial.print(" ");
      }
      Serial.println();
    }

    // K matrix
    Serial.println("K:");
    for (int i = 0; i < K_mat.rows(); ++i) {
      for (int j = 0; j < K_mat.cols(); ++j) {
        Serial.print(K_mat(i, j), 6);  // 소수점 6자리까지 출력
        Serial.print(" ");
      }
      Serial.println();
    }

    // x vector
    Serial.println("x:");
    for (int i = 0; i < x.rows(); ++i) {
      Serial.print(x(i, 0), 6);  // 소수점 6자리까지 출력
      Serial.println();
    }

    // x_pred vector
    Serial.println("x_pred:");
    for (int i = 0; i < x_pred.rows(); ++i) {
      Serial.print(x_pred(i, 0), 6);  // 소수점 6자리까지 출력
      Serial.println();
    }

    // z vector
    Serial.println("z:");
    for (int i = 0; i < z.rows(); ++i) {
      Serial.print(z(i, 0), 6);  // 소수점 6자리까지 출력
      Serial.println();
    }

    // h_obs vector
    Serial.println("h_obs:");
    for (int i = 0; i < h_obs.rows(); ++i) {
      Serial.print(h_obs(i, 0), 6);  // 소수점 6자리까지 출력
      Serial.println();
    }
  }


  void reset_estimator() {
    P = Eigen::Matrix<float, 4, 4>::Identity() * 0.1;
  }

  bool estimate_state(Eigen::Matrix<float, 4, 1> &x_, const Eigen::Matrix<float, 8, 1> &z_) {
    setState(x_);
    setMeasurement(z_);
    if (!predict()) {
      return false;
    }
    update();
    x_ = x;
    return true;
  }

  void setState(const Eigen::Matrix<float, 4, 1> &x_) {
    x = x_;
  }

  void setMeasurement(const Eigen::Matrix<float, 8, 1> &z_) {
    z = z_;
  }

  bool predict() {
    Pol_ref.setState(x);
    if (!Pol_ref.prepare_state_prediction()) {
      return false;
    }  // update M, nle, dM_dtheta, dnle_dtheta, dnle_dqdot
    // 예측 단계 수행
    Pol_ref.predict_state(x_pred, F_mat);                        // 상태 예측
    P_pred = F_mat * P * F_mat.transpose() + Q_cov.toDenseMatrix();  // 공분산 업데이트
    return true;
  }

  void update() {
    predict_measurement();                                                                        // h_obs 및 H 업데이트
    K_mat = P_pred * H.transpose() * (H * P_pred * H.transpose() + R_cov.toDenseMatrix()).inverse();  // 칼만 이득 계산
    x = x_pred + K_mat * (z - h_obs);                                                                 // 상태 업데이트
    P = (Eigen::Matrix<float, 4, 4>::Identity() - K_mat * H) * P_pred;                                // 오차 공분산 업데이트
  }


private:
  // 관측 모델 함수
  void predict_measurement() {
    float theta_ddot = Pol_ref.f(1);
    float v_dot = Pol_ref.f(2);
    float psi_ddot = Pol_ref.f(3);


    float theta = x_pred(0);
    float theta_dot = x_pred(1);
    float v = x_pred(2);
    float psi_dot = x_pred(3);

    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float cos_theta_2 = std::pow(cos_theta, 2);
    float sin_theta_2 = std::pow(sin_theta, 2);
    float sin_cos_theta = cos_theta * sin_theta;

    float h = Pol_ref.h;
    float g = Pol_ref.g;
    float L = Pol_ref.L;
    float R = Pol_ref.R;



    // h_obs 계산
    h_obs(0) = h * theta_ddot + v_dot * cos_theta - g * sin_theta - h * std::pow(psi_dot, 2) * sin_cos_theta;
    h_obs(1) = psi_dot * v + h * psi_ddot * sin_theta + h * psi_dot * theta_dot * cos_theta * 2.0;
    h_obs(2) = -h * std::pow(theta_dot, 2) + g * cos_theta + v_dot * sin_theta - std::pow(psi_dot, 2) * (h - h * cos_theta_2);
    h_obs(3) = -psi_dot * sin_theta;
    h_obs(4) = theta_dot;
    h_obs(5) = psi_dot * cos_theta;
    h_obs(6) = theta_dot - v / R - (L * psi_dot) / R;
    h_obs(7) = -theta_dot + v / R - (L * psi_dot) / R;

    // H 행렬 계산
    H(0, 0) = std::pow(psi_dot, 2) * (h * sin_theta_2 - h * cos_theta_2) - g * cos_theta - v_dot * sin_theta;
    H(1, 0) = h * psi_ddot * cos_theta - h * psi_dot * theta_dot * sin_theta * 2.0;
    H(2, 0) = v_dot * cos_theta - g * sin_theta - h * std::pow(psi_dot, 2) * sin_cos_theta * 2.0;
    H(3, 0) = -psi_dot * cos_theta;
    // H(4, 0) = 0.0;
    H(5, 0) = -psi_dot * sin_theta;
    // H(6, 0) = 0.0;
    // H(7, 0) = 0.0;

    // H(0, 1) = 0.0;
    H(1, 1) = h * psi_dot * cos_theta * 2.0;
    H(2, 1) = h * theta_dot * -2.0;
    // H(3, 1) = 0.0;
    H(4, 1) = 1.0;
    // H(5, 1) = 0.0;
    H(6, 1) = 1.0;
    H(7, 1) = -1.0;

    // H(0, 2) = 0.0;
    H(1, 2) = psi_dot;
    // H(2, 2) = 0.0;
    // H(3, 2) = 0.0;
    // H(4, 2) = 0.0;
    // H(5, 2) = 0.0;
    H(6, 2) = -1.0 / R;
    H(7, 2) = 1.0 / R;

    H(0, 3) = h * psi_dot * sin_cos_theta * -2.0;
    H(1, 3) = v + h * theta_dot * cos_theta * 2.0;
    H(2, 3) = psi_dot * (h - h * cos_theta_2) * -2.0;
    H(3, 3) = -sin_theta;
    // H(4, 3) = 0.0;
    H(5, 3) = cos_theta;
    H(6, 3) = -L / R;
    H(7, 3) = -L / R;
  }
};

#endif  // EKF_H
