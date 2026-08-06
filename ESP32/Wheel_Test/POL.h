#ifndef POL_H
#define POL_H

#include <ArduinoEigenDense.h>
#include "Params.h"

class POL {
private:
  // Calculate Helper
  float cos_theta, sin_theta, cos_theta_2, sin_theta_2, sin_cos_theta, h_2;
  float theta_dot_2, psi_dot_2;



  Eigen::Matrix3f I_B_LW;  // Left Wheel Inertia Tensor (constant)
  Eigen::Matrix3f I_B_RW;  // Right Wheel Inertia Tensor (constant)

  Eigen::Matrix3f I_B_B;   // Body Inertia Tensor (need to be calculated)
  Eigen::Vector3f p_bcom;  // CoM offset (need to be calculated)

  // Kinematic properties
  float a, b, l1, l2, l3, l4, l5, sqr_fwd;  // for kinematics
  Eigen::Vector2f theta_hips;
  float angle_EDF, AB;

  // For CoM calculator
  Eigen::Vector2f theta_As;
  Eigen::Vector2f theta_Bs;
  Eigen::Vector2f theta_ks;
  Eigen::Matrix<float, 3, 7> p_vecs, c_vecs;
  Eigen::Matrix<float, 7, 1> m_vecs;
  std::vector<Eigen::Matrix3f> R_matrices;
  std::vector<Eigen::Matrix3f> IG_matrices;
  float theta_eq;

  // Dynamic properties and Gradient
  Eigen::Matrix3f M;
  Eigen::Matrix3f M_inv;
  Eigen::Matrix3f dM_dtheta;
  Eigen::Matrix3f dnle_dqdot;
  Eigen::Vector3f nle;
  Eigen::Vector3f dnle_dtheta;
  Eigen::Matrix<float, 3, 2> B;  // input matrix
  Eigen::Matrix<float, 4, 4> fx;
  Eigen::Matrix<float, 4, 2> fu;

  Eigen::Matrix<float, 2, 1> u;        // input vector
  Eigen::Matrix<float, 4, 1> x;        // state
  float theta, theta_dot, v, psi_dot;  // state variables

public:
  Eigen::Matrix<float, 4, 1> f;
  float h, phi;  // h (m), phi(degree)
  const float g = 9.80665f;
  float m_B, m_LW, m_RW, L, R;

public:
  POL(const Properties properties_) {
    // properties update
    u.setZero();
    x.setZero();
    theta = 0;
    theta_dot = 0;
    v = 0;
    psi_dot = 0;

    fx.setZero();
    fx(0, 1) = 1;
    fu.setZero();
    a = properties_.a;
    b = properties_.b;
    l1 = properties_.l1;
    l2 = properties_.l2;
    l3 = properties_.l3;
    l4 = properties_.l4;
    l5 = properties_.l5;
    sqr_fwd = a * a + b * b + l1 * l1 + l2 * l2 - l3 * l3;
    L = properties_.L;
    R = properties_.R;

    angle_EDF = atan(l5 / l4);
    AB = sqrt(a * a + b * b);

    B << 1.0f, -1.0f,
      -1.0f / R, 1.0f / R,
      -L / R, -L / R;

    p_vecs.col(0) << 0.0f, 0.0f, 0.0f;
    p_vecs.col(1) << -0.064951905284f, -0.086f, 0.0375f;
    p_vecs.col(2) << -0.064951905284f, 0.086f, 0.0375f;
    p_vecs.col(3) << 0.0f, -0.081f, 0.0f;
    p_vecs.col(4) << 0.0f, 0.081f, 0.0f;
    R_matrices.resize(7);
    R_matrices[0] = Eigen::Matrix3f::Identity();

    c_vecs.col(0) = properties_.CoM_Body;
    c_vecs.col(1) = properties_.CoM_TAR;
    c_vecs.col(2) = properties_.CoM_TAL;
    c_vecs.col(3) = properties_.CoM_TPR;
    c_vecs.col(4) = properties_.CoM_TPL;
    c_vecs.col(5) = properties_.CoM_CR;
    c_vecs.col(6) = properties_.CoM_CL;

    m_vecs << properties_.m_Body, properties_.m_TAR, properties_.m_TAL, properties_.m_TPR, properties_.m_TPL, properties_.m_CR, properties_.m_CL;
    m_B = m_vecs.sum();
    m_RW = properties_.m_RW;
    m_LW = properties_.m_LW;

    IG_matrices.resize(7);
    IG_matrices[0] = properties_.I_Body;
    IG_matrices[1] = properties_.I_TAR;
    IG_matrices[2] = properties_.I_TAL;
    IG_matrices[3] = properties_.I_TPR;
    IG_matrices[4] = properties_.I_TPL;
    IG_matrices[5] = properties_.I_CR;
    IG_matrices[6] = properties_.I_CL;
    I_B_RW = properties_.I_RW;
    I_B_LW = properties_.I_LW;
  }

  // void initializePol(){

  // }

  void setHR(const float h_, const float phi_) {
    h = h_;
    phi = phi_;
  }

  void setState(const Eigen::Matrix<float, 4, 1>& x_) {
    x = x_;

    theta = x(0);
    theta_dot = x(1);
    v = x(2);
    psi_dot = x(3);
  }

  void setInput(const Eigen::Matrix<float, 2, 1>& u_) {
    u = u_;
  }

  void get_theta_eq(float& theta_eq) {
    theta_eq = atan(-p_bcom(0) / (h + p_bcom(2)));
  }

  Eigen::Vector2f get_theta_hips() {
    return theta_hips;
  }

  bool prepare_state_prediction() {
    prepare_calculate();  // 预先计算重复计算值以提高计算效率
    calculate_M();

    // print_all_variables();

    if (!calculate_Minv()) {
      return false;
    }
    calculate_nle();
    calculate_dM_dtheta();
    calculate_dnle_dtheta();
    calculate_dnle_dqdot();

    calculate_fx();
    return true;
  }

  void predict_state(Eigen::Matrix<float, 4, 1>& x_pred, Eigen::Matrix<float, 4, 4>& F_mat) {
    f(0) = x(1);
    f.segment<3>(1) = M_inv * (-nle + B * u);
    x_pred = x + f * dt;
    F_mat = fx;
  }

  void calculate_com_and_inertia();

  void print_all_variables();

  void solve_inverse_kinematics();

private:

  void prepare_calculate();
  void calculate_M();
  bool calculate_Minv();
  void calculate_nle();
  void calculate_dM_dtheta();
  void calculate_dnle_dtheta();
  void calculate_dnle_dqdot();
  void calculate_fx();
  void calculate_fu();
  inline Eigen::Matrix<float, 3, 3> rotation_matrix_y(float theta);
  void solve_forward_kinematics();
};

void POL::prepare_calculate() {
  cos_theta = cosf(theta);
  sin_theta = sinf(theta);
  cos_theta_2 = cos_theta * cos_theta;
  sin_theta_2 = sin_theta * sin_theta;
  sin_cos_theta = cos_theta * sin_theta;
  h_2 = h * h;
  theta_dot_2 = theta_dot * theta_dot;
  psi_dot_2 = psi_dot * psi_dot;
}

void POL::calculate_M() {
  // M(0,0)
  M(0, 0) = I_B_B(1, 1) + m_B * (h_2 + p_bcom(0) * p_bcom(0) + p_bcom(2) * p_bcom(2)) + 2.0f * h * m_B * p_bcom(2);

  // M(1,0) and M(0,1)
  M(1, 0) = m_B * (-sin_theta * p_bcom(0) + cos_theta * (h + p_bcom(2)));
  M(0, 1) = M(1, 0);

  // M(2,0) and M(0,2)
  M(2, 0) = cos_theta * (I_B_B(2, 1) - m_B * p_bcom(1) * (h + p_bcom(2)))
            - sin_theta * (I_B_B(1, 0) - m_B * p_bcom(0) * p_bcom(1));
  M(0, 2) = M(2, 0);

  // M(1,1)
  M(1, 1) = m_B + m_LW + m_RW + 1.0 / (R * R) * (I_B_LW(1, 1) + I_B_RW(1, 1));

  // M(2,1) and M(1,2)
  M(2, 1) = -m_B * p_bcom(1) - L * (m_LW - m_RW) - L * 1.0 / (R * R) * (I_B_LW(1, 1) - I_B_RW(1, 1))
            + (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1))) / R
            - (sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0))) / R;
  M(1, 2) = M(2, 1);

  // M(2,2)
  M(2, 2) = I_B_B(0, 0) + I_B_LW(0, 0) + I_B_RW(0, 0)
            + 1.0 / (R * R) * (L * L * (I_B_LW(1, 1) + I_B_RW(1, 1)))
            + L * L * (m_LW + m_RW)
            + m_B * (h_2 + p_bcom(1) * p_bcom(1) + p_bcom(2) * p_bcom(2))
            - cos_theta * ((I_B_LW(2, 1) * L * 2.0) / R - (I_B_RW(2, 1) * L * 2.0) / R)
            + sin_theta * ((I_B_LW(1, 0) * L * 2.0) / R - (I_B_RW(1, 0) * L * 2.0) / R)
            - cos_theta_2 * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2) + m_B * (h * p_bcom(2) * 2.0 + h_2 - p_bcom(0) * p_bcom(0) + p_bcom(2) * p_bcom(2)))
            - sin_cos_theta * (I_B_B(2, 0) * 2.0 + I_B_LW(2, 0) * 2.0 + I_B_RW(2, 0) * 2.0 - m_B * (p_bcom(0) * (h + p_bcom(2))) * 2.0)
            + h * m_B * p_bcom(2) * 2.0;
}

bool POL::calculate_Minv() {
  M_inv = M.inverse();

  // Check for NaN or Inf values in the result
  if (!M_inv.array().isFinite().all()) {
    Serial.println("[Error] M matrix is Singular matrix.");
    return false;
  }
  return true;
}

void POL::calculate_nle() {
  // First equation (nle(0))
  nle(0) = -psi_dot_2 * (I_B_B(2, 2) + I_B_LW(2, 2) + I_B_RW(2, 2) - m_B * (h * p_bcom(0) + p_bcom(0) * p_bcom(2) - cos_theta_2 * (h * p_bcom(0) * 2.0 + p_bcom(0) * p_bcom(2) * 2.0) - sin_cos_theta * (h * p_bcom(2) * 2.0 + h_2 - p_bcom(0) * p_bcom(0) + p_bcom(2) * p_bcom(2))) - cos_theta_2 * (I_B_B(2, 2) * 2.0 + I_B_LW(2, 2) * 2.0 + I_B_RW(2, 2) * 2.0) + sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (I_B_LW(1, 0) - I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) - I_B_RW(2, 1)))) / R)
           - g * m_B * (sin_theta * (h + p_bcom(2)) + p_bcom(0) * cos_theta)
           + (psi_dot * v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

  // Second equation (nle(1))
  nle(1) = -m_B * theta_dot_2 * (sin_theta * (h + p_bcom(2)) + p_bcom(0) * cos_theta)
           - (psi_dot * theta_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

  // Third equation (nle(2))
  nle(2) = -theta_dot_2 * (-m_B * (sin_theta * (p_bcom(1) * (h + p_bcom(2))) + p_bcom(0) * p_bcom(1) * cos_theta) + I_B_B(1, 1) * cos_theta + I_B_B(2, 2) * sin_theta)
           + psi_dot * theta_dot * (I_B_B(2, 2) * 2.0 + I_B_LW(2, 2) * 2.0 + I_B_RW(2, 2) * 2.0 - m_B * (h * p_bcom(0) * 2.0 + p_bcom(0) * p_bcom(2) * 2.0 - cos_theta_2 * (p_bcom(0) * 4.0 * (h + p_bcom(2))) - sin_cos_theta * (h * p_bcom(2) * 4.0 + h_2 * 2.0 - p_bcom(0) * p_bcom(0) * 2.0 + p_bcom(2) * p_bcom(2) * 2.0)))
           - cos_theta_2 * (I_B_B(2, 2) * 4.0 + I_B_LW(2, 2) * 4.0 + I_B_RW(2, 2) * 4.0)
           + sin_cos_theta * (I_B_B(0, 0) * 2.0 - I_B_B(2, 2) * 2.0 + I_B_LW(0, 0) * 2.0 + I_B_RW(0, 0) * 2.0 - I_B_LW(2, 2) * 2.0 - I_B_RW(2, 2) * 2.0)
           + (L * (cos_theta * (I_B_LW(1, 0) * 2.0 - I_B_RW(1, 0) * 2.0) + sin_theta * (I_B_LW(2, 1) * 2.0 - I_B_RW(2, 1) * 2.0))) / R
           - (theta_dot * v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;
}

void POL::calculate_dM_dtheta() {
  // Compute the elements of the matrix
  dM_dtheta(1, 0) = -m_B * p_bcom(0) * cos_theta - m_B * sin_theta * (h + p_bcom(2));
  dM_dtheta(2, 0) = -cos_theta * (I_B_B(1, 0) - m_B * p_bcom(0) * p_bcom(1))
                    - sin_theta * (I_B_B(2, 0) - m_B * (p_bcom(1) * (h + p_bcom(2))));

  dM_dtheta(0, 1) = dM_dtheta(1, 0);
  dM_dtheta(1, 1) = 0.0;
  dM_dtheta(2, 1) = -(cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0))) / R
                    - (sin_theta * (I_B_LW(2, 0) + I_B_RW(2, 0))) / R;

  dM_dtheta(0, 2) = dM_dtheta(2, 0);
  dM_dtheta(1, 2) = dM_dtheta(2, 1);

  dM_dtheta(2, 2) = I_B_B(2, 0) * 2.0 + I_B_LW(2, 0) * 2.0 + I_B_RW(2, 0) * 2.0
                    - cos_theta_2 * (I_B_B(2, 0) * 4.0 + I_B_LW(2, 0) * 4.0 + I_B_RW(2, 0) * 4.0 - m_B * p_bcom(0) * 4.0 * (h + p_bcom(2)))
                    - m_B * (p_bcom(0) * 2.0 * (h + p_bcom(2)))
                    + cos_theta * ((I_B_LW(1, 0) * L * 2.0) / R - (I_B_RW(1, 0) * L * 2.0) / R)
                    + sin_theta * ((I_B_LW(2, 0) * L * 2.0) / R - (I_B_RW(2, 0) * L * 2.0) / R)
                    + sin_cos_theta * (I_B_B(0, 0) * 2.0 - I_B_B(2, 0) * 2.0 + I_B_LW(0, 0) * 2.0 + I_B_RW(0, 0) * 2.0 - I_B_LW(2, 0) * 2.0 - I_B_RW(2, 0) * 2.0 + m_B * (h * p_bcom(2) * 4.0 - p_bcom(0) * p_bcom(0) * 2.0) + m_B * (h_2 + p_bcom(2) * p_bcom(2)) * 2.0);
}

void POL::calculate_dnle_dtheta() {
  dnle_dtheta(0) = -psi_dot_2 * (-I_B_B(0, 0) + I_B_B(2, 2) - I_B_LW(0, 0) - I_B_RW(0, 0) + I_B_LW(2, 2) + I_B_RW(2, 2) - m_B * (2.0 * h * p_bcom(2) - 2.0 * cos_theta_2 * (h * p_bcom(2) * 2.0 + pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2)) + pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2) + 4.0 * sin_cos_theta * p_bcom(0) * (h + p_bcom(2))) + 2.0 * cos_theta_2 * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + 4.0 * sin_cos_theta * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + (L * (cos_theta * (I_B_LW(2, 1) - I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) - I_B_RW(1, 0)))) / R)
                   - g * m_B * (cos_theta * (h + p_bcom(2)) - p_bcom(0) * sin_theta)
                   + (psi_dot * v * (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)))) / R;

  dnle_dtheta(1) = -m_B * theta_dot_2 * (cos_theta * (h + p_bcom(2)) - p_bcom(0) * sin_theta)
                   - (psi_dot * theta_dot * (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)))) / R;

  dnle_dtheta(2) = theta_dot_2 * (m_B * (cos_theta * (h * p_bcom(1) + p_bcom(1) * p_bcom(2)) - p_bcom(0) * p_bcom(1) * sin_theta) - I_B_B(2, 1) * cos_theta + I_B_B(1, 0) * sin_theta)
                   + psi_dot * theta_dot * (2.0 * (-I_B_B(0, 0) + I_B_B(2, 2) - I_B_LW(0, 0) - I_B_RW(0, 0) + I_B_LW(2, 2) + I_B_RW(2, 2)) - m_B * (4.0 * h * p_bcom(2) - 4.0 * cos_theta_2 * (h * p_bcom(2) * 2.0 + pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2)) + 2.0 * (pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2)) + 8.0 * sin_cos_theta * (h * p_bcom(0) + p_bcom(0) * p_bcom(2))) + 4.0 * cos_theta_2 * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + 8.0 * sin_cos_theta * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + (L * (2.0 * cos_theta * (I_B_LW(2, 1) - I_B_RW(2, 1)) - 2.0 * sin_theta * (I_B_LW(1, 0) - I_B_RW(1, 0)))) / R)
                   - (theta_dot * v * (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)))) / R;
}

void POL::calculate_dnle_dqdot() {
  dnle_dqdot(0, 0) = 0.0;

  dnle_dqdot(1, 0) = -psi_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1))) / R
                     - 2.0f * m_B * theta_dot * (sin_theta * (h + p_bcom(2)) + p_bcom(0) * cos_theta);

  dnle_dqdot(2, 0) = psi_dot * (2.0f * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) - m_B * (2.0f * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - 4.0f * cos_theta_2 * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - sin_cos_theta * (4.0f * h * p_bcom(2) + 2.0f * h_2 - 2.0f * p_bcom(0) * p_bcom(0) + 2.0f * p_bcom(2) * p_bcom(2))) - 4.0f * cos_theta_2 * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + 2.0f * sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (2.0f * I_B_LW(1, 0) - 2.0f * I_B_RW(1, 0)) + sin_theta * (2.0f * I_B_LW(2, 1) - 2.0f * I_B_RW(2, 1)))) / R)
                     - theta_dot * (-m_B * (2.0f * sin_theta * (h * p_bcom(1) + p_bcom(1) * p_bcom(2)) + 2.0f * p_bcom(0) * p_bcom(1) * cos_theta) + 2.0f * I_B_B(1, 0) * cos_theta + 2.0f * I_B_B(2, 1) * sin_theta)
                     - (v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

  dnle_dqdot(0, 1) = (psi_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

  dnle_dqdot(1, 1) = 0.0;

  dnle_dqdot(2, 1) = -(theta_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

  dnle_dqdot(0, 2) = -psi_dot * (2.0f * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) - m_B * (2.0f * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - cos_theta_2 * (4.0f * h * p_bcom(0) + 4.0f * p_bcom(0) * p_bcom(2)) - sin_cos_theta * (4.0f * h * p_bcom(2) + 2.0f * h_2 - 2.0f * p_bcom(0) * p_bcom(0) + 2.0f * p_bcom(2) * p_bcom(2))) - 4.0f * cos_theta_2 * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + 2.0f * sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (2.0f * I_B_LW(1, 0) - 2.0f * I_B_RW(1, 0)) + sin_theta * (2.0f * I_B_LW(2, 1) - 2.0f * I_B_RW(2, 1)))) / R)
                     + (v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

  dnle_dqdot(1, 2) = -(theta_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

  dnle_dqdot(2, 2) = theta_dot * (2.0f * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) - m_B * (2.0f * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - cos_theta_2 * (4.0f * h * p_bcom(0) + 4.0f * p_bcom(0) * p_bcom(2)) - sin_cos_theta * (4.0f * h * p_bcom(2) + 2.0f * h_2 - 2.0f * p_bcom(0) * p_bcom(0) + 2.0f * p_bcom(2) * p_bcom(2))) - 4.0f * cos_theta_2 * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + 2.0f * sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (2.0f * I_B_LW(1, 0) - 2.0f * I_B_RW(1, 0)) + sin_theta * (2.0f * I_B_LW(2, 1) - 2.0f * I_B_RW(2, 1)))) / R);
}

void POL::calculate_fx() {
  fx.block<3, 1>(1, 0) = -M_inv * (dM_dtheta * M_inv * (-nle + B * u) + (dnle_dtheta));
  // fx.block<3, 1>(1, 0) = -M_inv * (dM_dtheta * M_inv * (-nle) + (dnle_dtheta));

  fx.block<3, 3>(1, 1) = -M_inv * dnle_dqdot;
}

void POL::calculate_fu() {
  fu.block<3, 2>(1, 0) = M_inv * B;
}

inline Eigen::Matrix<float, 3, 3> POL::rotation_matrix_y(float theta) {
  float c = cosf(theta), s = sinf(theta);
  Eigen::Matrix<float, 3, 3> R;
  R << c, 0, s, 0, 1, 0, -s, 0, c;
  return R;
}

void POL::solve_forward_kinematics() {
  theta_Bs(0) = -theta_hips(0);
  theta_Bs(1) = theta_hips(1);

  float thB0 = theta_Bs(0), cB0 = cosf(thB0), sB0 = sinf(thB0);
  float thB1 = theta_Bs(1), cB1 = cosf(thB1), sB1 = sinf(thB1);

  float ax0 = a + l2 * cB0, ay0 = b + l2 * sB0;
  float ax1 = a + l2 * cB1, ay1 = b + l2 * sB1;

  float dist0 = sqrtf(ax0 * ax0 + ay0 * ay0);
  float dist1 = sqrtf(ax1 * ax1 + ay1 * ay1);

  float num0 = sqr_fwd + 2 * l2 * (a * cB0 + b * sB0);
  float num1 = sqr_fwd + 2 * l2 * (a * cB1 + b * sB1);

  float r0 = (num0 / (2 * l1)) / dist0;
  float r1 = (num1 / (2 * l1)) / dist1;

  if (fabsf(r0) > 1.0f || fabsf(r1) > 1.0f) {
    throw std::runtime_error("Ratio out of range");
  }

  float alpha0 = atan2f(b + l2 * sB0, a + l2 * cB0);
  float alpha1 = atan2f(b + l2 * sB1, a + l2 * cB1);

  theta_As(0) = -acosf(r0) + alpha0;
  theta_As(1) = -acosf(r1) + alpha1;

  float thA0 = theta_As(0), cA0 = cosf(thA0), sA0 = sinf(thA0);
  float thA1 = theta_As(1), cA1 = cosf(thA1), sA1 = sinf(thA1);

  float nx0 = (a + l2 * cB0 - l1 * cA0), ny0 = (b + l2 * sB0 - l1 * sA0);
  float nx1 = (a + l2 * cB1 - l1 * cA1), ny1 = (b + l2 * sB1 - l1 * sA1);

  theta_ks(0) = atan2f(ny0, nx0);
  theta_ks(1) = atan2f(ny1, nx1);

  // 前面的都是常量，所以在构造函数中初始化它
  p_vecs.col(5) << -a - l2 * cB0, -0.102f, b + l2 * sB0;
  p_vecs.col(6) << -a - l2 * cB1, 0.102f, b + l2 * sB1;

  R_matrices[1] = rotation_matrix_y(theta_Bs(0));
  R_matrices[2] = rotation_matrix_y(theta_Bs(1));
  R_matrices[3] = rotation_matrix_y(theta_As(0));
  R_matrices[4] = rotation_matrix_y(theta_As(1));
  R_matrices[5] = rotation_matrix_y(theta_ks(0));
  R_matrices[6] = rotation_matrix_y(theta_ks(1));
}

void POL::solve_inverse_kinematics() {

  // degree to rad
  phi = phi * M_PI / 180;

  // h_饱和度设置
  float phi_max = std::min(atanf((h - HEIGHT_MIN) / L), atanf((HEIGHT_MAX - h) / L));
  float phi_min = -phi_max;

  phi = constrain(phi, phi_min, phi_max);  // 限制 phi 值

  float hR = h - L * tanf(phi);
  float hL = h + L * tanf(phi);
  float hs[2] = { hR, hL };


  for (int i = 0; i < 2; i++) {
    float h_val = hs[i];

    // 角度计算

    float angle_ADE = acos((pow(l1, 2) + pow(l4, 2) + pow(l5, 2) - pow(h_val, 2)) / (2 * l1 * sqrt(pow(l4, 2) + pow(l5, 2))));
    float angle_ADC = M_PI - (angle_ADE + angle_EDF);
    float AC = sqrt(pow(l1, 2) + pow(l3, 2) - 2 * l1 * l3 * cos(angle_ADC));
    float S = (pow(AB, 2) + pow(l2, 2) - pow(AC, 2)) / (2 * AB * l2);

    if (abs(S) > 1 && abs(AC - sqrt(pow(a, 2) + pow(b, 2)) - l2) < 0.1) {
      AC = sqrt(pow(a, 2) + pow(b, 2)) + l2;
      S = 1;
    }

    float angle_ABC = acos((pow(AB, 2) + pow(l2, 2) - pow(AC, 2)) / (2 * AB * l2));
    theta_hips[i] = (5 * M_PI / 6) - angle_ABC;
  }

  theta_hips[1] = -theta_hips[1];  // BL角反转
}

void POL::calculate_com_and_inertia() {
  // Serial.print("h:"); Serial.println(h,5);

  solve_inverse_kinematics();
  // Serial.print("theta_hips:"); Serial.print(theta_hips(0),5); Serial.print(", ") ;Serial.println(theta_hips(1),5);
  solve_forward_kinematics();

  Eigen::Vector3f rB[7];
  for (int i = 0; i < 7; i++) {
    rB[i] = p_vecs.col(i) + R_matrices.at(i) * c_vecs.col(i);
    p_bcom += m_vecs(i) * rB[i];
  }
  p_bcom /= m_B;

  for (int i = 0; i < 7; i++) {
    Eigen::Vector3f rC = rB[i] - p_bcom;
    float nn = rC.squaredNorm();
    I_B_B += R_matrices.at(i) * IG_matrices.at(i) * R_matrices.at(i).transpose() + m_vecs(i) * (nn * Eigen::Matrix3f::Identity() - rC * rC.transpose());
  }

  // Serial.print("p_bcom:"); Serial.print(p_bcom(0),5); Serial.print(", "); Serial.print(p_bcom(1),5); Serial.print(", "); Serial.println(p_bcom(2),5);

  // Output I_B_B
  // Serial.println("I_B_B:");
  // for (int row = 0; row < 3; row++) {
  //   for (int col = 0; col < 3; col++) {
  //     Serial.print(I_B_B(row, col),5);
  //     if (col < 2) Serial.print(", "); // Add comma for separation
  //   }
  //   Serial.println(); // Newline after each row
  // }
}

void POL::print_all_variables() {

  //状态变量输出
  Serial.println("State Variables:");
  Serial.print("theta: ");
  Serial.println(theta, 5);
  Serial.print("theta_dot: ");
  Serial.println(theta_dot, 5);
  Serial.print("v: ");
  Serial.println(v, 5);
  Serial.print("psi_dot: ");
  Serial.println(psi_dot, 5);

  // 辅助变量输出
  Serial.println("\nHelper Variables:");
  Serial.print("cos_theta: ");
  Serial.println(cos_theta, 5);
  Serial.print("sin_theta: ");
  Serial.println(sin_theta, 5);
  Serial.print("cos_theta_2: ");
  Serial.println(cos_theta_2, 5);
  Serial.print("sin_theta_2: ");
  Serial.println(sin_theta_2, 5);
  Serial.print("sin_cos_theta: ");
  Serial.println(sin_cos_theta, 5);
  Serial.print("h_2: ");
  Serial.println(h_2, 5);
  Serial.print("theta_dot_2: ");
  Serial.println(theta_dot_2, 5);
  Serial.print("psi_dot_2: ");
  Serial.println(psi_dot_2, 5);

  // 惯性张量输出
  Serial.println("\nInertia Tensors:");
  Serial.print("I_B_LW:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(I_B_LW(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("I_B_RW:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(I_B_RW(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("I_B_B:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(I_B_B(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  // CoM 偏移输出
  Serial.print("p_bcom: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(p_bcom(i), 5);
    Serial.print(" ");
  }
  Serial.println();

  // 运动学属性输出
  Serial.println("\nKinematic Properties:");
  Serial.print("a: ");
  Serial.println(a, 5);
  Serial.print("b: ");
  Serial.println(b, 5);
  Serial.print("l1: ");
  Serial.println(l1, 5);
  Serial.print("l2: ");
  Serial.println(l2, 5);
  Serial.print("l3: ");
  Serial.println(l3, 5);
  Serial.print("l4: ");
  Serial.println(l4, 5);
  Serial.print("l5: ");
  Serial.println(l5, 5);
  Serial.print("sqr_fwd: ");
  Serial.println(sqr_fwd, 5);

  Serial.print("theta_hips: ");
  for (int i = 0; i < 2; i++) {
    Serial.print(theta_hips(i), 5);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("angle_EDF: ");
  Serial.println(angle_EDF, 5);
  Serial.print("AB: ");
  Serial.println(AB, 5);

  // 对于 CoM 计算器输出
  Serial.println("\nFor CoM Calculator:");
  Serial.print("theta_As: ");
  for (int i = 0; i < 2; i++) {
    Serial.print(theta_As(i), 5);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("theta_Bs: ");
  for (int i = 0; i < 2; i++) {
    Serial.print(theta_Bs(i), 5);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("theta_ks: ");
  for (int i = 0; i < 2; i++) {
    Serial.print(theta_ks(i), 5);
    Serial.print(" ");
  }
  Serial.println();

  // p_vecs、c_vecs、m_vecs 输出
  Serial.print("p_vecs:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 7; j++) {
      Serial.print(p_vecs(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("c_vecs:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 7; j++) {
      Serial.print(c_vecs(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("m_vecs: ");
  for (int i = 0; i < 7; i++) {
    Serial.print(m_vecs(i), 5);
    Serial.print(" ");
  }
  Serial.println();

  // 惯性和梯度矩阵输出
  Serial.println("\nInertia and Gradient Matrices:");
  Serial.print("R_matrices: ");
  for (size_t i = 0; i < R_matrices.size(); i++) {
    Serial.print("R_matrix ");
    Serial.print(i);
    Serial.println(":");
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        Serial.print(R_matrices[i](j, k), 5);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  Serial.print("IG_matrices: ");
  for (size_t i = 0; i < IG_matrices.size(); i++) {
    Serial.print("IG_matrix ");
    Serial.print(i);
    Serial.println(":");
    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        Serial.print(IG_matrices[i](j, k), 5);
        Serial.print(" ");
      }
      Serial.println();
    }
  }

  // CoM 计算和其他输出
  Serial.print("theta_eq: ");
  Serial.println(theta_eq, 5);

  // 动态属性和梯度输出
  Serial.println("\nDynamic Properties and Gradient:");
  Serial.print("M:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(M(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("M_inv:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(M_inv(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("dM_dtheta:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print(dM_dtheta(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("dnle_dqdot:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      Serial.print(dnle_dqdot(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("nle: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(nle(i), 5);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("dnle_dtheta: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(dnle_dtheta(i), 5);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("B:\n");
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      Serial.print(B(i, j), 5);
      Serial.print(" ");
    }
    Serial.println();
  }

  // fx、fu 输出
  Serial.print("fx:\n");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      Serial.print(fx(i, j));
      Serial.print(" ");
    }
    Serial.println();
  }

  Serial.print("fu:\n");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 2; j++) {
      Serial.print(fu(i, j));
      Serial.print(" ");
    }
    Serial.println();
  }

  // u、x 输出
  Serial.print("u: ");
  for (int i = 0; i < 2; i++) {
    Serial.print(u(i));
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("x: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(x(i));
    Serial.print(" ");
  }
  Serial.println();

  // 物性输出
  Serial.println("\nPhysical Properties:");
  Serial.print("h: ");
  Serial.println(h);
  Serial.print("phi: ");
  Serial.println(phi);
  Serial.print("m_B: ");
  Serial.println(m_B);
  Serial.print("m_LW: ");
  Serial.println(m_LW);
  Serial.print("m_RW: ");
  Serial.println(m_RW);
  Serial.print("L: ");
  Serial.println(L);
  Serial.print("R: ");
  Serial.println(R);
}



#endif  // POL_H_H
