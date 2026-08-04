#ifndef MPU6050_STANDALONE_TEST
#define MPU6050_STANDALONE_TEST 0
#endif
#ifndef WHEEL_MOTOR_STANDALONE_TEST
#define WHEEL_MOTOR_STANDALONE_TEST 0
#endif
#ifndef WHEEL_MOTOR_SOURCE_HARDWARE_TEST
#define WHEEL_MOTOR_SOURCE_HARDWARE_TEST 0
#endif
#ifndef HIP_SERVO_STANDALONE_TEST
#define HIP_SERVO_STANDALONE_TEST 0
#endif
#ifndef HIP_SERVO_MPU6050_COMBINED_TEST
#define HIP_SERVO_MPU6050_COMBINED_TEST 1
#endif

#if MPU6050_STANDALONE_TEST
#include "testmpu.h"
#elif WHEEL_MOTOR_SOURCE_HARDWARE_TEST
#include "testMG0.h"
#elif HIP_SERVO_MPU6050_COMBINED_TEST
#include "test.h"
#elif HIP_SERVO_STANDALONE_TEST
#include "testHR.h"
#elif WHEEL_MOTOR_STANDALONE_TEST
#include "testMG.h"
#else

#include "Params.h"
#include "Receiver.h"
#include "HRController.h"
#include "VYBController.h"
#include "MGServo.h"
#include "IMU.h"
#include "POL.h"
#include "EKF.h"
#include "CompenFilter.h"
#include "Logger.h"
#include "Timer.h"


// Properties와 Receiver, Controller 초기화
const Properties properties = createDefaultProperties();
POL Pol(properties);
HardwareSerial RS485(1);
MGServo ServoLW(1, RS485);
MGServo ServoRW(2, RS485);
Receiver receiver(Serial2);
IMU MPU6050;

HRController HR_controller;
VYBController VYB_controller(ServoRW, ServoLW);
EKF Estimator(Pol);
// CompenFilter Estimator(properties);

Logger WIFI_Logger(ssid, password);

Eigen::Matrix<float, 4, 1> x = Eigen::Matrix<float, 4, 1>::Zero();
Eigen::Matrix<float, 4, 1> x_d = Eigen::Matrix<float, 4, 1>::Zero();
Eigen::Matrix<float, 2, 1> u_prev = Eigen::Matrix<float, 2, 1>::Zero();
Eigen::Matrix<float, 2, 1> u = Eigen::Matrix<float, 2, 1>::Zero();
Eigen::Matrix<float, 8, 1> z = Eigen::Matrix<float, 8, 1>::Zero();
Eigen::Matrix<float, 2, 1> iq_vec = Eigen::Matrix<float, 2, 1>::Zero();

int i = 0;
Timer log_timer(Timer::TimerType::Millis);
Timer sampling_timer(Timer::TimerType::Millis);
Timer temp_timer(Timer::TimerType::Millis);
float h_d = HEIGHT_MAX, phi_d = 0;
float v_d = 0, dpsi_d = 0;

void serialPrintStates();

// ==============================================================================
//                                    SETUP
// ==============================================================================
void setup() {
  // Serial 통신, Receiver, HR Controller 초기화
  Serial.begin(115200);               // Serial 통신 시작
  receiver.begin();                            // Receiver 초기화
  HR_controller.attachServos(LH_PIN, RH_PIN);  // Servo Pin 설정

  // IMU (MPU6050) 초기화
  if (!MPU6050.begin()) {
    Serial.println("[ERROR] Fail to initialize IMU.");
  }

  // RS485 초기화
  pinMode(RS485_DE_RE, OUTPUT);                                         // RS485 방향 제어 핀 설정
  digitalWrite(RS485_DE_RE, LOW);                                       // RS485 수신 모드 설정
  RS485.begin(460800, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);  // RS485 통신 시작

  // WIFI 연결
  WIFI_Logger.begin();

  // PSRAM 상태 확인 및 초기화
  if (psramFound()) {
    Serial.println("PSRAM available.");
    if (!psramInit()) {
      Serial.println("PSRAM initialization failed!");
      while (1) {}  // 초기화 실패 시 무한 루프
    } else {
      Serial.println("PSRAM initialized successfully!");
    }
    Serial.printf("PSRAM size: %d bytes\n", ESP.getPsramSize());  // PSRAM 크기 출력
  } else {
    Serial.println("PSRAM not available.");
    while (1) {}  // PSRAM 미탐지 시 무한 루프
  }

  // SBUS 데이터 수신 대기 (타임아웃 처리)
  Timer receiver_timer(Timer::TimerType::Millis);
  receiver_timer.start();
  const unsigned long timeout = 5000;  // 타임아웃 5초 설정

  while (!receiver.readData()) {
    if (receiver_timer.getDuration() > timeout) {
      Serial.println("Timeout: No data received from SBUS.");
      receiver_timer.start();  // 타임아웃 초기화
    }
  }
  receiver.updateData();  // 데이터 업데이트

  // Logger pre-allocation ==============================================
  WIFI_Logger.readyToLogValue("cal_time");

  WIFI_Logger.readyToLogTimeStamp();  // 시간 기록
  // Desired states
  WIFI_Logger.readyToLogValue("h_d"); 
  WIFI_Logger.readyToLogValue("theta_d");
  WIFI_Logger.readyToLogValue("v_d");
  WIFI_Logger.readyToLogValue("psi_dot_d");
  
  // Estimated states
  WIFI_Logger.readyToLogValue("theta_hat");
  WIFI_Logger.readyToLogValue("theta_dot_hat");  // 시간 기록
  WIFI_Logger.readyToLogValue("v_hat");          // 시간 기록
  WIFI_Logger.readyToLogValue("psi_dot_hat");    // 시간 기록

  // Control inputs
  WIFI_Logger.readyToLogValue("tau_RW");  // 시간 기록
  WIFI_Logger.readyToLogValue("tau_LW");  // 시간 기록

  // Measurements
  WIFI_Logger.readyToLogValue("acc_x");
  WIFI_Logger.readyToLogValue("acc_y");  // 시간 기록
  WIFI_Logger.readyToLogValue("acc_z");  // 시간 기록
  WIFI_Logger.readyToLogValue("gyr_x");  // 시간 기록
  WIFI_Logger.readyToLogValue("gyr_y");
  WIFI_Logger.readyToLogValue("gyr_z");         // 시간 기록
  WIFI_Logger.readyToLogValue("theta_dot_RW");  // 시간 기록
  WIFI_Logger.readyToLogValue("theta_dot_LW");  // 시간 기록

  WIFI_Logger.readyToLogValue("current_RW");
  WIFI_Logger.readyToLogValue("current_LW");
  // =======================================================================

  // 시간 측정 시작
  log_timer.start();
  sampling_timer.start();
}

// ==============================================================================
//                                    LOOP
// ==============================================================================
void loop() {
  // sampling time이 경과했을 때만 실행
  if (sampling_timer.getDuration() >= dt * 1000) {
    // 경과 시간 출력
    Serial.print("SamplingTime(ms):");
    Serial.print(sampling_timer.getDuration());
    Serial.print(" ");

    sampling_timer.start();  // sampling timer 초기화


    if (receiver.readData()) {
      receiver.updateData();
    }

    if (receiver.isRun()) {
      // Running Mode
      //// update Pol state and input for EKF ////
      Pol.setState(x);
      // Pol.setInput(u);
      Pol.setInput(u_prev);  // u_k-2

      // measurement update
      MPU6050.readData();  // read k-th IMU measurements
      MPU6050.getIMUMeasurement(z);

      VYB_controller.sendReadStateCommand();  // read k-th motor speed
      VYB_controller.getMotorSpeedMeasurement(z);
      VYB_controller.getMotorCurrentMeasurement(iq_vec);

      //// state estimation ////
      if (!Estimator.estimate_state(x, z)) {
        // Diverge Safe Gaurd
        ServoLW.sendTorqueControlCommand(0);
        ServoRW.sendTorqueControlCommand(0);
        while (true) {
          Serial.println("Mass matrix Singularity Error. Change mode to Off Mode...");
          x.setZero();
          u.setZero();
          Estimator.reset_estimator();
          if (receiver.readData()) {
            receiver.updateData();
          }
          delay(500);
          if (!receiver.isRun()) {
            return;
          }
        }
      }

      //// Get Desired States from receiver ////
      receiver.updateDesiredStates();
      h_d = receiver.getDesiredHeight();
      // phi_d = receiver.getDesiredRoll();
      phi_d = 0;  // roll control disable
      v_d = receiver.getDesiredVel();
      dpsi_d = receiver.getDesiredYawVel();

      x_d.segment<2>(2) << v_d, dpsi_d;

      //// calculate CoM and Inertia from CoM Calculator ////
      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();  // 여기서 inverse kinematics로 theta_hips도 계산됨
      Pol.get_theta_eq(x_d(0));         // update desired pitch angle with equilibrium point

      //// compute VYB controller gain ////
      VYB_controller.computeGainK(h_d);
      VYB_controller.computeInput(x_d, x);

      //// Send control command of HR controller and VYB controller
      HR_controller.controlHipServos(Pol.get_theta_hips());
      VYB_controller.sendControlCommand();
      u_prev = u;
      u = VYB_controller.getInputVector();

      //============= Logging ======================================================================
      // Logging calculating time and timestamp
      WIFI_Logger.logValue("cal_time", sampling_timer.getDuration());  // Log the calculating time
      WIFI_Logger.logTimeStamp(log_timer.getDuration());               // Log the current timestamp

      // Desired states (reference values for control)
      WIFI_Logger.logValue("h_d", h_d);           // Desired height
      WIFI_Logger.logValue("theta_d", x_d(0));    // Desired pitch angle (theta)
      WIFI_Logger.logValue("v_d", x_d(2));        // Desired velocity
      WIFI_Logger.logValue("psi_dot_d", x_d(3));  // Desired yaw rate (psi_dot)
      

      // Estimated states (current system state estimates)
      WIFI_Logger.logValue("theta_hat", x(0));      // Estimated pitch angle (theta)
      WIFI_Logger.logValue("theta_dot_hat", x(1));  // Estimated pitch rate (theta_dot)
      WIFI_Logger.logValue("v_hat", x(2));          // Estimated velocity
      WIFI_Logger.logValue("psi_dot_hat", x(3));    // Estimated yaw rate (psi_dot)

      // Control inputs (commands to the system)
      WIFI_Logger.logValue("tau_RW", u(0));  // Control torque for the right wheel (RW)
      WIFI_Logger.logValue("tau_LW", u(1));  // Control torque for the left wheel (LW)

      // Measurements (sensor readings)
      WIFI_Logger.logValue("acc_x", z(0));         // Acceleration in x-direction
      WIFI_Logger.logValue("acc_y", z(1));         // Acceleration in y-direction
      WIFI_Logger.logValue("acc_z", z(2));         // Acceleration in z-direction
      WIFI_Logger.logValue("gyr_x", z(3));         // Gyroscope reading in x-direction
      WIFI_Logger.logValue("gyr_y", z(4));         // Gyroscope reading in y-direction
      WIFI_Logger.logValue("gyr_z", z(5));         // Gyroscope reading in z-direction
      WIFI_Logger.logValue("theta_dot_RW", z(6));  // Angular velocity of the right wheel
      WIFI_Logger.logValue("theta_dot_LW", z(7));  // Angular velocity of the left wheel

      // Current measurements for the wheels
      WIFI_Logger.logValue("current_RW", iq_vec(0));  // Current for the right wheel
      WIFI_Logger.logValue("current_LW", iq_vec(1));  // Current for the left wheel
      //=============================================================================================
    } else if (receiver.isReset()) {
      // Estimator Reset
      x_d.setZero();
      VYB_controller.theta_d = 0.f;
      x.setZero();
      u.setZero();
      Estimator.reset_estimator();
      WIFI_Logger.resetLogData();
      log_timer.start();

    } else {
      // Off Mode
      ServoLW.sendTorqueControlCommand(0);
      ServoRW.sendTorqueControlCommand(0);
      u.setZero();

      WIFI_Logger.handleClientRequests();  // Log Data 전송

      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();
      Pol.get_theta_eq(x_d(0));

      Pol.setState(x);
      Pol.setInput(u);

      // measurement update
      MPU6050.readData();
      MPU6050.getIMUMeasurement(z);
      VYB_controller.getMotorSpeedMeasurement(z);
      // state estimation
      if (!Estimator.estimate_state(x, z)) {
        // Diverge Safe Gaurd
        ServoLW.sendTorqueControlCommand(0);
        ServoRW.sendTorqueControlCommand(0);
        while (true) {
          Serial.println("Mass matrix Singularity Error. Change mode to Off Mode...");
          x.setZero();
          u.setZero();
          Estimator.reset_estimator();
          if (receiver.readData()) {
            receiver.updateData();
          }
          delay(500);
          if (!receiver.isRun()) {
            return;
          }
        }
      }

      // //============= Logging ======================================================================
      // // Logging calculating time and timestamp
      // WIFI_Logger.logValue("cal_time", sampling_timer.getDuration());  // Log the calculating time
      // WIFI_Logger.logTimeStamp(log_timer.getDuration());               // Log the current timestamp

      // // Desired states (reference values for control)
      // WIFI_Logger.logValue("h_d", h_d);           // Desired height
      // WIFI_Logger.logValue("theta_d", x_d(0));    // Desired pitch angle (theta)
      // WIFI_Logger.logValue("v_d", x_d(2));        // Desired velocity
      // WIFI_Logger.logValue("psi_dot_d", x_d(3));  // Desired yaw rate (psi_dot)

      // // Estimated states (current system state estimates)
      // WIFI_Logger.logValue("theta_hat", x(0));      // Estimated pitch angle (theta)
      // WIFI_Logger.logValue("theta_dot_hat", x(1));  // Estimated pitch rate (theta_dot)
      // WIFI_Logger.logValue("v_hat", x(2));          // Estimated velocity
      // WIFI_Logger.logValue("psi_dot_hat", x(3));    // Estimated yaw rate (psi_dot)

      // // Control inputs (commands to the system)
      // WIFI_Logger.logValue("tau_RW", u(0));  // Control torque for the right wheel (RW)
      // WIFI_Logger.logValue("tau_LW", u(1));  // Control torque for the left wheel (LW)

      // // Measurements (sensor readings)
      // WIFI_Logger.logValue("acc_x", z(0));         // Acceleration in x-direction
      // WIFI_Logger.logValue("acc_y", z(1));         // Acceleration in y-direction
      // WIFI_Logger.logValue("acc_z", z(2));         // Acceleration in z-direction
      // WIFI_Logger.logValue("gyr_x", z(3));         // Gyroscope reading in x-direction
      // WIFI_Logger.logValue("gyr_y", z(4));         // Gyroscope reading in y-direction
      // WIFI_Logger.logValue("gyr_z", z(5));         // Gyroscope reading in z-direction
      // WIFI_Logger.logValue("theta_dot_RW", z(6));  // Angular velocity of the right wheel
      // WIFI_Logger.logValue("theta_dot_LW", z(7));  // Angular velocity of the left wheel

      // // Current measurements for the wheels
      // WIFI_Logger.logValue("current_RW", iq_vec(0));  // Current for the right wheel
      // WIFI_Logger.logValue("current_LW", iq_vec(1));  // Current for the left wheel
      // //=============================================================================================

      serialPrintStates();
      // MPU6050.printData();
      Serial.println(" Off Mode");
    }
  }
}

void serialPrintStates() {
  Serial.print("theta:");
  Serial.print(x(0) * 180 / M_PI, 6);
  Serial.print(" ");
  Serial.print("theta_dot:");
  Serial.print(x(1) * 180 / M_PI, 6);
  Serial.print(" ");
  Serial.print("v:");
  Serial.print(x(2), 6);
  Serial.print(" ");
  Serial.print("psi_dot:");
  Serial.print(x(3) * 180 / M_PI, 6);
  Serial.print(" ");
  Serial.print("u_RW:");
  Serial.print(u(0), 6);
  Serial.print(" ");
  Serial.print("u_LW:");
  Serial.print(u(1), 6);
  Serial.print(" ");
}

#endif  // standalone test selection
