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
#define HIP_SERVO_MPU6050_COMBINED_TEST 0
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
HardwareSerial motor1Serial(1);
HardwareSerial motor2Serial(2);
MGServo ServoLW(1, motor1Serial, true);
MGServo ServoRW(2, motor2Serial, true);
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
bool serial_run_enabled = false;
bool compute_only_enabled = false;
bool serial_reset_requested = false;
bool diagnostic_output_enabled = false;
bool imu_read_ok = false;
bool motor_rw_read_ok = false;
bool motor_lw_read_ok = false;
uint32_t last_diagnostic_ms = 0;
String serial_command;

void serialPrintStates();
void printDiagnosticSnapshot(bool refresh_measurements);

void printSerialControlHelp() {
  Serial0.println("Commands: run, stop, reset, h <0.07..0.20>, v <-1..1>, yaw <-1..1>");
  Serial0.println("          status, diag, diag on/off, control on/off, help");
}

void handleSerialControlCommand(String command) {
  command.trim();
  if (command.isEmpty()) return;
  if (command.equalsIgnoreCase("run")) {
    compute_only_enabled = false;
    serial_run_enabled = true;
    Serial0.println("[CONTROL] RUN enabled");
  } else if (command.equalsIgnoreCase("stop")) {
    serial_run_enabled = false;
    compute_only_enabled = false;
    v_d = 0;
    dpsi_d = 0;
    ServoLW.sendTorqueControlCommand(0);
    ServoRW.sendTorqueControlCommand(0);
    Serial0.println("[CONTROL] STOP: wheel torque is zero");
  } else if (command.equalsIgnoreCase("reset")) {
    serial_reset_requested = true;
  } else if (command.equalsIgnoreCase("control on")) {
    serial_run_enabled = false;
    compute_only_enabled = true;
    ServoLW.sendTorqueControlCommand(0);
    ServoRW.sendTorqueControlCommand(0);
    Serial0.println("[CONTROL] COMPUTE ONLY: actuator output disabled");
  } else if (command.equalsIgnoreCase("control off")) {
    compute_only_enabled = false;
    Serial0.println("[CONTROL] compute-only mode disabled");
  } else if (command.equalsIgnoreCase("diag")) {
    printDiagnosticSnapshot(true);
  } else if (command.equalsIgnoreCase("diag on")) {
    diagnostic_output_enabled = true;
    Serial0.println("[DIAG] continuous output ON (1 Hz)");
  } else if (command.equalsIgnoreCase("diag off")) {
    diagnostic_output_enabled = false;
    Serial0.println("[DIAG] continuous output OFF");
  } else if (command.equalsIgnoreCase("status")) {
    Serial0.printf("run=%s compute_only=%s h=%.3f m v=%.3f m/s yaw=%.3f rad/s\n",
                   serial_run_enabled ? "ON" : "OFF",
                   compute_only_enabled ? "ON" : "OFF", h_d, v_d, dpsi_d);
  } else if (command.equalsIgnoreCase("help") || command == "?") {
    printSerialControlHelp();
  } else {
    float value = 0.0f;
    if (sscanf(command.c_str(), "h %f", &value) == 1) {
      h_d = constrain(value, HEIGHT_MIN, HEIGHT_MAX);
      Serial0.printf("[CONTROL] height=%.3f m\n", h_d);
    } else if (sscanf(command.c_str(), "v %f", &value) == 1) {
      v_d = constrain(value, -VEL_MAX, VEL_MAX);
      Serial0.printf("[CONTROL] velocity=%.3f m/s\n", v_d);
    } else if (sscanf(command.c_str(), "yaw %f", &value) == 1) {
      dpsi_d = constrain(value, -YAW_MAX, YAW_MAX);
      Serial0.printf("[CONTROL] yaw rate=%.3f rad/s\n", dpsi_d);
    } else {
      Serial0.println("[ERROR] Unknown command; enter help to list commands");
    }
  }
}

void updateSerialControl() {
  while (Serial0.available()) {
    const char ch = static_cast<char>(Serial0.read());
    if (ch == '\r' || ch == '\n') {
      handleSerialControlCommand(serial_command);
      serial_command = "";
    } else if (ch == '\b' || ch == 0x7F) {
      if (!serial_command.isEmpty()) serial_command.remove(serial_command.length() - 1);
    } else if (isPrintable(ch)) {
      serial_command += ch;
    }
  }
}

void printDiagnosticSnapshot(bool refresh_measurements) {
  if (refresh_measurements) {
    imu_read_ok = MPU6050.readData();
    if (imu_read_ok) MPU6050.getIMUMeasurement(z);
    motor_rw_read_ok = ServoRW.sendCommandReadMotorState2();
    motor_lw_read_ok = ServoLW.sendCommandReadMotorState2();
    VYB_controller.getMotorSpeedMeasurement(z);
    VYB_controller.getMotorCurrentMeasurement(iq_vec);
  }

  const float acc_norm = z.segment<3>(0).norm();
  const bool rw_saturated = fabsf(u(0)) >= MAX_TORQUE - 0.001f;
  const bool lw_saturated = fabsf(u(1)) >= MAX_TORQUE - 0.001f;
  Serial0.printf("DIAG io imu=%s rw=%s lw=%s\n",
                 imu_read_ok ? "OK" : "FAIL",
                 motor_rw_read_ok ? "OK" : "FAIL",
                 motor_lw_read_ok ? "OK" : "FAIL");
  Serial0.printf("DIAG acc[m/s2] x=%+.3f y=%+.3f z=%+.3f norm=%.3f\n",
                 z(0), z(1), z(2), acc_norm);
  Serial0.printf("DIAG gyro[rad/s] x=%+.4f y=%+.4f z=%+.4f\n",
                 z(3), z(4), z(5));
  Serial0.printf("DIAG wheel[rad/s] rw=%+.3f lw=%+.3f current[A] rw=%+.3f lw=%+.3f\n",
                 z(6), z(7), iq_vec(0), iq_vec(1));
  Serial0.printf("DIAG ekf theta=%+.4f theta_dot=%+.4f v=%+.3f yaw=%+.3f\n",
                 x(0), x(1), x(2), x(3));
  Serial0.printf("DIAG target theta=%+.4f v=%+.3f yaw=%+.3f\n",
                 x_d(0), x_d(2), x_d(3));
  Serial0.printf("DIAG torque[Nm] rw=%+.4f%s lw=%+.4f%s mode=%s\n",
                 u(0), rw_saturated ? " SAT" : "",
                 u(1), lw_saturated ? " SAT" : "",
                 serial_run_enabled ? "RUN" :
                 (compute_only_enabled ? "COMPUTE_ONLY" : "STOP"));
}

// ==============================================================================
//                                    SETUP
// ==============================================================================
void setup() {
  // Serial 통신, Receiver, HR Controller 초기화
  Serial0.begin(115200);              // FTDI/UART0 command and status port
  if (USE_SBUS_RECEIVER) receiver.begin();
  if (!HR_controller.begin()) {
    Serial.println("[ERROR] Failed to initialize LU9685 I2C bus");
  }

  // IMU (MPU6050) 초기화
  if (!MPU6050.begin()) {
    Serial.println("[ERROR] Fail to initialize IMU.");
  }

  motor1Serial.begin(MOTOR_BAUDRATE, SERIAL_8N1, MOTOR1_RX_PIN, MOTOR1_TX_PIN);
  motor2Serial.begin(MOTOR_BAUDRATE, SERIAL_8N1, MOTOR2_RX_PIN, MOTOR2_TX_PIN);

  // WIFI 연결
  if (USE_WIFI_LOGGER) WIFI_Logger.begin();

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

  while (USE_SBUS_RECEIVER && !receiver.readData()) {
    if (receiver_timer.getDuration() > timeout) {
      Serial.println("Timeout: No data received from SBUS.");
      receiver_timer.start();  // 타임아웃 초기화
    }
  }
  if (USE_SBUS_RECEIVER) receiver.updateData();

  // Logger pre-allocation ==============================================
#if USE_WIFI_LOGGER
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
#endif
  // =======================================================================

  // 시간 측정 시작
  log_timer.start();
  sampling_timer.start();
}

// ==============================================================================
//                                    LOOP
// ==============================================================================
void loop() {
  updateSerialControl();

  if (diagnostic_output_enabled && millis() - last_diagnostic_ms >= 1000) {
    last_diagnostic_ms = millis();
    printDiagnosticSnapshot(!serial_run_enabled && !compute_only_enabled);
  }

  // sampling time이 경과했을 때만 실행
  if (sampling_timer.getDuration() >= dt * 1000) {
    // 경과 시간 출력
    Serial.print("SamplingTime(ms):");
    Serial.print(sampling_timer.getDuration());
    Serial.print(" ");

    sampling_timer.start();  // sampling timer 초기화


    if (USE_SBUS_RECEIVER && receiver.readData()) {
      receiver.updateData();
    }

    if (serial_run_enabled || compute_only_enabled ||
        (USE_SBUS_RECEIVER && receiver.isRun())) {
      // Running Mode
      //// update Pol state and input for EKF ////
      Pol.setState(x);
      // Pol.setInput(u);
      Pol.setInput(u_prev);  // u_k-2

      // measurement update
      imu_read_ok = MPU6050.readData();  // read k-th IMU measurements
      if (imu_read_ok) MPU6050.getIMUMeasurement(z);

      motor_rw_read_ok = ServoRW.sendCommandReadMotorState2();
      motor_lw_read_ok = ServoLW.sendCommandReadMotorState2();
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
          if (USE_SBUS_RECEIVER && receiver.readData()) {
            receiver.updateData();
          }
          delay(500);
          if (!serial_run_enabled &&
              (!USE_SBUS_RECEIVER || !receiver.isRun())) {
            return;
          }
        }
      }

      //// Get Desired States from receiver ////
      if (USE_SBUS_RECEIVER) {
        receiver.updateDesiredStates();
        h_d = receiver.getDesiredHeight();
        v_d = receiver.getDesiredVel();
        dpsi_d = receiver.getDesiredYawVel();
      }
      phi_d = 0;  // roll control disable

      x_d.segment<2>(2) << v_d, dpsi_d;

      //// calculate CoM and Inertia from CoM Calculator ////
      Pol.setHR(h_d, phi_d);
      Pol.calculate_com_and_inertia();  // 여기서 inverse kinematics로 theta_hips도 계산됨
      Pol.get_theta_eq(x_d(0));         // update desired pitch angle with equilibrium point

      //// compute VYB controller gain ////
      VYB_controller.computeGainK(h_d);
      VYB_controller.computeInput(x_d, x);

      //// Send control command of HR controller and VYB controller
      if (serial_run_enabled || (USE_SBUS_RECEIVER && receiver.isRun())) {
        HR_controller.controlHipServos(Pol.get_theta_hips());
        VYB_controller.sendControlCommand();
      }
      u_prev = u;
      u = VYB_controller.getInputVector();

      //============= Logging ======================================================================
#if USE_WIFI_LOGGER
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
#endif
      //=============================================================================================
    } else if (serial_reset_requested ||
               (USE_SBUS_RECEIVER && receiver.isReset())) {
      // Estimator Reset
      x_d.setZero();
      VYB_controller.theta_d = 0.f;
      x.setZero();
      u.setZero();
      Estimator.reset_estimator();
      if (USE_WIFI_LOGGER) WIFI_Logger.resetLogData();
      log_timer.start();
      serial_reset_requested = false;

    } else {
      // Off Mode
      ServoLW.sendTorqueControlCommand(0);
      ServoRW.sendTorqueControlCommand(0);
      u.setZero();

      if (USE_WIFI_LOGGER) WIFI_Logger.handleClientRequests();

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
          if (USE_SBUS_RECEIVER && receiver.readData()) {
            receiver.updateData();
          }
          delay(500);
          if (!serial_run_enabled &&
              (!USE_SBUS_RECEIVER || !receiver.isRun())) {
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
