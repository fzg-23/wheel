#include "Params.h"
#include "Receiver.h"
#include "HRController.h"
#include "VYBController.h"
#include "MGServo.h"
#include "IMU.h"
#include "POL.h"
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

Logger WIFI_Logger(ssid, password);

Eigen::Matrix<float, 8, 1> z = Eigen::Matrix<float, 8, 1>::Zero();
Eigen::Matrix<int16_t, 2, 1> iq_inputs = Eigen::Matrix<int16_t, 2, 1>::Zero();
Eigen::Matrix<int16_t, 2, 1> iq_outputs = Eigen::Matrix<int16_t, 2, 1>::Zero();

int i = 0;
Timer log_timer(Timer::TimerType::Millis);
Timer sampling_timer(Timer::TimerType::Millis);
Timer temp_timer(Timer::TimerType::Micros);
float h_d = HEIGHT_MAX, phi_d = 0;

std::vector<int16_t> command_vec;  // torque command vector (LSD)
int16_t command_max = 1000;
int16_t command_increment = 10;  // LSD
int command_idx = 0;
int dt_command = 42;  // milli sec

void serialPrintStates();

void setup() {
  // ================================
  // Command 벡터 초기화
  // ================================
  for (int16_t command = command_increment; command < command_max; command += command_increment) {
    command_vec.push_back(command);   // 명령 벡터에 양수 값 추가
    command_vec.push_back(-command);  // 명령 벡터에 음수 값 추가
  }

  // ================================
  // 시리얼 통신, 리시버, 서보 컨트롤러 초기화
  // ================================
  Serial.begin(115200);                        // 시리얼 통신 시작
  receiver.begin();                            // 리시버 시작
  HR_controller.attachServos(LH_PIN, RH_PIN);  // 서보 핀 설정

  // ================================
  // IMU(MPU6050) 초기화
  // ================================
  if (!MPU6050.begin()) {
    Serial.println("[ERROR] Fail to initialize IMU.");
  }

  // ================================
  // RS485 초기화
  // ================================
  pinMode(RS485_DE_RE, OUTPUT);    // RS485 방향 제어 핀 설정
  digitalWrite(RS485_DE_RE, LOW);  // RS485 수신 모드 설정
  RS485.begin(460800, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);  // RS485 통신 시작

  // ================================
  // WIFI 연결 설정
  // ================================
  WIFI_Logger.begin();  // WIFI 연결 초기화

  // ================================
  // PSRAM 상태 확인 및 초기화
  // ================================
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

  // ================================
  // SBUS 데이터 수신 대기 (타임아웃 처리)
  // ================================
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

  // ================================
  // Logger pre-allocation
  // ================================
  WIFI_Logger.readyToLogValue("cal_time");
  WIFI_Logger.readyToLogTimeStamp();            // 시간 기록
  WIFI_Logger.readyToLogValue("h_d");           // (m)
  WIFI_Logger.readyToLogValue("tau_RW");        // (LSD)
  WIFI_Logger.readyToLogValue("tau_LW");        // (LSD)
  WIFI_Logger.readyToLogValue("acc_x");         // m/s^2
  WIFI_Logger.readyToLogValue("acc_y");         // m/s^2
  WIFI_Logger.readyToLogValue("acc_z");         // m/s^2
  WIFI_Logger.readyToLogValue("gyr_x");         // rad/s
  WIFI_Logger.readyToLogValue("gyr_y");         // rad/s
  WIFI_Logger.readyToLogValue("gyr_z");         // rad/s
  WIFI_Logger.readyToLogValue("theta_dot_RW");  // rad/s
  WIFI_Logger.readyToLogValue("theta_dot_LW");  // rad/s
  WIFI_Logger.readyToLogValue("iq_RW");         // (LSD)
  WIFI_Logger.readyToLogValue("iq_LW");         // (LSD)
  // WIFI_Logger.readyToLogValue("log_time");      // (us)


  // ================================
  // 시간 측정 시작
  // ================================
  log_timer.start();
  sampling_timer.start();
}


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
      if (i >= dt_command / (dt * 1000)) {
        if (command_idx < command_vec.size()) {
          iq_inputs << command_vec.at(command_idx), command_vec.at(command_idx);
          // iq_inputs << 0, 0;
          command_idx++;
          i = 0;
        } else {
          iq_inputs << 0, 0;
        }
      }

      receiver.updateDesiredStates();
      h_d = receiver.getDesiredHeight();
      phi_d = 0;  // roll control diable


      Pol.setHR(h_d, phi_d);
      // temp_timer.start();
      Pol.solve_inverse_kinematics();
      // Serial.print("inverse_kinematics_time(us):");
      // Serial.print(temp_timer.getDuration());
      // Serial.print(" ");

      // temp_timer.start();
      // HR_controller.controlHipServos(Pol.get_theta_hips());
      VYB_controller.sendDirectControlCommand(iq_inputs);
      // Serial.print("send_command_time(us):");
      // Serial.print(temp_timer.getDuration());
      // Serial.print(" ");

      // temp_timer.start();
      // measurement update
      MPU6050.readData();
      MPU6050.getIMUMeasurement(z);
      VYB_controller.getMotorSpeedMeasurement(z);
      VYB_controller.getMotorCurrentMeasurement(iq_outputs);
      // Serial.print("measurement_update_time(us):");
      // Serial.print(temp_timer.getDuration());
      // Serial.print(" ");

      // MPU6050.printData();
      Serial.println(" Run Mode");

      ///// Logging /////
      WIFI_Logger.logValue("cal_time", sampling_timer.getDuration());

      // temp_timer.start();
      WIFI_Logger.logTimeStamp(log_timer.getDuration());  // 시간 기록
      // state 기록
      WIFI_Logger.logValue("h_d", h_d);  // (m)

      WIFI_Logger.logValue("tau_RW", iq_inputs(0));  // (LSD)
      WIFI_Logger.logValue("tau_LW", iq_inputs(1));  // (LSD)

      WIFI_Logger.logValue("acc_x", z(0));                         // m/s^2
      WIFI_Logger.logValue("acc_y", z(1));                         // m/s^2
      WIFI_Logger.logValue("acc_z", z(2));                         // m/s^2
      WIFI_Logger.logValue("gyr_x", z(3));                         // rad/s
      WIFI_Logger.logValue("gyr_y", z(4));                         // rad/s
      WIFI_Logger.logValue("gyr_z", z(5));                         // rad/s
      WIFI_Logger.logValue("theta_dot_RW", z(6));                  // rad/s
      WIFI_Logger.logValue("theta_dot_LW", z(7));                  // rad/s
      WIFI_Logger.logValue("iq_RW", iq_outputs(0));                // (LSD)
      WIFI_Logger.logValue("iq_LW", iq_outputs(1));                // (LSD)
      // WIFI_Logger.logValue("log_time", temp_timer.getDuration());  // (us)
      ////////////////////

      i++;
    } else if (receiver.isReset()) {
      // Estimator Reset
      iq_inputs.setZero();
      WIFI_Logger.resetLogData();
      i = 0;
      command_idx = 0;
      log_timer.start();

    } else {
      // Off Mode
      ServoLW.sendTorqueControlCommand(0);
      ServoRW.sendTorqueControlCommand(0);

      // measurement update
      MPU6050.readData();
      MPU6050.getIMUMeasurement(z);
      VYB_controller.getMotorSpeedMeasurement(z);
      VYB_controller.getMotorCurrentMeasurement(iq_outputs);

      WIFI_Logger.handleClientRequests();  // Log Data 전송

      Serial.println(" Off Mode");
    }
  }
}