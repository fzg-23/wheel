#ifndef MGSERVO_H
#define MGSERVO_H

#include <Arduino.h>
#include "Params.h"

// 定义命令和响应帧的长度
constexpr uint8_t FRAME_COMMAND_LENGTH = 5;
constexpr uint8_t TORQUE_CONTROL_LENGTH = 2;
constexpr uint8_t SPEED_CONTROL_LENGTH = 4;
constexpr uint8_t READ_MOTOR_STATE2_LENGTH = 7;

// 命令类型定义
constexpr uint8_t FRAME_HEAD = 0x3E;
constexpr uint8_t TORQUE_CONTROL_COMMAND = 0xA1;
constexpr uint8_t SPEED_CONTROL_COMMAND = 0xA2;
constexpr uint8_t READ_MOTOR_STATE2_COMMAND = 0x9C;

// MGServo类定义
class MGServo {
private:
  HardwareSerial& RS485;
  bool automaticDirection;

  uint8_t motorID;
  int8_t temperature;  // temperature (℃)
  uint16_t encoder;
  int16_t iq_raw;     // raw current (LSD)
  int16_t speed_raw;  // speed raw (LSD)
  float iq;           // current (A)
  float speed;        // speed (deg/s)


public:
  MGServo(uint8_t motorID, HardwareSerial& RS485_ref,
          bool automaticDirection_ = false)
    : RS485(RS485_ref), automaticDirection(automaticDirection_),
      motorID(motorID) {
    temperature = 0;
    encoder = 0;
    iq_raw = 0;
    speed_raw = 0;
    iq = 0;
    speed = 0;
  }

  //////// Getter functions ////////////////
  uint8_t getMotorID() const {
    return motorID;
  }

  int8_t getMotorTemperature() const {
    return temperature;
  }

  uint16_t getMotorEncoder() const {
    return encoder;
  }

  int16_t getMotorIqRaw() const {
    return iq_raw;
  }

  float getMotorIq() const {
    return iq;
  }

  float getMotorSpeed() const {
    return speed;
  }
  ////////////////////////////////////////////

  // 扭矩控制指令传输
  bool sendTorqueControlCommand(int16_t iqControl) {
    const size_t commandLength = FRAME_COMMAND_LENGTH + TORQUE_CONTROL_LENGTH + 1;      // frame + data + data_checksum
    const size_t responseLength = FRAME_COMMAND_LENGTH + READ_MOTOR_STATE2_LENGTH + 1;  // frame + data + data_checksum

    uint8_t command[commandLength];
    uint8_t response[responseLength];

    writeTorqueControlCommand(command, iqControl);  // 在命令数组中创建命令帧
    sendCommand(command, commandLength);            // 通过RS485传输命令

    // 通过RS485接收响应并验证响应
    if (!readResponse(response, responseLength) || !validateResponse(response, responseLength)) {
      return false;
    }

    // 通过响应信息更新电机状态
    extractReadState2Data(response);
    return true;
  }

  // 发送电机状态2读取命令
  bool sendCommandReadMotorState2() {
    const size_t commandLength = FRAME_COMMAND_LENGTH;                                  // frame + data + data_checksum
    const size_t responseLength = FRAME_COMMAND_LENGTH + READ_MOTOR_STATE2_LENGTH + 1;  // frame + data + data_checksum

    uint8_t command[commandLength];
    uint8_t response[responseLength];

    writeReadState2Command(command);      // 在命令数组中创建命令帧
    sendCommand(command, commandLength);  // 通过RS485传输命令

    // 通过RS485接收响应并验证响应
    if (!readResponse(response, responseLength) || !validateResponse(response, responseLength)) {
      return false;
    }

    // 通过响应信息更新电机状态
    extractReadState2Data(response);
    return true;
  }

  void printMotorState() const {
    Serial.print("Temperature(");
    Serial.print(motorID);
    Serial.print("):");
    Serial.print(temperature);
    Serial.print(" Encoder(");
    Serial.print(motorID);
    Serial.print("):");
    Serial.print(encoder);
    Serial.print(" Speed(");
    Serial.print(motorID);
    Serial.print("):");
    Serial.print(speed);
    Serial.print(" Current(");
    Serial.print(motorID);
    Serial.print("):");
    Serial.print(iq);
  }

private:

  // 扭矩控制命令帧准备
  void writeTorqueControlCommand(uint8_t* command, int16_t iqControl) {
    prepareCommandFrame(command, TORQUE_CONTROL_COMMAND, TORQUE_CONTROL_LENGTH);
    command[FRAME_COMMAND_LENGTH] = iqControl & 0xFF;
    command[FRAME_COMMAND_LENGTH + 1] = (iqControl >> 8) & 0xFF;
    command[FRAME_COMMAND_LENGTH + 2] = calculateChecksum(&command[FRAME_COMMAND_LENGTH], TORQUE_CONTROL_LENGTH);
  }

  // 速度控制命令帧准备
  void writeSpeedControlCommand(uint8_t* command, int32_t speedControl) {
    prepareCommandFrame(command, SPEED_CONTROL_COMMAND, SPEED_CONTROL_LENGTH);
    command[FRAME_COMMAND_LENGTH] = *(uint8_t*)(&speedControl);
    command[FRAME_COMMAND_LENGTH + 1] = *((uint8_t*)(&speedControl) + 1);
    command[FRAME_COMMAND_LENGTH + 2] = *((uint8_t*)(&speedControl) + 2);
    command[FRAME_COMMAND_LENGTH + 3] = *((uint8_t*)(&speedControl) + 3);
    command[FRAME_COMMAND_LENGTH + 4] = calculateChecksum(&command[FRAME_COMMAND_LENGTH], SPEED_CONTROL_LENGTH);
  }

  // 电机状态 2 命令帧就绪
  void writeReadState2Command(uint8_t* command) {
    prepareCommandFrame(command, READ_MOTOR_STATE2_COMMAND, 0x00);
  }

  // 准备命令帧
  void prepareCommandFrame(uint8_t* command, const uint8_t commandType, const uint8_t dataLength) {
    command[0] = FRAME_HEAD;
    command[1] = commandType;                    // 命令类型
    command[2] = motorID;                        // 电机编号
    command[3] = dataLength;                     // 数据长度
    command[4] = calculateChecksum(command, 4);  // 校验和计算
  }

  // 电机状态2数据提取
  void extractReadState2Data(const uint8_t* response) {
    temperature = (int8_t)response[FRAME_COMMAND_LENGTH];
    iq_raw = (int16_t)(response[FRAME_COMMAND_LENGTH + 1] | (response[FRAME_COMMAND_LENGTH + 2] << 8));
    iq = static_cast<float>(iq_raw) * 3.3 / 2048;
    speed_raw = (int16_t)(response[FRAME_COMMAND_LENGTH + 3] | (response[FRAME_COMMAND_LENGTH + 4] << 8));
    speed = static_cast<float>(speed_raw) / 10;
    encoder = (uint16_t)(response[FRAME_COMMAND_LENGTH + 5] | (response[FRAME_COMMAND_LENGTH + 6] << 8));
  }

  // 发送命令
  void sendCommand(const uint8_t* command, const size_t commandLength) {
    // 发送/接收缓冲区初始化
    while (RS485.available()) {
      RS485.read();  // 空缓冲区
    }

    // 切换至RS485传输模式
    toggleRS485Mode(HIGH);

    // 发送命令
    RS485.write(command, commandLength);
    RS485.flush();  // 等待传输完成

    // 返回RS485接收模式
    toggleRS485Mode(LOW);
  }


  // 读取回复
  bool readResponse(uint8_t* response, const size_t responseLength) {
    toggleRS485Mode(LOW);

    unsigned long startTime = micros();  // 计时器开始
    while (RS485.available() < responseLength) {
      if (micros() - startTime > 5000) {
            Serial.println("[ERROR] Response timeout! No sufficient data received.");
            return false;  // 发生超时
        }
    }
    size_t bytesRead = RS485.readBytes(response, responseLength);

    if (bytesRead != responseLength) {
      Serial.print("[ERROR] Failed to read the full response! ");
      printInfo();
      return false;
    }

    return true;
  }

  // 响应验证
  bool validateResponse(const uint8_t* response, size_t responseLength) const {
    if (calculateChecksum(response, 4) != response[4]) {
      Serial.print("[ERROR] Frame Checksum mismatch!");
      printInfo();
      return false;
    }

    uint8_t dataLength = response[3];
    if (calculateChecksum(&response[5], dataLength) != response[responseLength - 1]) {
      Serial.print("[ERROR] Data Checksum mismatch!");
      printInfo();
      return false;
    }

    if (response[2] != motorID) {
      Serial.print("[ERROR] ID mismatch!");
      printInfo();
      return false;
    }

    return true;
  }

  // RS485模式切换
  void toggleRS485Mode(uint8_t mode) const {
    if (!automaticDirection) digitalWrite(RS485_DE_RE, mode);
  }

  // 校验和计算
  uint8_t calculateChecksum(const uint8_t* data, uint8_t length) const {
    uint16_t checksum = 0;
    for (uint8_t i = 0; i < length; ++i) {
      checksum += data[i];
    }
    return checksum & 0xFF;
  }

  // 输出信息用于调试
  void printInfo() const {
    Serial.print("Motor ID: ");
    Serial.print(motorID);
    Serial.print(" | Time: ");
    Serial.print(millis());
    Serial.println(" ms");
  }
};

#endif  // MGSERVO_H
