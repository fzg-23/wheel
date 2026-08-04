#ifndef LU9685_MPU6050_COMBINED_TEST_H
#define LU9685_MPU6050_COMBINED_TEST_H

#include <Arduino.h>
#include <Wire.h>

namespace HRMPUTest {

#define TEST_SERIAL Serial0

// Use both ESP32-S3 I2C controllers so the two devices have no pin conflict.
constexpr int LU9685_SDA_PIN = 8;
constexpr int LU9685_SCL_PIN = 9;
constexpr int MPU6050_SDA_PIN = 10;
constexpr int MPU6050_SCL_PIN = 11;
constexpr uint32_t LU9685_I2C_CLOCK = 100000;
constexpr uint32_t MPU6050_I2C_CLOCK = 400000;

constexpr uint8_t LU9685_ADDRESS = 0x00;
constexpr uint8_t SERVO_CHANNELS[] = {0, 2};

constexpr uint8_t MPU_ADDRESS_LOW = 0x68;
constexpr uint8_t MPU_ADDRESS_HIGH = 0x69;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint32_t MPU_PRINT_INTERVAL_MS = 500;

// Two independent automatic-direction TTL/RS485 channels from testMG.h.
constexpr int MOTOR1_RX_PIN = 18;
constexpr int MOTOR1_TX_PIN = 17;
constexpr int MOTOR2_RX_PIN = 16;
constexpr int MOTOR2_TX_PIN = 15;
constexpr uint32_t MOTOR_BAUD = 115200;
constexpr uint8_t MOTOR1_ID = 1;
constexpr uint8_t MOTOR2_ID = 2;
constexpr int16_t MAX_MOTOR_COMMAND = 100;
constexpr uint32_t MOTOR_WATCHDOG_MS = 1000;
constexpr uint32_t MOTOR_RESPONSE_TIMEOUT_US = 5000;
constexpr uint8_t MOTOR_FRAME_HEAD = 0x3E;
constexpr uint8_t MOTOR_TORQUE_COMMAND = 0xA1;
constexpr uint8_t MOTOR_READ_STATE2_COMMAND = 0x9C;
constexpr size_t MOTOR_RESPONSE_LENGTH = 13;

HardwareSerial motor1Serial(1);
HardwareSerial motor2Serial(2);
int16_t motor1Command = 0;
int16_t motor2Command = 0;
uint32_t lastMotorCommandMs = 0;

uint8_t mpuAddress = 0;
bool mpuOutputEnabled = false;
uint32_t lastMpuPrintMs = 0;

struct MotorChannel {
  uint8_t id;
  HardwareSerial& serial;
  const char* name;
};

struct MotorState {
  int8_t temperatureC = 0;
  int16_t iqRaw = 0;
  int16_t speedRaw = 0;
  uint16_t encoder = 0;
};

MotorChannel motor1{MOTOR1_ID, motor1Serial, "Motor 1"};
MotorChannel motor2{MOTOR2_ID, motor2Serial, "Motor 2"};

uint8_t motorChecksum(const uint8_t* data, size_t length) {
  uint16_t sum = 0;
  for (size_t i = 0; i < length; ++i) sum += data[i];
  return static_cast<uint8_t>(sum);
}

void clearMotorRx(HardwareSerial& serial) {
  while (serial.available()) serial.read();
}

bool readMotorExact(HardwareSerial& serial, uint8_t* data, size_t length) {
  const uint32_t start = micros();
  size_t received = 0;
  while (received < length) {
    while (serial.available() && received < length) {
      data[received++] = static_cast<uint8_t>(serial.read());
    }
    if (micros() - start >= MOTOR_RESPONSE_TIMEOUT_US) return false;
  }
  return true;
}

bool decodeMotorState(const uint8_t* response, uint8_t expectedId,
                      MotorState& state) {
  if (response[0] != MOTOR_FRAME_HEAD || response[2] != expectedId ||
      response[3] != 7 || motorChecksum(response, 4) != response[4] ||
      motorChecksum(response + 5, 7) != response[12]) {
    return false;
  }
  state.temperatureC = static_cast<int8_t>(response[5]);
  state.iqRaw = static_cast<int16_t>(response[6] | (response[7] << 8));
  state.speedRaw = static_cast<int16_t>(response[8] | (response[9] << 8));
  state.encoder = static_cast<uint16_t>(response[10] | (response[11] << 8));
  return true;
}

bool motorTransact(MotorChannel& motor, const uint8_t* frame,
                   size_t frameLength, MotorState& state) {
  uint8_t response[MOTOR_RESPONSE_LENGTH];
  clearMotorRx(motor.serial);
  motor.serial.write(frame, frameLength);
  motor.serial.flush();
  if (!readMotorExact(motor.serial, response, sizeof(response))) {
    TEST_SERIAL.printf("[ERROR] %s ID %u response timeout\n",
                       motor.name, motor.id);
    return false;
  }
  if (!decodeMotorState(response, motor.id, state)) {
    TEST_SERIAL.printf("[ERROR] %s ID %u invalid response\n",
                       motor.name, motor.id);
    return false;
  }
  return true;
}

void printMotorState(const MotorChannel& motor, const MotorState& state) {
  TEST_SERIAL.printf(
      "%s ID=%u | temp=%d C | iqRaw=%d | speed=%.1f deg/s | encoder=%u\n",
      motor.name, motor.id, state.temperatureC, state.iqRaw,
      state.speedRaw / 10.0f, state.encoder);
}

bool setMotorTorque(MotorChannel& motor, int requested,
                    bool printResult = true) {
  const int16_t command = static_cast<int16_t>(
      constrain(requested, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND));
  uint8_t frame[8] = {
      MOTOR_FRAME_HEAD, MOTOR_TORQUE_COMMAND, motor.id, 2, 0,
      static_cast<uint8_t>(command & 0xFF),
      static_cast<uint8_t>((command >> 8) & 0xFF), 0};
  frame[4] = motorChecksum(frame, 4);
  frame[7] = motorChecksum(frame + 5, 2);
  MotorState state;
  const bool ok = motorTransact(motor, frame, sizeof(frame), state);
  if (ok && printResult) {
    TEST_SERIAL.printf("Torque command=%d | ", command);
    printMotorState(motor, state);
  }
  return ok;
}

bool readMotorState(MotorChannel& motor) {
  uint8_t frame[5] = {
      MOTOR_FRAME_HEAD, MOTOR_READ_STATE2_COMMAND, motor.id, 0, 0};
  frame[4] = motorChecksum(frame, 4);
  MotorState state;
  const bool ok = motorTransact(motor, frame, sizeof(frame), state);
  if (ok) printMotorState(motor, state);
  return ok;
}

void stopMotors(bool printMessage = true) {
  setMotorTorque(motor1, 0, false);
  setMotorTorque(motor2, 0, false);
  motor1Command = 0;
  motor2Command = 0;
  if (printMessage) TEST_SERIAL.println("STOP: both motor commands are zero");
}

void handleMotorCommand(const String& text) {
  char target = 0;
  int first = 0;
  int second = 0;
  const int count = sscanf(text.c_str(), " w%c %d %d", &target, &first, &second);
  target = static_cast<char>(tolower(target));

  if (target == 's') {
    stopMotors();
  } else if (target == 'q') {
    readMotorState(motor1);
    readMotorState(motor2);
  } else if (target == '1' && count >= 2) {
    setMotorTorque(motor1, first);
    motor1Command = constrain(first, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
    lastMotorCommandMs = millis();
  } else if (target == '2' && count >= 2) {
    setMotorTorque(motor2, first);
    motor2Command = constrain(first, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
    lastMotorCommandMs = millis();
  } else if (target == 'b' && count >= 3) {
    setMotorTorque(motor1, first);
    setMotorTorque(motor2, second);
    motor1Command = constrain(first, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
    motor2Command = constrain(second, -MAX_MOTOR_COMMAND, MAX_MOTOR_COMMAND);
    lastMotorCommandMs = millis();
  } else {
    TEST_SERIAL.println("Motor commands: w1 <torque>, w2 <torque>, wb <m1> <m2>, wq, ws");
  }
}

bool addressAcknowledges(TwoWire& bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

bool writeMPURegister(uint8_t reg, uint8_t value) {
  if (mpuAddress == 0) return false;
  Wire1.beginTransmission(mpuAddress);
  Wire1.write(reg);
  Wire1.write(value);
  return Wire1.endTransmission(true) == 0;
}

bool readMPURegisters(uint8_t reg, uint8_t* data, size_t length) {
  if (mpuAddress == 0) return false;
  Wire1.beginTransmission(mpuAddress);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;

  const size_t received = Wire1.requestFrom(
      mpuAddress, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
  if (received != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = Wire1.read();
  return true;
}

bool initializeMPU6050() {
  if (addressAcknowledges(Wire1, MPU_ADDRESS_LOW)) {
    mpuAddress = MPU_ADDRESS_LOW;
  } else if (addressAcknowledges(Wire1, MPU_ADDRESS_HIGH)) {
    mpuAddress = MPU_ADDRESS_HIGH;
  } else {
    return false;
  }

  uint8_t whoAmI = 0;
  if (!readMPURegisters(REG_WHO_AM_I, &whoAmI, 1) ||
      (whoAmI & 0x7E) != 0x68) {
    mpuAddress = 0;
    return false;
  }

  TEST_SERIAL.printf("MPU6050 ACK: address=0x%02X, WHO_AM_I=0x%02X\n",
                     mpuAddress, whoAmI);
  if (!writeMPURegister(REG_PWR_MGMT_1, 0x01)) return false;
  delay(100);
  return writeMPURegister(REG_SMPLRT_DIV, 9) &&
         writeMPURegister(REG_CONFIG, 0x03) &&
         writeMPURegister(REG_GYRO_CONFIG, 0x00) &&
         writeMPURegister(REG_ACCEL_CONFIG, 0x00);
}

int16_t combineBytes(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

void printMPU6050() {
  uint8_t data[14];
  if (!readMPURegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) {
    TEST_SERIAL.println("[ERROR] MPU6050 read failed");
    return;
  }

  const float ax = combineBytes(data[0], data[1]) / 16384.0f;
  const float ay = combineBytes(data[2], data[3]) / 16384.0f;
  const float az = combineBytes(data[4], data[5]) / 16384.0f;
  const float temperature = combineBytes(data[6], data[7]) / 340.0f + 36.53f;
  const float gx = combineBytes(data[8], data[9]) / 131.0f;
  const float gy = combineBytes(data[10], data[11]) / 131.0f;
  const float gz = combineBytes(data[12], data[13]) / 131.0f;

  TEST_SERIAL.printf(
      "MPU acc[g] X:%+.3f Y:%+.3f Z:%+.3f | gyro[d/s] X:%+.2f Y:%+.2f Z:%+.2f | T:%.2f C\n",
      ax, ay, az, gx, gy, gz, temperature);
}

bool setLU9685Angle(uint8_t channel, float angle) {
  if ((channel != 0 && channel != 2) || angle < 0.0f || angle > 180.0f) {
    return false;
  }

  const uint8_t rawAngle = static_cast<uint8_t>(lroundf(angle));
  Wire.beginTransmission(LU9685_ADDRESS);
  Wire.write(channel);
  Wire.write(rawAngle);
  if (Wire.endTransmission() != 0) {
    TEST_SERIAL.printf("[ERROR] LU9685 write failed on channel %u\n", channel);
    return false;
  }

  TEST_SERIAL.printf("Servo CH%u: %.1f deg -> raw angle: %u\n",
                     channel, angle, rawAngle);
  return true;
}

bool setBothServoAngles(float angle) {
  if (angle < 0.0f || angle > 180.0f) return false;
  for (uint8_t channel : SERVO_CHANNELS) {
    if (!setLU9685Angle(channel, angle)) return false;
  }
  return true;
}

void printHelp() {
  TEST_SERIAL.println("Commands:");
  TEST_SERIAL.println("  <angle>       set servo channels 0 and 2 (0..180)");
  TEST_SERIAL.println("  <ch> <angle>  set servo channel 0 or 2");
  TEST_SERIAL.println("  m             enable/disable continuous MPU output");
  TEST_SERIAL.println("  r             print one MPU measurement");
  TEST_SERIAL.println("  w1 <value>    motor 1 torque (-100..100)");
  TEST_SERIAL.println("  w2 <value>    motor 2 torque (-100..100)");
  TEST_SERIAL.println("  wb <v1> <v2> both motor torques");
  TEST_SERIAL.println("  wq / ws       read motors / stop motors");
  TEST_SERIAL.println("  help          show commands");
}

void handleCommand(String text) {
  text.trim();
  if (text.isEmpty()) return;

  if (text.equalsIgnoreCase("help") || text == "?") {
    printHelp();
    return;
  }
  if (text.equalsIgnoreCase("m")) {
    mpuOutputEnabled = !mpuOutputEnabled;
    TEST_SERIAL.printf("Continuous MPU output: %s\n",
                       mpuOutputEnabled ? "ON" : "OFF");
    return;
  }
  if (text.equalsIgnoreCase("r")) {
    if (mpuAddress != 0) printMPU6050();
    else TEST_SERIAL.println("[ERROR] MPU6050 is not initialized");
    return;
  }
  if (text[0] == 'w' || text[0] == 'W') {
    handleMotorCommand(text);
    return;
  }

  char* end = nullptr;
  const float first = strtof(text.c_str(), &end);
  if (end == text.c_str()) {
    TEST_SERIAL.println("Invalid command; enter help");
    return;
  }
  while (*end == ' ') ++end;

  if (*end == '\0') {
    if (!setBothServoAngles(first)) {
      TEST_SERIAL.println("Out of range. Enter angle 0..180.");
    }
    return;
  }

  char* angleEnd = nullptr;
  const float angle = strtof(end, &angleEnd);
  while (*angleEnd == ' ') ++angleEnd;
  const int channel = static_cast<int>(first);
  if (*angleEnd != '\0' || first != channel ||
      !setLU9685Angle(static_cast<uint8_t>(channel), angle)) {
    TEST_SERIAL.println("Use: <ch> <angle>, ch is 0 or 2, angle is 0..180");
  }
}

}  // namespace HRMPUTest

void setup() {
  using namespace HRMPUTest;

  TEST_SERIAL.begin(115200);
  delay(1000);
  if (!Wire.begin(LU9685_SDA_PIN, LU9685_SCL_PIN, LU9685_I2C_CLOCK)) {
    TEST_SERIAL.println("[ERROR] Failed to start LU9685 I2C bus");
    return;
  }
  Wire.setTimeOut(50);
  if (!Wire1.begin(MPU6050_SDA_PIN, MPU6050_SCL_PIN, MPU6050_I2C_CLOCK)) {
    TEST_SERIAL.println("[ERROR] Failed to start MPU6050 I2C bus");
    return;
  }
  Wire1.setTimeOut(50);

  motor1Serial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR1_RX_PIN, MOTOR1_TX_PIN);
  motor2Serial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR2_RX_PIN, MOTOR2_TX_PIN);

  initializeMPU6050();
}

void loop() {
  using namespace HRMPUTest;
  static String line;

  while (TEST_SERIAL.available()) {
    const char ch = static_cast<char>(TEST_SERIAL.read());
    if (ch == '\r' || ch == '\n') {
      handleCommand(line);
      line = "";
    } else if (ch == '\b' || ch == 0x7F) {
      if (!line.isEmpty()) line.remove(line.length() - 1);
    } else if (isPrintable(ch)) {
      line += ch;
    }
  }

  const uint32_t now = millis();
  if (mpuOutputEnabled && mpuAddress != 0 &&
      now - lastMpuPrintMs >= MPU_PRINT_INTERVAL_MS) {
    lastMpuPrintMs = now;
    printMPU6050();
  }

  if ((motor1Command != 0 || motor2Command != 0) &&
      now - lastMotorCommandMs >= MOTOR_WATCHDOG_MS) {
    TEST_SERIAL.println("[WATCHDOG] Motor command timeout");
    stopMotors();
  }
}

#endif  // LU9685_MPU6050_COMBINED_TEST_H
