#ifndef WHEEL_MOTOR_TEST_H
#define WHEEL_MOTOR_TEST_H

#include <Arduino.h>

// Two independent automatic-direction TTL/RS485 channels.
// All four signal pins are on the same side of a common ESP32-S3 N16R8 board.
namespace WheelMotorTest {

#define TEST_SERIAL Serial

constexpr int MOTOR1_RX_PIN = 18;
constexpr int MOTOR1_TX_PIN = 17;
constexpr int MOTOR2_RX_PIN = 16;
constexpr int MOTOR2_TX_PIN = 15;
constexpr uint32_t MOTOR_BAUD = 115200;

constexpr uint8_t MOTOR1_ID = 1;
constexpr uint8_t MOTOR2_ID = 2;
constexpr int16_t MAX_TEST_COMMAND = 100;
constexpr uint32_t COMMAND_WATCHDOG_MS = 1000;
constexpr uint32_t RESPONSE_TIMEOUT_US = 5000;

constexpr uint8_t FRAME_HEAD = 0x3E;
constexpr uint8_t TORQUE_COMMAND = 0xA1;
constexpr uint8_t SPEED_COMMAND = 0xA2;
constexpr uint8_t READ_STATE2_COMMAND = 0x9C;
constexpr size_t STATE2_RESPONSE_LENGTH = 13;
constexpr int32_t AUTO_TEST_SPEED = 16000;  // 160 deg/s
constexpr uint32_t SPEED_COMMAND_INTERVAL_MS = 100;

HardwareSerial motor1Serial(1);
HardwareSerial motor2Serial(2);
uint32_t lastNonzeroCommandMs = 0;
uint32_t lastSpeedCommandMs = 0;
int16_t motor1Command = 0;
int16_t motor2Command = 0;
bool manualMotorRunning = false;
bool autoSpeedEnabled = true;

struct MotorChannel {
  uint8_t id;
  HardwareSerial& serial;
  const char* name;
};

MotorChannel motor1{MOTOR1_ID, motor1Serial, "Motor 1"};
MotorChannel motor2{MOTOR2_ID, motor2Serial, "Motor 2"};

struct MotorState {
  int8_t temperatureC = 0;
  int16_t iqRaw = 0;
  int16_t speedRaw = 0;
  uint16_t encoder = 0;
};

uint8_t checksum(const uint8_t* data, size_t length) {
  uint16_t sum = 0;
  for (size_t i = 0; i < length; ++i) sum += data[i];
  return static_cast<uint8_t>(sum);
}

void clearRx(HardwareSerial& serial) {
  while (serial.available()) serial.read();
}

bool readExact(HardwareSerial& serial, uint8_t* data, size_t length) {
  const uint32_t start = micros();
  size_t received = 0;
  while (received < length) {
    while (serial.available() && received < length) {
      data[received++] = static_cast<uint8_t>(serial.read());
    }
    if (micros() - start >= RESPONSE_TIMEOUT_US) return false;
  }
  return true;
}

bool validateAndDecode(const uint8_t* response, uint8_t expectedId,
                       MotorState& state) {
  if (response[0] != FRAME_HEAD || response[2] != expectedId ||
      response[3] != 7 || checksum(response, 4) != response[4] ||
      checksum(response + 5, 7) != response[12]) {
    return false;
  }

  state.temperatureC = static_cast<int8_t>(response[5]);
  state.iqRaw = static_cast<int16_t>(response[6] | (response[7] << 8));
  state.speedRaw = static_cast<int16_t>(response[8] | (response[9] << 8));
  state.encoder = static_cast<uint16_t>(response[10] | (response[11] << 8));
  return true;
}

bool transact(MotorChannel& motor, const uint8_t* command,
              size_t commandLength, MotorState& state) {
  uint8_t response[STATE2_RESPONSE_LENGTH];
  clearRx(motor.serial);
  motor.serial.write(command, commandLength);
  motor.serial.flush();

  if (!readExact(motor.serial, response, sizeof(response))) {
    TEST_SERIAL.printf("[ERROR] %s (ID %u) response timeout\n",
                       motor.name, motor.id);
    return false;
  }
  if (!validateAndDecode(response, motor.id, state)) {
    TEST_SERIAL.printf("[ERROR] %s (ID %u) invalid response\n",
                       motor.name, motor.id);
    return false;
  }
  return true;
}

void printState(const MotorChannel& motor, const MotorState& state) {
  TEST_SERIAL.printf(
      "%s ID=%u | temp=%d C | iqRaw=%d | speed=%.1f deg/s | encoder=%u\n",
      motor.name, motor.id, state.temperatureC, state.iqRaw,
      state.speedRaw / 10.0f, state.encoder);
}

bool setTorque(MotorChannel& motor, int commandValue,
               bool printResult = true) {
  const int16_t command = static_cast<int16_t>(
      constrain(commandValue, -MAX_TEST_COMMAND, MAX_TEST_COMMAND));
  uint8_t frame[8] = {
      FRAME_HEAD, TORQUE_COMMAND, motor.id, 2, 0,
      static_cast<uint8_t>(command & 0xFF),
      static_cast<uint8_t>((command >> 8) & 0xFF), 0};
  frame[4] = checksum(frame, 4);
  frame[7] = checksum(frame + 5, 2);

  MotorState state;
  const bool ok = transact(motor, frame, sizeof(frame), state);
  if (printResult && ok) {
    TEST_SERIAL.printf("Torque command=%d | ", command);
    printState(motor, state);
  }
  return ok;
}

bool setSpeed(MotorChannel& motor, int32_t speedCommand,
              bool printResult = true) {
  uint8_t frame[10] = {
      FRAME_HEAD, SPEED_COMMAND, motor.id, 4, 0,
      static_cast<uint8_t>(speedCommand & 0xFF),
      static_cast<uint8_t>((speedCommand >> 8) & 0xFF),
      static_cast<uint8_t>((speedCommand >> 16) & 0xFF),
      static_cast<uint8_t>((speedCommand >> 24) & 0xFF), 0};
  frame[4] = checksum(frame, 4);
  frame[9] = checksum(frame + 5, 4);

  MotorState state;
  const bool ok = transact(motor, frame, sizeof(frame), state);
  if (printResult && ok) {
    TEST_SERIAL.printf("Speed command=%.2f deg/s | ", speedCommand / 100.0f);
    printState(motor, state);
  }
  return ok;
}

bool readState(MotorChannel& motor) {
  uint8_t frame[5] = {FRAME_HEAD, READ_STATE2_COMMAND, motor.id, 0, 0};
  frame[4] = checksum(frame, 4);
  MotorState state;
  const bool ok = transact(motor, frame, sizeof(frame), state);
  if (ok) printState(motor, state);
  return ok;
}

void stopAll(bool printMessage = true) {
  setTorque(motor1, 0, false);
  setTorque(motor2, 0, false);
  motor1Command = 0;
  motor2Command = 0;
  manualMotorRunning = false;
  if (printMessage) TEST_SERIAL.println("STOP: both motor commands are zero");
}

void printHelp() {
  TEST_SERIAL.println("\n=== Dual wheel motor test ===");
  TEST_SERIAL.println("Raise both wheels off the ground before testing.");
  TEST_SERIAL.printf("M1 ID1: RX=%d TX=%d (UART1)\n",
                     MOTOR1_RX_PIN, MOTOR1_TX_PIN);
  TEST_SERIAL.printf("M2 ID2: RX=%d TX=%d (UART2)\n",
                     MOTOR2_RX_PIN, MOTOR2_TX_PIN);
  TEST_SERIAL.println("1 <value>       motor ID 1 torque, e.g. 1 20");
  TEST_SERIAL.println("2 <value>       motor ID 2 torque, e.g. 2 20");
  TEST_SERIAL.println("b <v1> <v2>    both motor torques, e.g. b 20 -20");
  TEST_SERIAL.println("q               read both motor states");
  TEST_SERIAL.println("s               emergency stop");
  TEST_SERIAL.println("g               resume automatic slow rotation");
  TEST_SERIAL.println("h               show help");
}

void recordManualCommand() {
  autoSpeedEnabled = false;
  manualMotorRunning = motor1Command != 0 || motor2Command != 0;
  lastNonzeroCommandMs = millis();
}

void handleCommand(String line) {
  line.trim();
  if (line.isEmpty()) return;

  char operation = 0;
  int first = 0;
  int second = 0;
  const int count = sscanf(line.c_str(), " %c %d %d",
                           &operation, &first, &second);

  if (operation == 's' || operation == 'S') {
    autoSpeedEnabled = false;
    stopAll();
  } else if (operation == 'g' || operation == 'G') {
    autoSpeedEnabled = true;
    manualMotorRunning = false;
    lastSpeedCommandMs = 0;
    TEST_SERIAL.println("Automatic slow rotation enabled for both motors");
  } else if (operation == 'h' || operation == 'H') {
    printHelp();
  } else if (operation == 'q' || operation == 'Q') {
    readState(motor1);
    readState(motor2);
  } else if (operation == '1' && count >= 2) {
    setTorque(motor1, first);
    motor1Command = constrain(first, -MAX_TEST_COMMAND, MAX_TEST_COMMAND);
    recordManualCommand();
  } else if (operation == '2' && count >= 2) {
    setTorque(motor2, first);
    motor2Command = constrain(first, -MAX_TEST_COMMAND, MAX_TEST_COMMAND);
    recordManualCommand();
  } else if ((operation == 'b' || operation == 'B') && count >= 3) {
    setTorque(motor1, first);
    setTorque(motor2, second);
    motor1Command = constrain(first, -MAX_TEST_COMMAND, MAX_TEST_COMMAND);
    motor2Command = constrain(second, -MAX_TEST_COMMAND, MAX_TEST_COMMAND);
    recordManualCommand();
  } else {
    TEST_SERIAL.println("Invalid command. Enter h for help.");
  }
}

}  // namespace WheelMotorTest

void setup() {
  using namespace WheelMotorTest;

  TEST_SERIAL.begin(115200);
  motor1Serial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR1_RX_PIN, MOTOR1_TX_PIN);
  motor2Serial.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR2_RX_PIN, MOTOR2_TX_PIN);
  delay(500);
  stopAll(false);
  printHelp();
  TEST_SERIAL.println("Starting both motors at 160 deg/s. Enter s to stop.");
  setSpeed(motor1, AUTO_TEST_SPEED);
  setSpeed(motor2, AUTO_TEST_SPEED);
  lastSpeedCommandMs = millis();
}

void loop() {
  using namespace WheelMotorTest;

  if (TEST_SERIAL.available()) {
    handleCommand(TEST_SERIAL.readStringUntil('\n'));
  }

  if (autoSpeedEnabled &&
      millis() - lastSpeedCommandMs >= SPEED_COMMAND_INTERVAL_MS) {
    setSpeed(motor1, AUTO_TEST_SPEED, false);
    setSpeed(motor2, AUTO_TEST_SPEED, false);
    lastSpeedCommandMs = millis();
  }

  if (manualMotorRunning &&
      millis() - lastNonzeroCommandMs >= COMMAND_WATCHDOG_MS) {
    TEST_SERIAL.println("[WATCHDOG] No new command; stopping both motors.");
    stopAll();
  }
}

#endif  // WHEEL_MOTOR_TEST_H
