#ifndef WHEEL_MOTOR_SOURCE_HARDWARE_TEST_H
#define WHEEL_MOTOR_SOURCE_HARDWARE_TEST_H

#include <Arduino.h>

namespace WheelMotorSourceTest {

#define TEST_SERIAL Serial0

// Original WBR hardware wiring and communication settings.
constexpr int RS485_DE_RE_PIN = 16;
constexpr int RS485_TX_PIN = 17;
constexpr int RS485_RX_PIN = 18;
constexpr uint32_t RS485_BAUD = 115200;

constexpr uint8_t RIGHT_MOTOR_ID = 1;
constexpr int16_t MAX_TEST_COMMAND = 100;
constexpr uint32_t WATCHDOG_MS = 1000;
constexpr uint32_t RESPONSE_TIMEOUT_US = 3000;

constexpr uint8_t FRAME_HEAD = 0x3E;
constexpr uint8_t TORQUE_COMMAND = 0xA1;
constexpr uint8_t READ_STATE2_COMMAND = 0x9C;
constexpr size_t STATE2_RESPONSE_SIZE = 13;

HardwareSerial rs485(1);
int16_t rightCommand = 0;
uint32_t lastCommandMs = 0;

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

void clearRx() {
  while (rs485.available()) rs485.read();
}

void sendFrame(const uint8_t* frame, size_t length) {
  clearRx();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  rs485.write(frame, length);
  rs485.flush();
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

bool readExact(uint8_t* response, size_t length) {
  const uint32_t start = micros();
  size_t received = 0;

  while (received < length) {
    while (rs485.available() && received < length) {
      response[received++] = static_cast<uint8_t>(rs485.read());
    }
    if (micros() - start >= RESPONSE_TIMEOUT_US) return false;
  }
  return true;
}

bool decodeState(const uint8_t* response, uint8_t expectedId,
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

bool transact(const uint8_t* frame, size_t length, uint8_t motorId,
              MotorState& state) {
  uint8_t response[STATE2_RESPONSE_SIZE];
  sendFrame(frame, length);

  if (!readExact(response, sizeof(response))) {
    TEST_SERIAL.printf("[ERROR] motor %u response timeout\n", motorId);
    return false;
  }
  if (!decodeState(response, motorId, state)) {
    TEST_SERIAL.printf("[ERROR] motor %u invalid response\n", motorId);
    return false;
  }
  return true;
}

void printState(uint8_t motorId, const MotorState& state) {
  TEST_SERIAL.printf(
      "Motor %u | temp=%d C | iqRaw=%d | speed=%.1f deg/s | encoder=%u\n",
      motorId, state.temperatureC, state.iqRaw,
      state.speedRaw / 10.0f, state.encoder);
}

bool setTorque(uint8_t motorId, int requestedCommand, bool print = true) {
  const int16_t command = static_cast<int16_t>(
      constrain(requestedCommand, -MAX_TEST_COMMAND, MAX_TEST_COMMAND));

  uint8_t frame[8] = {
      FRAME_HEAD, TORQUE_COMMAND, motorId, 2, 0,
      static_cast<uint8_t>(command & 0xFF),
      static_cast<uint8_t>((command >> 8) & 0xFF), 0};
  frame[4] = checksum(frame, 4);
  frame[7] = checksum(frame + 5, 2);

  MotorState state;
  const bool ok = transact(frame, sizeof(frame), motorId, state);
  if (ok && print) {
    TEST_SERIAL.printf("Torque command=%d | ", command);
    printState(motorId, state);
  }
  return ok;
}

bool readState(uint8_t motorId) {
  uint8_t frame[5] = {FRAME_HEAD, READ_STATE2_COMMAND, motorId, 0, 0};
  frame[4] = checksum(frame, 4);

  MotorState state;
  const bool ok = transact(frame, sizeof(frame), motorId, state);
  if (ok) printState(motorId, state);
  return ok;
}

void stopAll() {
  setTorque(RIGHT_MOTOR_ID, 0, false);
  rightCommand = 0;
  TEST_SERIAL.println("STOP: connected motor ID 1 command is zero");
}

void printHelp() {
  TEST_SERIAL.println("\n=== Original WBR wheel-motor hardware test ===");
  TEST_SERIAL.println("UART1 RX=18 TX=17, DE/RE=16, 115200 baud, 8N1");
  TEST_SERIAL.printf("Torque command limit: -%d..%d; watchdog: %lu ms\n",
                     MAX_TEST_COMMAND, MAX_TEST_COMMAND,
                     static_cast<unsigned long>(WATCHDOG_MS));
  TEST_SERIAL.println("1 <value>          connected motor ID 1, e.g. 1 10");
  TEST_SERIAL.println("q 1                read connected motor state");
  TEST_SERIAL.println("s                  emergency stop");
  TEST_SERIAL.println("h                  show help");
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
    stopAll();
  } else if (operation == 'h' || operation == 'H') {
    printHelp();
  } else if (operation == '1' && count >= 2) {
    setTorque(RIGHT_MOTOR_ID, first);
    rightCommand = constrain(first, -100, 100);
    lastCommandMs = millis();
  } else if ((operation == 'q' || operation == 'Q') && count >= 2 &&
             first == RIGHT_MOTOR_ID) {
    readState(RIGHT_MOTOR_ID);
  } else {
    TEST_SERIAL.println("Invalid command; enter h for help");
  }
}

}  // namespace WheelMotorSourceTest

void setup() {
  using namespace WheelMotorSourceTest;

  TEST_SERIAL.begin(115200);
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  rs485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  delay(500);

  stopAll();
}

void loop() {
  using namespace WheelMotorSourceTest;

  if (TEST_SERIAL.available()) {
    handleCommand(TEST_SERIAL.readStringUntil('\n'));
  }

  if (rightCommand != 0 && millis() - lastCommandMs >= WATCHDOG_MS) {
    TEST_SERIAL.println("[WATCHDOG] command timeout");
    stopAll();
  }
}

#endif  // WHEEL_MOTOR_SOURCE_HARDWARE_TEST_H
