#ifndef LU9685_HIP_SERVO_TEST_H
#define LU9685_HIP_SERVO_TEST_H

/*
 * Ported from /home/fzg/桌面/esp32_lu9685_270/src/main.cpp.
 * ESP32-S3 -> LU9685-20CU -> 9imod 270-degree servos on channels 0 and 2.
 *
 * Wiring:
 *   GPIO8 -> SDA
 *   GPIO9 -> SCL
 *   GND   -> LU9685 GND and servo-power GND
 */

#include <Arduino.h>
#include <Wire.h>

namespace LU9685Test {

#define TEST_SERIAL Serial0

constexpr int LU9685_SDA_PIN = 8;
constexpr int LU9685_SCL_PIN = 9;
constexpr uint32_t LU9685_I2C_CLOCK = 100000;
constexpr uint8_t LU9685_I2C_ADDRESS = 0x00;
constexpr uint8_t SERVO_CHANNELS[] = {0, 2};
constexpr float SERVO_RANGE_DEG = 180.0f;

bool resetLU9685() {
  Wire.beginTransmission(LU9685_I2C_ADDRESS);
  Wire.write(0xFB);
  Wire.write(0xFB);
  return Wire.endTransmission() == 0;
}

bool setLU9685Angle(uint8_t channel, uint8_t boardAngle) {
  if (channel > 19 || boardAngle > 180) return false;

  Wire.beginTransmission(LU9685_I2C_ADDRESS);
  Wire.write(channel);
  Wire.write(boardAngle);
  return Wire.endTransmission() == 0;
}

bool physicalToLU9685Angle(float physicalAngle, uint8_t* boardAngle) {
  if (physicalAngle < 0.0f || physicalAngle > SERVO_RANGE_DEG) return false;

  *boardAngle = static_cast<uint8_t>(lroundf(physicalAngle));
  return true;
}

bool setServoAngle(uint8_t channel, float physicalAngle) {
  uint8_t boardAngle = 0;
  if (!physicalToLU9685Angle(physicalAngle, &boardAngle)) return false;

  if (!setLU9685Angle(channel, boardAngle)) {
    TEST_SERIAL.printf(
        "I2C write failed on channel %u. Check SDA/SCL, power, GND, and address.\n",
        channel);
    return false;
  }

  TEST_SERIAL.printf("Servo CH%u: %.1f deg -> LU9685 raw angle: %u\n",
                     channel, physicalAngle, boardAngle);
  return true;
}

bool setAllServoAngles(float physicalAngle) {
  uint8_t boardAngle = 0;
  if (!physicalToLU9685Angle(physicalAngle, &boardAngle)) return false;

  for (uint8_t channel : SERVO_CHANNELS) {
    if (!setLU9685Angle(channel, boardAngle)) {
      TEST_SERIAL.printf(
          "I2C write failed on channel %u. Check SDA/SCL, power, GND, and address.\n",
          channel);
      return false;
    }
  }

  TEST_SERIAL.printf("Servos CH0/CH2: %.1f deg -> LU9685 raw angle: %u\n",
                     physicalAngle, boardAngle);
  return true;
}

void printHelp() {
  TEST_SERIAL.println("Commands:");
  TEST_SERIAL.println("  <angle>         set channels 0 and 2, raw angle 0..180");
  TEST_SERIAL.println("  <ch> <angle>    set one channel, ch 0 or 2, raw angle 0..180");
  TEST_SERIAL.println("Examples: 135, 0 90, 2 180");
}

void handleCommand(String text) {
  text.trim();
  if (text.isEmpty()) return;

  if (text == "?" || text.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }

  char* end = nullptr;
  const float firstValue = strtof(text.c_str(), &end);
  if (end == text.c_str()) {
    TEST_SERIAL.println("Invalid input.");
    printHelp();
    return;
  }

  while (end != nullptr && *end == ' ') ++end;

  if (end == nullptr || *end == '\0') {
    if (!setAllServoAngles(firstValue)) {
      TEST_SERIAL.println("Out of range. Enter angle 0..180.");
    }
    return;
  }

  char* angleEnd = nullptr;
  const float physicalAngle = strtof(end, &angleEnd);
  while (angleEnd != nullptr && *angleEnd == ' ') ++angleEnd;

  if (angleEnd == end || (angleEnd != nullptr && *angleEnd != '\0')) {
    TEST_SERIAL.println("Invalid input.");
    printHelp();
    return;
  }

  const int channel = static_cast<int>(firstValue);
  if ((channel != 0 && channel != 2) || firstValue != channel) {
    TEST_SERIAL.println("Invalid channel. Use channel 0 or 2.");
    return;
  }

  if (!setServoAngle(static_cast<uint8_t>(channel), physicalAngle)) {
    TEST_SERIAL.println("Out of range. Enter angle 0..180.");
  }
}

}  // namespace LU9685Test

void setup() {
  using namespace LU9685Test;

  TEST_SERIAL.begin(115200);
  Wire.begin(LU9685_SDA_PIN, LU9685_SCL_PIN);
  Wire.setClock(LU9685_I2C_CLOCK);
  delay(1500);

  TEST_SERIAL.println();
  TEST_SERIAL.println("ESP32-S3 LU9685 I2C command controller ready");
  TEST_SERIAL.printf("I2C address: 0x%02X, SDA: GPIO%d, SCL: GPIO%d\n",
                     LU9685_I2C_ADDRESS, LU9685_SDA_PIN, LU9685_SCL_PIN);
  TEST_SERIAL.println("No command is sent on boot; observe the servo initial position now.");
  printHelp();
  TEST_SERIAL.print("> ");
}

void loop() {
  using namespace LU9685Test;
  static String line;

  while (TEST_SERIAL.available()) {
    const char ch = static_cast<char>(TEST_SERIAL.read());

    if (ch == '\r' || ch == '\n') {
      TEST_SERIAL.println();
      handleCommand(line);
      line = "";
      TEST_SERIAL.print("> ");
      continue;
    }

    if (ch == '\b' || ch == 0x7F) {
      if (!line.isEmpty()) {
        line.remove(line.length() - 1);
        TEST_SERIAL.print("\b \b");
      }
      continue;
    }

    if (isPrintable(ch)) {
      line += ch;
      TEST_SERIAL.write(ch);
    }
  }
}

#endif  // LU9685_HIP_SERVO_TEST_H
