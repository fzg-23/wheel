#ifndef MPU6050_STANDALONE_TEST_H
#define MPU6050_STANDALONE_TEST_H

#include <Arduino.h>
#include <Wire.h>

namespace MPU6050Test {

#ifndef MPU6050_SDA_PIN
#define MPU6050_SDA_PIN 8
#endif
#ifndef MPU6050_SCL_PIN
#define MPU6050_SCL_PIN 17
#endif

// Pins are supplied by the selected PlatformIO environment.
constexpr int I2C_SDA_PIN = MPU6050_SDA_PIN;
constexpr int I2C_SCL_PIN = MPU6050_SCL_PIN;
constexpr uint32_t I2C_FREQUENCY = 400000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 100;

constexpr uint8_t ADDRESS_LOW = 0x68;
constexpr uint8_t ADDRESS_HIGH = 0x69;
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

uint8_t mpuAddress = 0;
uint32_t lastSampleMs = 0;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readRegisters(uint8_t reg, uint8_t* data, size_t length) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  const size_t received = Wire.requestFrom(
      mpuAddress, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
  if (received != length) return false;

  for (size_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool addressAcknowledges(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool findMPU6050() {
  if (addressAcknowledges(ADDRESS_LOW)) mpuAddress = ADDRESS_LOW;
  else if (addressAcknowledges(ADDRESS_HIGH)) mpuAddress = ADDRESS_HIGH;
  else return false;

  uint8_t whoAmI = 0;
  if (!readRegisters(REG_WHO_AM_I, &whoAmI, 1)) return false;
  Serial.printf("MPU6050 ACK at 0x%02X, WHO_AM_I=0x%02X\n",
                mpuAddress, whoAmI);
  return (whoAmI & 0x7E) == 0x68;
}

bool initializeMPU6050() {
  // Wake up and use the X-axis gyro as the clock source.
  if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
  delay(100);
  // 1 kHz / (1 + 9) = 100 Hz sample rate, 44 Hz DLPF.
  return writeRegister(REG_SMPLRT_DIV, 9) &&
         writeRegister(REG_CONFIG, 0x03) &&
         writeRegister(REG_GYRO_CONFIG, 0x00) &&   // +/-250 deg/s
         writeRegister(REG_ACCEL_CONFIG, 0x00);    // +/-2 g
}

int16_t combineBytes(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

void printMeasurement() {
  uint8_t data[14];
  if (!readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) {
    Serial.println("[ERROR] Failed to read MPU6050 data");
    return;
  }

  const float ax = combineBytes(data[0], data[1]) / 16384.0f;
  const float ay = combineBytes(data[2], data[3]) / 16384.0f;
  const float az = combineBytes(data[4], data[5]) / 16384.0f;
  const float temperature = combineBytes(data[6], data[7]) / 340.0f + 36.53f;
  const float gx = combineBytes(data[8], data[9]) / 131.0f;
  const float gy = combineBytes(data[10], data[11]) / 131.0f;
  const float gz = combineBytes(data[12], data[13]) / 131.0f;

  Serial.printf("acc[g] X:%+.3f Y:%+.3f Z:%+.3f  ", ax, ay, az);
  Serial.printf("gyro[deg/s] X:%+.2f Y:%+.2f Z:%+.2f  T:%.2f C\n",
                gx, gy, gz, temperature);
}

}  // namespace MPU6050Test

void setup() {
  using namespace MPU6050Test;

  Serial.begin(115200);
  delay(1000);
  Serial.printf("\nMPU6050 I2C test: SDA=GPIO%d, SCL=GPIO%d\n",
                I2C_SDA_PIN, I2C_SCL_PIN);

  if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY)) {
    Serial.println("[ERROR] Failed to start I2C controller");
    return;
  }
  Wire.setTimeOut(50);

  if (!findMPU6050()) {
    Serial.println("[ERROR] MPU6050 not found at 0x68 or 0x69");
    Serial.println("Check 3V3, GND, SDA, SCL and the AD0 pin.");
    return;
  }
  if (!initializeMPU6050()) {
    Serial.println("[ERROR] MPU6050 register initialization failed");
    mpuAddress = 0;
    return;
  }

  Serial.println("MPU6050 initialized successfully");
}

void loop() {
  using namespace MPU6050Test;

  if (mpuAddress == 0) {
    delay(1000);
    return;
  }

  const uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    printMeasurement();
  }
}

#endif  // MPU6050_STANDALONE_TEST_H
