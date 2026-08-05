#ifndef MPU6050_STANDALONE_TEST_H
#define MPU6050_STANDALONE_TEST_H

#include <Arduino.h>
#include <Wire.h>

namespace MPU6050Test {

#ifndef MPU6050_SDA_PIN
#define MPU6050_SDA_PIN 10
#endif
#ifndef MPU6050_SCL_PIN
#define MPU6050_SCL_PIN 11
#endif

// Pins are supplied by the selected PlatformIO environment.
constexpr int I2C_SDA_PIN = MPU6050_SDA_PIN;
constexpr int I2C_SCL_PIN = MPU6050_SCL_PIN;
constexpr uint32_t I2C_FREQUENCY = 400000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 100;
constexpr size_t CALIBRATION_SAMPLES = 1000;
constexpr uint32_t CALIBRATION_INTERVAL_MS = 10;

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
bool continuousOutputEnabled = false;

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

void calibrateMPU6050() {
  if (mpuAddress == 0) {
    Serial.println("[ERROR] MPU6050 is not initialized");
    return;
  }

  Serial.println("MPU calibration starts: keep the sensor completely still and level.");
  Serial.printf("Collecting %u samples (about %lu seconds)...\n",
                static_cast<unsigned>(CALIBRATION_SAMPLES),
                static_cast<unsigned long>(
                    CALIBRATION_SAMPLES * CALIBRATION_INTERVAL_MS / 1000));

  int64_t accelSum[3] = {0, 0, 0};
  int64_t gyroSum[3] = {0, 0, 0};
  int64_t temperatureSum = 0;
  size_t collected = 0;
  size_t failed = 0;

  while (collected < CALIBRATION_SAMPLES) {
    const uint32_t sampleStart = millis();
    uint8_t data[14];
    if (readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) {
      accelSum[0] += combineBytes(data[0], data[1]);
      accelSum[1] += combineBytes(data[2], data[3]);
      accelSum[2] += combineBytes(data[4], data[5]);
      temperatureSum += combineBytes(data[6], data[7]);
      gyroSum[0] += combineBytes(data[8], data[9]);
      gyroSum[1] += combineBytes(data[10], data[11]);
      gyroSum[2] += combineBytes(data[12], data[13]);
      ++collected;
      if (collected % 100 == 0) {
        Serial.printf("Calibration progress: %u/%u\n",
                      static_cast<unsigned>(collected),
                      static_cast<unsigned>(CALIBRATION_SAMPLES));
      }
    } else {
      ++failed;
      if (failed >= 100) {
        Serial.println("[ERROR] Too many MPU6050 read failures; calibration aborted");
        return;
      }
    }

    const uint32_t elapsed = millis() - sampleStart;
    if (elapsed < CALIBRATION_INTERVAL_MS) {
      delay(CALIBRATION_INTERVAL_MS - elapsed);
    }
  }

  const double count = static_cast<double>(CALIBRATION_SAMPLES);
  const double axRaw = accelSum[0] / count;
  const double ayRaw = accelSum[1] / count;
  const double azRaw = accelSum[2] / count;
  const double gxRaw = gyroSum[0] / count;
  const double gyRaw = gyroSum[1] / count;
  const double gzRaw = gyroSum[2] / count;
  const double expectedZRaw = azRaw >= 0.0 ? 16384.0 : -16384.0;

  Serial.println("=== MPU6050 1000-sample calibration result ===");
  Serial.printf("Average accel raw: X=%.2f Y=%.2f Z=%.2f\n",
                axRaw, ayRaw, azRaw);
  Serial.printf("Average accel [g]: X=%+.6f Y=%+.6f Z=%+.6f\n",
                axRaw / 16384.0, ayRaw / 16384.0, azRaw / 16384.0);
  Serial.printf("Accel bias raw (level, Z vertical): X=%.2f Y=%.2f Z=%.2f\n",
                axRaw, ayRaw, azRaw - expectedZRaw);
  Serial.printf("Gyro bias raw: X=%.2f Y=%.2f Z=%.2f\n",
                gxRaw, gyRaw, gzRaw);
  Serial.printf("Gyro bias [deg/s]: X=%+.6f Y=%+.6f Z=%+.6f\n",
                gxRaw / 131.0, gyRaw / 131.0, gzRaw / 131.0);
  Serial.printf("Average temperature: %.2f C | failed reads: %u\n",
                temperatureSum / count / 340.0 + 36.53,
                static_cast<unsigned>(failed));
  Serial.println("Use corrected value = measured value - reported bias.");
}

void handleCommand(String command) {
  command.trim();
  if (command.equalsIgnoreCase("c")) {
    calibrateMPU6050();
  } else if (command.equalsIgnoreCase("r")) {
    printMeasurement();
  } else if (command.equalsIgnoreCase("m")) {
    continuousOutputEnabled = !continuousOutputEnabled;
    Serial.printf("Continuous MPU output: %s\n",
                  continuousOutputEnabled ? "ON" : "OFF");
  } else if (!command.isEmpty()) {
    Serial.println("Commands: c = calibrate, r = read once, m = toggle continuous output");
  }
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
  Serial.println("Commands: c = calibrate, r = read once, m = toggle continuous output");
}

void loop() {
  using namespace MPU6050Test;

  static String command;
  while (Serial.available()) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r' || ch == '\n') {
      handleCommand(command);
      command = "";
    } else if (ch == '\b' || ch == 0x7F) {
      if (!command.isEmpty()) command.remove(command.length() - 1);
    } else if (isPrintable(ch)) {
      command += ch;
    }
  }

  if (mpuAddress == 0) {
    delay(1000);
    return;
  }

  const uint32_t now = millis();
  if (continuousOutputEnabled && now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    printMeasurement();
  }
}

#endif  // MPU6050_STANDALONE_TEST_H
