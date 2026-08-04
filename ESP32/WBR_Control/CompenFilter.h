#ifndef CompendFiter_H
#define CompendFiter_H

#include <ArduinoEigenDense.h>
#include "Params.h"

class CompenFilter {
private:
  Properties properties;
  float K_compen;                       // Compensation Gain: weight to balance between gyro and accelerometer data
  float acc_angle;                      // Accelerometer-derived angle
  float gyro_angle;                     // Gyroscope-derived angle
  bool is_first;                        // Flag to check if this is the first update (for initialization)
  Eigen::Matrix<float, 4, 1> x_compen;  // Estimated state vector [angle, angular velocity, linear velocity, rotation rate]
  Eigen::Matrix<float, 8, 1> z;         // Measurement vector

  // Set measurement vector `z` with new data
  void setMeasurement(const Eigen::Matrix<float, 8, 1> &z_) {
    z = z_;
  }

  // Update the state based on current measurements
  void update() {
    acc_angle = atan(-z(0) / z(2));  // Compute angle using accelerometer data
    gyro_angle += dt * z(4);         // Integrate gyroscope angular velocity over time

    if (is_first) {
      // Initialize state with first measurement
      x_compen(0) = acc_angle;  // Initial angle from accelerometer
      gyro_angle = acc_angle;

      x_compen(1) = z(4);  // Initial angular velocity
      x_compen(2) = 0;     // Initial linear velocity
      x_compen(3) = 0;     // Initial rotation rate
      is_first = false;    // Mark initialization as complete
    } else {
      // Update state using complementary filter logic
      x_compen(0) = K_compen * gyro_angle + (1.f - K_compen) * acc_angle;  // Fusion of gyro and accel angles
      x_compen(1) = z(4);                                                  // Angular velocity from measurement
      x_compen(2) = properties.R / 2 * (z(7) - z(6) + 2 * x_compen(1));    // Compute linear velocity
      x_compen(3) = -(z(6) + z(7)) * properties.R / (2 * properties.L);    // Compute rotation rate
    }
  }

public:
  // Constructor: Initialize filter parameters and state
  CompenFilter(const Properties &properties_)
    : K_compen(0.98f), properties(properties_) {
    x_compen.setZero();  // Set all state values to zero
    z.setZero();         // Set all measurements to zero
    acc_angle = 0;       // Reset accelerometer angle
    gyro_angle = 0;      // Reset gyroscope angle
    is_first = true;     // Mark as first update
  }

  // Print all estimation data (state vector and measurement vector) to Serial
  void printAllEstimationData() {
    Serial.println("x_compen:");
    for (int i = 0; i < x_compen.rows(); ++i) {
      Serial.print(x_compen(i, 0), 6);  // Print state values with 6 decimal places
      Serial.println();
    }

    Serial.println("z:");
    for (int i = 0; i < z.rows(); ++i) {
      Serial.print(z(i, 0), 6);  // Print measurement values with 6 decimal places
      Serial.println();
    }
  }

  // Estimate the state vector based on the current measurement
  bool estimate_state(Eigen::Matrix<float, 4, 1> &x_, const Eigen::Matrix<float, 8, 1> &z_) {
    setMeasurement(z_);  // Update measurement vector
    update();            // Update state based on measurements
    x_ = x_compen;       // Return the updated state
    return true;         // Indicate successful estimation
  }

  // Reset the estimator completely (including angles and first-time flag)
  void reset_estimator() {
    acc_angle = 0;       // Reset accelerometer angle
    gyro_angle = 0;      // Reset gyroscope angle
    is_first = true;     // Mark as first update
    x_compen.setZero();  // Reset state vector
    z.setZero();         // Reset measurement vector
  }
};

#endif  // CompendFiter_H
