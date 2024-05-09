#include <Wire.h>
#include <MadgwickAHRS.h>
#include <MPU6050.h>

MPU6050 mpu;

// Kalman filter variables for each axis
float q_angle = 0.001;
float q_bias = 0.003;
float r_measure = 0.03;
float angleX, angleY, angleZ; // Filtered angles for each axis
float P[2][2] = {{1, 0}, {0, 1}}; // Error covariance matrix
float K[2]; // Kalman gain

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true); // request a total of 14 registers
  int16_t AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  int16_t AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  int16_t AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  int16_t Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  int16_t GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  int16_t GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  int16_t GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // Calculate angles using accelerometer data
  angleX = atan2(AcY, AcZ) * RAD_TO_DEG;
  angleY = atan(-AcX / sqrt(AcY * AcY + AcZ * AcZ)) * RAD_TO_DEG;

  // Calculate gyro drift
  float gyroXrate = GyX / 131.0; // Convert raw gyro readings to deg/s
  float gyroYrate = GyY / 131.0;

  // Apply Kalman filter
  float dt = 0.01; // Time step in seconds
  angleX += dt * (gyroXrate - q_bias);
  angleY += dt * (gyroYrate - q_bias);

  // Update error covariance matrix
  float Pdot[4] = {0, 0, 0, 0};
  Pdot[0] = q_angle - P[0][1] - P[1][0];
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = q_bias - P[1][1];
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;

  // Calculate Kalman gain
  float S = P[0][0] + r_measure;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // Update angles
  float y = angleX - angleX;
  angleX += K[0] * y;
  angleY += K[1] * y;

  // Update bias
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  // Print the filtered sensor data
  Serial.print("Filtered Angle X: ");
  Serial.print(angleX);
  Serial.print(" | Filtered Angle Y: ");
  Serial.println(angleY);

  delay(100);
}
