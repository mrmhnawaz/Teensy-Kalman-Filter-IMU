Teensy Kalman Madgwick Filter IMU
Welcome to the Teensy Kalman Filter IMU project repository! This project implements a Kalman filter on a Teensy microcontroller to process data from an MPU6050 accelerometer and gyroscope sensor for inertial measurement unit (IMU) applications.

Overview
This project aims to demonstrate the implementation of a Kalman filter for sensor fusion on the Teensy platform. 
The Kalman filter combines accelerometer and gyroscope data from the MPU6050 sensor to estimate the orientation angles of the IMU. 
This filtered data can be used for various applications such as stabilization, orientation tracking, and navigation in robotics and unmanned systems.

Features
Kalman filter implementation for sensor fusion of accelerometer and gyroscope data.
Real-time estimation of roll and pitch angles based on sensor readings.
Adjustable filter parameters for fine-tuning performance and response.
Serial communication for outputting filtered angle data to a host computer for monitoring and analysis.

Hardware Requirements
Teensy 4.0 microcontroller board
MPU6050 accelerometer and gyroscope sensor module
Connecting wires

Software Requirements
Arduino IDE for Teensy development
MPU6050 library for interfacing with the sensor
Kalman filter algorithm implementation (included in the sketch)

Setup Instructions
Connect the MPU6050 sensor to the Teensy microcontroller as per the wiring diagram provided in the documentation.
Install the necessary libraries (MPU6050) in the Arduino IDE.
Upload the provided Arduino sketch (Teensy_Kalman_MadgwickFilter_IMU.ino) to the Teensy board.
Open the serial monitor in the Arduino IDE to view the filtered angle data in real-time.

Repository Structure
src: Contains the Arduino sketch (Teensy_Kalman_MadgwickFilter_IMU.ino) implementing the Kalman filter on the Teensy platform.
docs: Documentation files including wiring diagrams, setup instructions, and additional resources.

Usage
Power on the Teensy board and observe the serial monitor output for filtered angle data.
Experiment with different filter parameters and sensor configurations to optimize performance for your specific application.
Integrate the filtered angle data into your project for stabilization, orientation tracking, or navigation purposes.

Contributions
Contributions to this project are welcome! If you have any suggestions, improvements, or bug fixes, please feel free to open an issue or submit a pull request.

Mohammed Hussain Nawaz: https://github.com/mrmhnawaz


License
This project is licensed under the MIT License.
