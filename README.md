# Position Estimator

Use a Kalman filter for Gyroscope, Accelerometer and Magnetometer of ICM20948.

Project on ESP32.

Try to recover path.

## TODO

[X] - Write driver for ICM20948
[X] - Add madgwick (Kalman variant) fusion algorithm
[ ] - Test computation
[ ] - Compute acceleration from attitude
[ ] - Integrate to get the position estimation
[ ] - Code wifi driver to get data
[ ] - Write packets to get data
[ ] - Calibrate Magnetometer, Accelerometer
