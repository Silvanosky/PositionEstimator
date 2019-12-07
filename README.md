# Position Estimator

Use a Kalman filter for Gyroscope, Accelerometer and Magnetometer of ICM20948.

Project on ESP32.

Try to recover path.

## TODO

- [X] Write driver for ICM20948
- [X] Add madgwick (Kalman variant) fusion algorithm
- [ ] Test computation
- [ ] Compute acceleration from attitude
- [ ] Integrate to get the position estimation
- [ ] Code wifi driver to get data
- [ ] Write packets to get data
- [ ] Calibrate Magnetometer, Accelerometer

Python:

- [ ] Wi-Fi module
    - [ ] Connect via Wi-Fi
    - [ ] Receive data
    - [ ] Build Graph
- [X] Implement Graph class
    - [X] Display points
    - [X] Log points
