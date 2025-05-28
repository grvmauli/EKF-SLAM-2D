# EKF SLAM 2D (C++)

This project implements **2D Extended Kalman Filter Simultaneous Localization and Mapping (EKF SLAM)** using C++. It simulates a robot navigating a 2D environment with landmarks, predicting its motion, updating its state with noisy sensor observations, and visualizing the result in real-time.

## ✨ Features

- Extended Kalman Filter for 2D SLAM
- Simulated robot control and observations
- Visualized robot path, landmarks, and uncertainty ellipses
- Modular C++ implementation using Eigen for linear algebra

## 📁 Project Structure

ekf_slam_2d/
├── main.cpp # Main application loop
├── ekf_slam.cpp/h # EKF SLAM implementation
├── simulator.cpp/h # Simulator for robot motion and landmark observations
├── visualize.cpp/h # Simple real-time visualization
├── utils.h # Utility functions
└── README.md # This file


## 🚀 Getting Started

### Prerequisites

- C++17 compiler (e.g., `g++`)
- [Eigen3](https://eigen.tuxfamily.org/) installed (for linear algebra)

### Build

```bash
g++ main.cpp ekf_slam.cpp simulator.cpp visualize.cpp -I. -I /usr/include/eigen3 -std=c++17 -o ekf_slam
./ekf_slam
```
## 🧪 Example Output
Robot trajectory (true vs estimated)

Landmark estimates with uncertainty ellipses

Real-time visualization of the SLAM process

## 🧱 Dependencies
Eigen3 — for matrix and vector operations

Install Eigen on Ubuntu:
```bash
sudo apt install libeigen3-dev
```
## 🤖 Future Improvements
Support for real sensor datasets (e.g., MIT robotics data)

Add GUI with OpenCV or Qt

3D SLAM extension

Landmark initialization from unknown map

## 🙌 Acknowledgments
Inspired by robotics lectures, Probabilistic Robotics, and SLAM course materials.
