# Quadcopter-Trajectory-Tracking-with-Integrated-Visual-Inertial-Odometry
This repository delves into the intricate integration of Visual-Inertial Odometry (VIO) with quadcopter trajectory planning and control. Leveraging state estimation, we devise a robust flight plan and control strategy, ensuring stable and accurate flights in GPS-denied environments.

# Autonomous Quadcopter Trajectory Tracking via Stereo Visual-Inertial Odometry (VIO)

## 1. Introduction

Throughout my studies and explorations in autonomous systems, I've sequentially approached different facets of quadcopter autonomy. Initially, I focused on the precision of trajectory planning and tracking, ensuring that given the robot's ground-truth states, like positions and velocities, we could attain smooth and predictable motion paths.

Subsequently, I ventured into the realm of state estimation. Here, I grappled with the challenge of determining the robot's state using just noisy sensor measurements. This phase was devoid of any planning or control, emphasizing purely on how sensor data can be accurately interpreted to surmise the robot's exact state.

Marrying these two explorations, my recent endeavor is to fuse state estimation techniques with my previously established planning and control mechanisms. The aspiration is simple yet challenging: to harness the power of Visual Inertial Odometry (VIO) and use it in-the-loop, enabling the quadcopter to autonomously estimate its state and make planning and control decisions accordingly. This technique, as showcased by several academic and industrial benchmarks, has emerged as a robust solution for implementing a GPS-denied autonomy stack.

Once this autonomous state estimation capability is achieved, the path forward beckons the integration of a mapping solution. This would empower the quadcopter to intuitively sense its surroundings, detecting obstacles via its onboard sensors. The ultimate goal is to devise an autonomous quadcopter that not only understands its spatial positioning but can also safely navigate environments by dynamically responding to obstacles.

## 2. Simulator Implementation

My approach to simulating a stereo VIO-based quadcopter was innovative. An idealistic approach would synthesize camera images directly from the robot's position. However, considering hardware limitations, I adopted an alternative method using flightsim, pre-rendering features affixed on simulator obstacles and boundaries. This method bypassed the need for full frame rendering, proving to be more compute-efficient.

## 3. Main Task: Trajectory Tracking with State Estimator

The core of this project was to ensure the quadcopter's adherence to the desired trajectory without a ground-truth state. My state estimator from Project 2 played a pivotal role. I employed visual aids for better performance and had the option to adjust various sensor noise parameters for both the IMU and stereo features. One of the key learnings was the necessity to modulate trajectory or PID tuning to prevent spikes in state estimation error.

### 3.1 Fundamental Equations

Given the state x(t) = [p(t), q(t), v(t), bω(t), ba(t)],

- p(t): position
- q(t): orientation (quaternion format)
- vtvt​: velocity
- bωtbωt​: gyroscope bias
- batbat​: accelerometer bias

The VIO state prediction based on IMU readings is:

x(t+1) = f(x(t), u(t))

where ut=[ωt,at]Tut​=[ωt​,at​]T is the IMU readings of angular velocity and linear acceleration.

### 3.2 Noise Modeling

Noise in VIO is typically modeled as Gaussian, for instance, the accelerometer reading is:

na = N(0, σa^2*I3)

Similarly, for the gyroscope:

nω = N(0, σω^2*I3)

## 4. Trajectory Planning and Control

### 4.1 Trajectory Representation

Using quintic splines, the trajectory can be represented as:

p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

### 4.2 Control Law

The control input \( u \) for tracking a desired trajectory uses a PID controller:

u(t) = Kp*e(t) + Ki*∫e(t) dt + Kd*de(t)/dt

## 5. Optimization for Robust Performance

### 5.1 Parameter Tuning

Parameters were fine-tuned in `flightsim/sensors/vio_utils.py` to optimize performance.

### 5.2 Trajectory Aggressiveness

An aggressive trajectory can affect state estimation. The relationship can be described as:
ΔE = Kt * ΔT

---

**In Summary:** This project combines the principles of state estimation through Visual-Inertial Odometry with trajectory planning and control. The mathematical frameworks and formulas provide a comprehensive understanding of the methodologies involved.

