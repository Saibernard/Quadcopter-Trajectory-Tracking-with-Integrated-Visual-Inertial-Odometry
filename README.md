# Quadcopter-Trajectory-Tracking-with-Integrated-Visual-Inertial-Odometry
This repository delves into the intricate integration of Visual-Inertial Odometry (VIO) with quadcopter trajectory planning and control. Leveraging state estimation, we devise a robust flight plan and control strategy, ensuring stable and accurate flights in GPS-denied environments.

# Autonomous Quadcopter Trajectory Tracking via Stereo Visual-Inertial Odometry (VIO)

## 1. Introduction

Throughout my journey in this course, I've progressively built on the foundations of robotic autonomy. In Project 1, I orchestrated a system where trajectories were planned and tracked accurately, leveraging the provided ground-truth states of the robot, namely its positions and velocities. By the time I embarked on Project 2, the emphasis shifted towards estimating the robot's state amidst the backdrop of noisy sensor readings, sidelining the robotic autonomy elements. In Project 3, the culmination was the integration of state estimation from Project 2 with the trajectory planning and control mechanisms established in Project 1. This method is in sync with reliable GPS-denied autonomy stacks prevalent in academic and industrial domains.

## 2. Simulator Implementation

My approach to simulating a stereo VIO-based quadcopter was innovative. An idealistic approach would synthesize camera images directly from the robot's position. However, considering hardware limitations, I adopted an alternative method using flightsim, pre-rendering features affixed on simulator obstacles and boundaries. This method bypassed the need for full frame rendering, proving to be more compute-efficient.

## 3. Main Task: Trajectory Tracking with State Estimator

The core of this project was to ensure the quadcopter's adherence to the desired trajectory without a ground-truth state. My state estimator from Project 2 played a pivotal role. I employed visual aids for better performance and had the option to adjust various sensor noise parameters for both the IMU and stereo features. One of the key learnings was the necessity to modulate trajectory or PID tuning to prevent spikes in state estimation error.

### 3.1 Fundamental Equations

Given the state \( x_t = [p_t, q_t, v_t, b_{ωt}, b_{at}]^T \):

- \( p_t \): position
- \( q_t \): orientation (quaternion format)
- And so on...

The VIO state prediction based on IMU readings is:
\[ x_{t+1} = f(x_t, u_t) \]

### 3.2 Noise Modeling

Noise in VIO is typically modeled as Gaussian, for instance, the accelerometer reading is:
\[ n_a \sim N(0, \sigma_a^2I_3) \]

Similarly, for the gyroscope:
\[ n_ω \sim N(0, \sigma_ω^2I_3) \]

## 4. Trajectory Planning and Control

### 4.1 Trajectory Representation

Using quintic splines, the trajectory can be represented as:
\[ p(t) = a_0 + a_1t + a_2t^2 + a_3t^3 + a_4t^4 + a_5t^5 \]

### 4.2 Control Law

The control input \( u \) for tracking a desired trajectory uses a PID controller:
\[ u(t) = K_pe(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt} \]

## 5. Optimization for Robust Performance

### 5.1 Parameter Tuning

Parameters were fine-tuned in `flightsim/sensors/vio_utils.py` to optimize performance.

### 5.2 Trajectory Aggressiveness

An aggressive trajectory can affect state estimation. The relationship can be described as:
\[ \Delta E = K_t \times \Delta T \]

---

**In Summary:** This project combines the principles of state estimation through Visual-Inertial Odometry with trajectory planning and control. The mathematical frameworks and formulas provide a comprehensive understanding of the methodologies involved.

