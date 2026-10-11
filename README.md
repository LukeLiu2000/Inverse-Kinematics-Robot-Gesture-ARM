# Inverse-Kinematics-Robot-Gesture-ARM
A course lab project.

## Demo Videos

### Main Demo Video
<video width="400" controls>
  <source src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/b293a466d7596babbb78964984078bba_raw.mp4" type="video/mp4">
</video>
<img src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/VideoCapture.png" alt="Robot arm setup" width="800"> 
[Download main demo video](https://github.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/raw/main/b293a466d7596babbb78964984078bba_raw.mp4)

### Linear Motion Demo
<video width="400" controls>
  <source src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/直线移动.mp4" type="video/mp4">
</video>

[Download linear motion video](https://github.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/raw/main/videos/直线移动.mp4)

### WeChat Video Demo
<video width="400" controls>
  <source src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/微信视频2026-04-29_043447_006.mp4" type="video/mp4">
</video>

[Download WeChat video](https://github.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/raw/main/微信视频2026-04-29_043447_006.mp4)

## Images

### Setup and Control
<img src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/arm.jpg" alt="Robot arm setup" width="250"> <img src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/control.png" alt="Control interface" width="250"> <img src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/demo.png" alt="Demo screenshot" width="250">

### Technical Diagrams
<img src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/coordinate.png" alt="Coordinate system" width="300"> <img src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/Spherical_with_grid.svg.png" alt="Spherical coordinate system" width="300">

### Trajectory Visualization
<img src="https://raw.githubusercontent.com/LukeLiu2000/Inverse-Kinematics-Robot-Gesture-ARM/main/videos/trajectory.png" alt="Trajectory visualization" width="400">

## Project Overview

For our project, we also designed an alternative robotic arm solution. This design features a robotic arm capable of pulling the robot forward. While it offers greater flexibility as an advantage, it requires more careful control.

### Kinematics Theory

We start from the planar, or two-dimensional, case. In this setup, the arm is modeled as a two-link manipulator with lengths L₁ and L₂. We define two key angles, α and β, which determine the configuration of the arm.

Using forward kinematics, we can compute the end-effector position (x, y) from these angles. More importantly, for control, we solve the inverse kinematics problem: given a target position (x, y), we compute the required angles.

Once the planar control is established, we extend the system to three-dimensional space. The key idea is that the arm itself still operates in a plane, but we add an additional degree of freedom: rotation about the vertical axis.

To handle this, we introduce a cylindrical coordinate representation. Instead of directly controlling (x, y, z), we convert it into (ρ, θ, z), where ρ = √(x² + y²) and θ = atan2(y, x). The angle θ directly controls the rotation servo.

Then, the remaining problem becomes a planar one again: we use (ρ, z) as inputs to the same inverse kinematics we derived earlier to compute α and β.

So overall, the control pipeline is: we input a Cartesian position (x, y, z), convert it into cylindrical coordinates, and then map it into servo angles (θ, α, β). This allows the robotic arm to reach any point within its workspace.

## Communication Protocol

Next, I will briefly introduce the communication between the high-level controller and the Arduino.

We use a UART serial interface to transmit control commands. On the high-level side, either MATLAB or Python sends four parameters, which are floating-point values representing the target position and an additional control parameter.

To ensure reliable communication, we designed a simple but robust communication protocol. Each data packet includes clear start and end markers, so that the Arduino can correctly identify and parse each message. Another important point is that we use fixed-width formatting for the floating-point numbers. This ensures that each parameter has a consistent length, which greatly reduces parsing errors during transmission.

During development, we found that without these design choices, the system would occasionally misread data or produce unstable behavior. After refining the protocol, the communication becomes very stable, and we can achieve almost error-free data transmission.
