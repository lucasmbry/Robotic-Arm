# 6-DOF Robotic Manipulator Arm

A custom 6-degree-of-freedom robotic manipulator engineered from scratch, featuring 3D-printed NEMA 17 joints, TMC2209 silent stepper control, and a full ROS 2 / MoveIt motion planning pipeline with digital twin Gazebo simulation

---

### Key Features

* **6-DOF Kinematic Chain:** Fully articulated arm with 6 revolute joints for full end-effector positioning and orientation
* **Closed-Loop / Silent Actuation:** Driven by NEMA steppers paired with TMC2209 drivers using limit switches for simple homing
* **Low-Latency Microcontroller Bridge:** Embedded MicroPython / C++ firmware handling joint trajectory execution and real-time step generation via serial
* **Digital Twin & Simulation:** Accurate URDF / Xacro models paired with Gazebo physics simulation for testing and collision checking
* **MoveIt 2 Motion Planning:** Autonomous trajectory generation, Cartesian path planning, inverse kinematics, and pick and place

---
