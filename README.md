# AMD-Pervasive-AI-Developer-Contest
This repository present solution for the AMD Pervasive AI Developer Contest.[AMD Pervasive AI Developer Contest](https://www.hackster.io/contests/amd2023).

![ROS2 Marker Output](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/imgs/ros2-marker.gif)

## Introduction
First of all thank you to AMD and hackster.io for hosting this exciting competition.

The main project is primarily summarized on hackster.io, so please refer to it there **under submission**.

This project integrates advanced technologies like Object Detection, DPU, PYNQ, and ROS2 with KR260.

A key feature is the incorporation of a **360° Camera**.

By leveraging the KR260, we've developed a robot with 360-degree AI vision.

![overview](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/imgs/overview.png)

## Structure
    .
    ├── bom                 # BOM file (BOM list)
    ├── jupyter_notebooks   # Jupyter Notebooks (pre-Test Program)
    ├── pcb                 # PCB link files   
    ├── src                 # Python files (Test Program)   
    ├── LICENSE
    └── README.md

## Solution overview

Using KR260 and PYNQ, 360° object detection is processed on the PL (FPGA).

The PL also handles PWM and GPIO for driving the Robot Car and Arm.

By integrating PL and PS, the robot achieves 360-degree AI vision.

![Using KR260 and PYNQ](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/imgs/Using-KR260-and-PYNQ.png)

