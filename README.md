# AMD Pervasive AI Developer Contest
This repository present solution for the AMD Pervasive AI Developer Contest 

[AMD Pervasive AI Developer Contest](https://www.hackster.io/contests/amd2023).

## Introduction
**First of all thank you to AMD and hackster.io for hosting this exciting competition.**

The main project is primarily summarized on hackster.io, so please refer to it there 

**<< under submission >>**.

This project integrates advanced technologies like Object Detection, DPU, PYNQ, and ROS2 with KR260.

A key feature is the incorporation of a **360° Camera**.

![overview](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/overview.png)

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

![Using KR260 and PYNQ](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/Using-KR260-and-PYNQ.png)

## Electrical Diagram overview

This is Main electrical diagram.

![Main-electrical-diagram](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/Main-electrical-diagram.png)

These are PMOD diagrams.

![PMOD1-diagram](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/PMOD1-diagram.png)

![PMOD2-diagram](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/PMOD2-diagram.png)

![PMOD4-diagram](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/PMOD4-diagram.png)

### 1. PYNQ + GPIO(LED Blinking)
For details and specifications, please refer to the hackster.io Subproject below.

[Control GPIO from PYNQ and KR260](https://www.hackster.io/iotengineer22/control-gpio-from-pynq-and-kr260-0d3613)

In this Subproject, we experimented with controlling GPIO on the KR260 FPGA board.

Using Python (PYNQ) , we managed to perform LED output and switch input via the PMOD connector with custom-designed board.

The test .bit .hwh .ipynb files are available on GitHub.

[/jupyter_notebooks/pynq-gpio](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/jupyter_notebooks/pynq-gpio)

![kr260-gpio](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-gpio.png)

### 2. PYNQ + PWM(DC-Motor Control)
For details and specifications, please refer to the hackster.io Subproject below.

[Control PWM(DC-Motor) from PYNQ and KR260](https://www.hackster.io/iotengineer22/control-pwm-dc-motor-from-pynq-and-kr260-bb0296)

In this Subproject, We tested controlling PWM (Pulse Width Modulation) on the KR260 FPGA board.

Using Python (PYNQ), we output PWM signals to control a motor driver board.

The test .bit .hwh .ipynb files are available on GitHub.

[/jupyter_notebooks/pynq-pwm](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/jupyter_notebooks/pynq-pwm)

![kr260-pwm](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-pwm.png)


### 3. Object Detection(Yolo) with DPU-PYNQ
For details and specifications, please refer to the hackster.io Subproject below.

[Object Detection(Yolo) with DPU-PYNQ and KR260](https://www.hackster.io/iotengineer22/object-detection-yolo-with-dpu-pynq-and-kr260-777fb5)

In this Subproject, We tested object detection on images from camera using the KR260 and YOLOv3.

Originally, there was a sample program for PYNQ-DPU, which we modified.

The test .ipynb .xmodel files are available on GitHub.

[/jupyter_notebooks/pynq-dpu](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/jupyter_notebooks/pynq-dpu)

![kr260-dpu](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-dpu.png)


### 4. Implementation DPU, GPIO, and PWM
For details and specifications, please refer to the hackster.io Subproject below.

[Implementation DPU, GPIO, and PWM for KR260](https://www.hackster.io/iotengineer22/implementation-dpu-gpio-and-pwm-for-kr260-f7637b)

In this Subproject, Using Vivado and Vitis, we created a project to synthesize the DPU IP.

We utilized the DPU created on PYNQ with KR260 to perform object detection using Vitis AI (Yolo).

The test .bit .hwh .xclbin .ipynb .xmodel files are available on GitHub.

[/jupyter_notebooks/pynq-original-dpu-model](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/jupyter_notebooks/pynq-original-dpu-model)

![kr260-my-dpu](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-my-dpu.png)


### 5. Remote Control 360° Camera
For details and specifications, please refer to the hackster.io Subproject below.

[Remote Control 360° Camera from KR260](https://www.hackster.io/iotengineer22/remote-control-360-camera-from-kr260-f0ead0)

In this Subproject, we tried controlling the RICOH THETA V 360° camera from the KR260 using PYNQ.

The test .ipynb files are available on GitHub.

[/jupyter_notebooks/theta-check](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/jupyter_notebooks/theta-check)

![kr260-theta](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-theta.png)


### 6. GStreamer + OpenCV with 360°Camera
For details and specifications, please refer to the hackster.io Subproject below.

[GStreamer + OpenCV with 360° Camera and KR260](https://www.hackster.io/iotengineer22/gstreamer-opencv-with-360-camera-and-kr260-308442)

In this Subproject, we'll walk you through how we achieved real-time image processing using a KR260 and a 360° camera (RICOH THETA).

The test .py files are available on GitHub.

[/src/gst-test](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/src/gst-test)

![kr260-gst](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-gst.png)


### 7. 360 Live Streaming + Object Detect(DPU)
For details and specifications, please refer to the hackster.io Subproject below.

[360 Live Streaming + Object Detect(DPU) with KR260](https://www.hackster.io/iotengineer22/360-live-streaming-object-detect-dpu-with-kr260-69dced)

In this Subproject, we conducted real-time object detection on 360 live streaming image data.

The test .bit .hwh .xclbin .py .xmodel  files are available on GitHub.

[/src/gst-dpu-test](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/src/gst-dpu-test)

![kr260-gst-dpu](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-gst-dpu.png)


### 8. ROS2 3D Marker from 360 Live Streaming
For details and specifications, please refer to the hackster.io Subproject below.

[ROS2 3D Marker from 360 Live Streaming with KR260](https://www.hackster.io/iotengineer22/ros2-3d-marker-from-360-live-streaming-with-kr260-a8c51c)

In this Subproject, we experimented with processing 360° camera images and markers using ROS2 Rviz2.

The test .bit .hwh .xclbin .py .xmodel  files are available on GitHub.

[/src/gst-ros2](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/src/gst-ros2)

![kr260-gst-ros2](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-gst-ros2.png)


### 9. Control 360° Object Detection Robot Car
For details and specifications, please refer to the hackster.io Subproject below.

[Control 360° Object Detection Robot Car with KR260](https://www.hackster.io/iotengineer22/control-360-object-detection-robot-car-with-kr260-95a07e)

In this Subproject, we control 360° Object Detection Robot Car with KR260.

The object detection is performed using DPU, and marker output is executed with ROS2 while the Robot Car is in motion.

The test .bit .hwh .xclbin .py .xmodel  files are available on GitHub.

[/src/gst-ros2](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/src/gst-ros2)

![kr260-run-robot](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-run-robot.png)


### 10. Improve Object Detection Speed with YOLOX
For details and specifications, please refer to the hackster.io Subproject below.

[Improve Object Detection Speed with YOLOX](https://www.hackster.io/iotengineer22/object-detection-with-yolox-pynq-and-kr260-13d32f)

In this Subproject, We conduct 360° Object Detection with YOLOX.

We conducted object detection using the KR260's DPU, and used the lightweight model "YOLOX-nano" with PyTorch.

The test .bit .hwh .xclbin .py .xmodel  files are available on GitHub.

[/jupyter_notebooks/pynq-yolox](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/jupyter_notebooks/pynq-yolox)

[/src/yolox-test](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/src/yolox-test)

![kr260-yolox](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-yolox.png)


### 11. Benchmark Architectures of the DPU
For details and specifications, please refer to the hackster.io Subproject below.

[Benchmark Architectures of the DPU](https://www.hackster.io/iotengineer22/benchmark-architectures-of-the-dpu-with-kr260-699f19)

In this Subproject, We compared the speeds of different DPU architectures.

We conducted object detection using the KR260's DPU, and used the lightweight model "YOLOX-nano" with PyTorch.

The test .bit .hwh .xclbin .py .xmodel  files are available on GitHub.

[/jupyter_notebooks/pynq-benchmark](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/jupyter_notebooks/pynq-benchmark)

![kr260-DPU-inference](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-DPU-inference.png)


### 12. Power Consumption of 360° Object Detection Robot Car
For details and specifications, please refer to the hackster.io Subproject below.

[Power Consumption of 360° Object Detection Robot Car](https://www.hackster.io/iotengineer22/power-consumption-of-robot-car-with-kr260-8c9fbc)

In this Subproject, We measured Power Consumption of Robot Car with KR260.

When trying to power the KR260 from a mobile battery(20W), a power shortage occurred during the program startup.

![kr260-power](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-power.png)


### 13. Application to Vitis AI ONNX Runtime Engine (VOE)
For details and specifications, please refer to the hackster.io Subproject below.

[Application to Vitis AI ONNX Runtime Engine (VOE)](https://www.hackster.io/iotengineer22/vitis-ai-onnx-runtime-engine-voe-with-kr260-python-0d02c3)

In this Subproject, We introduce Vitis AI ONNX Runtime Engine (VOE) with KR260.

We built an ONNX environment on the KR260 and executed ONNX Runtime.

**In this subproject, we will conduct tests in a different environment from the main project as part of the benchmarking process.**

[/src/onnx-test](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/src/onnx-test)

![kr260-onnx](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-onnx.png)


### 14. Appendix: Object Detection Using YOLOX with a Webcam
For details and specifications, please refer to the hackster.io Subproject below.

[Object Detection Using YOLOX with a Webcam](https://www.hackster.io/iotengineer22/object-detection-with-kr260-from-a-webcam-85c261)

In this Subproject, We tried object detection with a regular USB-connected webcam using the KR260.

Many people may not have the 360° camera required for Main project.

**Therefore, as a reference, we will introduce a subproject using a generic webcam**, as well as controlling DPU, GPIO and Output ROS2.

[/src/usb-camera](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/tree/main/src/usb-camera)

![kr260-onnx](https://github.com/iotengineer22/AMD-Pervasive-AI-Developer-Contest/blob/main/imgs/kr260-onnx.png)
