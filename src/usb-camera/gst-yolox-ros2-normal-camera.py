#!/usr/bin/env python
# -*- coding: utf-8 -*-

print(" ")
print("yolox-ros2-gst-normal-camera")
print(" ")

# ***********************************************************************
# Import Packages
# ***********************************************************************
import os
import time
import numpy as np
import cv2
import random
import colorsys
import threading

from matplotlib.patches import Rectangle
from matplotlib import pyplot as plt

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from inputs import devices
from inputs import get_gamepad

# cv2_Display True/False
display = False

# detect_target
detect_target = 0 #Human
detect_flag ='q0'
auto_move_flag = False

# ***********************************************************************
# input file names
# ***********************************************************************
dpu_model   = os.path.abspath("dpu.bit")
cnn_xmodel  = os.path.join("./"        , "yolox_nano_pt.xmodel")
labels_file = os.path.join("./img"     , "coco2017_classes.txt")

# ***********************************************************************
# Prepare the Overlay and load the "cnn.xmodel"
# ***********************************************************************
from pynq_dpu import DpuOverlay
from pynq import Overlay
from pynq.lib import AxiGPIO

overlay = DpuOverlay(dpu_model)
overlay.load_model(cnn_xmodel)
ol = overlay

# ***********************************************************************
# Utility Functions (PWM/GPIO)
# ***********************************************************************

# GPIO_set
gpio_0_ip = ol.ip_dict['axi_gpio_0']
gpio_out = AxiGPIO(gpio_0_ip).channel1
mask = 0xffffffff
gpio_out.write(0x07,mask) #2*motor_driver_enable + Blue LED_ON

# PWM_set
# utility functions for bit manipulation
def set_bit(value, bit):
    return value | (1 << bit)

def clear_bit(value, bit):
    return value & ~(1 << bit)

def get_bit(value, bit):
    return (value >> bit) & 1

motor_B2 = ol.axi_timer_0
motor_B1 = ol.axi_timer_1
motor_A2 = ol.axi_timer_2
motor_A1 = ol.axi_timer_3

# extract register addresses (will be the same for every Axi Timer)
TCSR0 = ol.ip_dict['axi_timer_0']['registers']['TCSR0']
TCSR1 = ol.ip_dict['axi_timer_0']['registers']['TCSR1']
TCSR0_address = TCSR0['address_offset']
TCSR1_address = TCSR1['address_offset']
TCSR0_register = TCSR0['fields'] # bit_offset for address
TCSR1_register = TCSR1['fields']
TLR0 = ol.ip_dict['axi_timer_0']['registers']['TLR0']
TLR1 = ol.ip_dict['axi_timer_0']['registers']['TLR1']
TLR0_address = TLR0['address_offset']
TLR1_address = TLR1['address_offset']

# create the configuration values for the control register
temp_val_0 = 0
temp_val_1 = 0

# The PWMA0 bit in TCSR0 and PWMB0 bit in TCSR1 must be set to 1 to enable PWM mode.
temp_val_0 = set_bit(temp_val_0, TCSR0_register['PWMA0']['bit_offset'])
temp_val_1 = set_bit(temp_val_1, TCSR1_register['PWMA1']['bit_offset'])

# The GenerateOut signals must be enabled in the TCSR (bit GENT set to 1). The PWM0
# signal is generated from the GenerateOut signals of Timer 0 and Timer 1, so these
# signals must be enabled in both timer/counters
temp_val_0 = set_bit(temp_val_0, TCSR0_register['GENT0']['bit_offset'])
temp_val_1 = set_bit(temp_val_1, TCSR1_register['GENT1']['bit_offset'])

# The counter can be set to count up or down. UDT
temp_val_0 = set_bit(temp_val_0, TCSR0_register['UDT0']['bit_offset'])
temp_val_1 = set_bit(temp_val_1, TCSR1_register['UDT1']['bit_offset'])

# set Autoreload (ARHT0 = 1)
temp_val_0 = set_bit(temp_val_0, TCSR0_register['ARHT0']['bit_offset'])
temp_val_1 = set_bit(temp_val_1, TCSR1_register['ARHT1']['bit_offset'])

# enable timer (ENT0 = 1)
temp_val_0 = set_bit(temp_val_0, TCSR0_register['ENT0']['bit_offset'])
temp_val_1 = set_bit(temp_val_1, TCSR1_register['ENT1']['bit_offset'])

# here you must see "0b1010010110" twice
print(bin(temp_val_0))
print(bin(temp_val_1))


# PWM_A_motor_set
def set_motor_A_pwm(duty_cycle, direction):
    _period_ = 20000  # 50Hz, 20ms
    _pulse_ = duty_cycle  # 0-100
    period = int((_period_ & 0x0ffff) * 100)
    pulse = int((_pulse_ & 0x07f) * period / 100)

    print(f"period 20ms: {period}")
    print(f"pulse {duty_cycle}%: {pulse}")
    print(f"direction: {direction}")

    motor_A1.write(TCSR0['address_offset'], temp_val_0)
    motor_A1.write(TCSR1['address_offset'], temp_val_1)
    motor_A1.write(TLR0['address_offset'], period)

    motor_A2.write(TCSR0['address_offset'], temp_val_0)
    motor_A2.write(TCSR1['address_offset'], temp_val_1)
    motor_A2.write(TLR0['address_offset'], period)

    # direction
    if direction == 'forward':
        motor_A1.write(TLR1['address_offset'], pulse)
        motor_A2.write(TLR1['address_offset'], 0)
    elif direction == 'reverse':
        motor_A1.write(TLR1['address_offset'], 0)
        motor_A2.write(TLR1['address_offset'], pulse)
    elif direction == 'coast':
        motor_A1.write(TLR1['address_offset'], 0)
        motor_A2.write(TLR1['address_offset'], 0)
    elif direction == 'break':
        motor_A1.write(TLR1['address_offset'], 100)
        motor_A2.write(TLR1['address_offset'], 100)   
    else:
        print("Invalid direction. Please use 'forward' or 'reverse'or 'coast' or 'break'.")


# PWM_B_motor_set

def set_motor_B_pwm(duty_cycle, direction):
    _period_ = 20000  # 50Hz, 20ms
    _pulse_ = duty_cycle  # 0-100
    period = int((_period_ & 0x0ffff) * 100)
    pulse = int((_pulse_ & 0x07f) * period / 100)

    print(f"period 20ms: {period}")
    print(f"pulse {duty_cycle}%: {pulse}")
    print(f"direction: {direction}")

    motor_B1.write(TCSR0['address_offset'], temp_val_0)
    motor_B1.write(TCSR1['address_offset'], temp_val_1)
    motor_B1.write(TLR0['address_offset'], period)

    motor_B2.write(TCSR0['address_offset'], temp_val_0)
    motor_B2.write(TCSR1['address_offset'], temp_val_1)
    motor_B2.write(TLR0['address_offset'], period)

    # direction
    if direction == 'forward':
        motor_B1.write(TLR1['address_offset'], pulse)
        motor_B2.write(TLR1['address_offset'], 0)
    elif direction == 'reverse':
        motor_B1.write(TLR1['address_offset'], 0)
        motor_B2.write(TLR1['address_offset'], pulse)
    elif direction == 'coast':
        motor_B1.write(TLR1['address_offset'], 0)
        motor_B2.write(TLR1['address_offset'], 0)
    elif direction == 'break':
        motor_B1.write(TLR1['address_offset'], 100)
        motor_B2.write(TLR1['address_offset'], 100)   
    else:
        print("Invalid direction. Please use 'forward' or 'reverse'or 'coast' or 'break'.")


# ***********************************************************************
# Utility Functions (Controller)
# ***********************************************************************

# Retrieve the gamepad device
gamepads = devices.gamepads
print(gamepads)
if not gamepads:
    raise ValueError("No gamepad found.")

pwm_power_1 = 10
pwm_power_2 = 30
pwm_power_3 = 50
pwm_power_4 = 99

#Left(B)-motor_set
def control_motor_based_on_joy_ly(joy_ly):
    # Control motor based on joy_ly value divided into 8 segments
    if 0 <= joy_ly < 32:  # Segment 1
        set_motor_B_pwm(pwm_power_3, 'forward')
    elif 32 <= joy_ly < 64:  # Segment 2
        set_motor_B_pwm(pwm_power_2, 'forward')
    elif 64 <= joy_ly < 96:  # Segment 3
        set_motor_B_pwm(pwm_power_1, 'forward')
    elif 96 <= joy_ly < 128:  # Segment 4
        set_motor_B_pwm(0, 'coast') # Consider this as neutral or coast
    elif 128 <= joy_ly < 160:  # Segment 5
        set_motor_B_pwm(0, 'coast') # Consider this as neutral or coast
    elif 160 <= joy_ly < 192:  # Segment 6
        set_motor_B_pwm(pwm_power_1, 'reverse')
    elif 192 <= joy_ly < 224:  # Segment 7
        set_motor_B_pwm(pwm_power_2, 'reverse')
    elif 224 <= joy_ly <= 255:  # Segment 8
        set_motor_B_pwm(pwm_power_3, 'reverse')

#Right(A)-motor_set
def control_motor_based_on_joy_ry(joy_ry):
    # Control motor based on joy_ly value divided into 8 segments
    if 0 <= joy_ry < 32:  # Segment 1
        set_motor_A_pwm(pwm_power_3, 'forward')
    elif 32 <= joy_ry < 64:  # Segment 2
        set_motor_A_pwm(pwm_power_2, 'forward')
    elif 64 <= joy_ry < 96:  # Segment 3
        set_motor_A_pwm(pwm_power_1, 'forward')
    elif 96 <= joy_ry < 128:  # Segment 4
        set_motor_A_pwm(0, 'coast') # Consider this as neutral or coast
    elif 128 <= joy_ry < 160:  # Segment 5
        set_motor_A_pwm(0, 'coast') # Consider this as neutral or coast
    elif 160 <= joy_ry < 192:  # Segment 6
        set_motor_A_pwm(pwm_power_1, 'reverse')
    elif 192 <= joy_ry < 224:  # Segment 7
        set_motor_A_pwm(pwm_power_2, 'reverse')
    elif 224 <= joy_ry <= 255:  # Segment 8
        set_motor_A_pwm(pwm_power_3, 'reverse')

def control_motor_based_on_arm_f():
    gpio_out.write(0x27,mask) #arm_forward

def control_motor_based_on_arm_r():
    gpio_out.write(0x47,mask) #arm_reverse

def controller_event():
    #joystick input and control the motor based on it
    events = get_gamepad()
    print(events)
    for event in events:
        if event.code == 'ABS_HAT0Y' and event.state == -1:  #forward
            print(event.code)
            set_motor_A_pwm(pwm_power_3, 'forward')
            set_motor_B_pwm(pwm_power_3, 'forward')
        elif event.code == 'ABS_HAT0Y' and event.state == 1: #back
            print(event.code)
            set_motor_A_pwm(pwm_power_3, 'reverse')
            set_motor_B_pwm(pwm_power_3, 'reverse')
        elif event.code == 'ABS_HAT0X' and event.state == 1:  #right-rotate
            print(event.code)
            set_motor_A_pwm(pwm_power_3, 'forward')
            set_motor_B_pwm(pwm_power_3, 'reverse')
        elif event.code == 'ABS_HAT0X' and event.state == -1:  #left-rotate
            print(event.code)
            set_motor_A_pwm(pwm_power_3, 'reverse')
            set_motor_B_pwm(pwm_power_3, 'forward')
        elif event.code == 'BTN_SOUTH':  #coast
            print(event.code)
            set_motor_A_pwm(0, 'coast')
            set_motor_B_pwm(0, 'coast')
        elif event.code == 'BTN_Z':  #High-right-rotate
            print(event.code)
            set_motor_A_pwm(pwm_power_4, 'forward')
            set_motor_B_pwm(pwm_power_4, 'reverse')
        elif event.code == 'BTN_WEST':  #High-left-rotate
            print(event.code)
            set_motor_A_pwm(pwm_power_4, 'reverse')
            set_motor_B_pwm(pwm_power_4, 'forward')       
        elif  event.code == 'BTN_C':    #Arm-forward
            print(event.code)
            control_motor_based_on_arm_f()
        elif  event.code == 'BTN_EAST': #Arm-reverse
            print(event.code)
            control_motor_based_on_arm_r()
        elif  event.code == 'BTN_NORTH': #Arm-free
            gpio_out.write(0x07,mask)   #2*motor_driver_enable + Blue LED_ON


def continuous_controller():
    global detect_flag, auto_move_flag
    rotate_90_time = 0.3
    foward_time = 0.3
    # Loop until the exit flag is set
    while not exit_flag:
        controller_event()  # Call the controller event function
        time.sleep(0.1)  # Set an appropriate execution interval
        print("func_detect_flag: {}".format(detect_flag))

        # if detect_flag == 'q1':  #front-detect
        #     auto_move_flag = True
        #     set_motor_A_pwm(pwm_power_3, 'forward')
        #     set_motor_B_pwm(pwm_power_3, 'forward')
        #     time.sleep(foward_time)  # Set an appropriate execution interval
        #     set_motor_A_pwm(0, 'coast')
        #     set_motor_B_pwm(0, 'coast')
        #     detect_flag ='q0'
        #     auto_move_flag = False
        # elif detect_flag == 'q2':  #right-detect
        #     auto_move_flag = True
        #     set_motor_A_pwm(pwm_power_3, 'reverse')
        #     set_motor_B_pwm(pwm_power_3, 'forward')
        #     time.sleep(rotate_90_time)  # Set an appropriate execution interval
        #     set_motor_A_pwm(0, 'coast')
        #     set_motor_B_pwm(0, 'coast')
        #     detect_flag ='q0'
        #     auto_move_flag = False
        # elif detect_flag == 'q3':  #back-detect
        #     auto_move_flag = True
        #     set_motor_A_pwm(pwm_power_3, 'reverse')
        #     set_motor_B_pwm(pwm_power_3, 'forward')
        #     time.sleep(2 * rotate_90_time)  # Set an appropriate execution interval
        #     set_motor_A_pwm(0, 'coast')
        #     set_motor_B_pwm(0, 'coast')
        #     detect_flag ='q0'
        #     auto_move_flag = False
        # elif detect_flag == 'q4':  #left-detect
        #     auto_move_flag = True
        #     set_motor_A_pwm(pwm_power_3, 'forward')
        #     set_motor_B_pwm(pwm_power_3, 'reverse')
        #     time.sleep(1 * rotate_90_time)  # Set an appropriate execution interval
        #     set_motor_A_pwm(0, 'coast')
        #     set_motor_B_pwm(0, 'coast')
        #     detect_flag ='q0'
        #     auto_move_flag = False       



# ***********************************************************************
# Utility Functions (DPU)
# ***********************************************************************
def preprocess(image, input_size, swap=(2, 0, 1)):
    if len(image.shape) == 3:
        padded_image = np.ones(
            (input_size[0], input_size[1], 3), dtype=np.uint8) * 114
    else:
        padded_image = np.ones(input_size, dtype=np.uint8) * 114

    ratio = min(input_size[0] / image.shape[0],
                input_size[1] / image.shape[1])
    resized_image = cv2.resize(
        image,
        (int(image.shape[1] * ratio), int(image.shape[0] * ratio)),
        interpolation=cv2.INTER_LINEAR,
    )
    resized_image = resized_image.astype(np.uint8)

    padded_image[:int(image.shape[0] * ratio), :int(image.shape[1] *
                                                    ratio)] = resized_image
    #padded_image = padded_image.transpose(swap)

    padded_image = np.ascontiguousarray(padded_image, dtype=np.float32)
    return padded_image, ratio

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def softmax(x):
    exp_x = np.exp(x - np.max(x))
    return exp_x / exp_x.sum(axis=-1, keepdims=True)


def postprocess(
    outputs,
    img_size,
    ratio,
    nms_th,
    nms_score_th,
    max_width,
    max_height,
    p6=False,
):
    grids = []
    expanded_strides = []

    if not p6:
        strides = [8, 16, 32]
    else:
        strides = [8, 16, 32, 64]

    hsizes = [img_size[0] // stride for stride in strides]
    wsizes = [img_size[1] // stride for stride in strides]

    for hsize, wsize, stride in zip(hsizes, wsizes, strides):
        xv, yv = np.meshgrid(np.arange(wsize), np.arange(hsize))
        grid = np.stack((xv, yv), 2).reshape(1, -1, 2)
        grids.append(grid)
        shape = grid.shape[:2]
        expanded_strides.append(np.full((*shape, 1), stride))

    grids = np.concatenate(grids, 1)
    expanded_strides = np.concatenate(expanded_strides, 1)
    outputs[..., :2] = (outputs[..., :2] + grids) * expanded_strides
    outputs[..., 2:4] = np.exp(outputs[..., 2:4]) * expanded_strides

    predictions = outputs[0]
    boxes = predictions[:, :4]
    scores = sigmoid(predictions[:, 4:5]) * softmax(predictions[:, 5:])
    #scores = predictions[:, 4:5] * predictions[:, 5:]
    
    boxes_xyxy = np.ones_like(boxes)
    boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.
    boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.
    boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.
    boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.
    boxes_xyxy /= ratio

    dets = multiclass_nms(
        boxes_xyxy,
        scores,
        nms_thr=nms_th,
        score_thr=nms_score_th,
    )

    bboxes, scores, class_ids = [], [], []
    if dets is not None:
        bboxes, scores, class_ids = dets[:, :4], dets[:, 4], dets[:, 5]
        for bbox in bboxes:
            bbox[0] = max(0, bbox[0])
            bbox[1] = max(0, bbox[1])
            bbox[2] = min(bbox[2], max_width)
            bbox[3] = min(bbox[3], max_height)

    return bboxes, scores, class_ids


def nms(boxes, scores, nms_thr):
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        ovr = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(ovr <= nms_thr)[0]
        order = order[inds + 1]

    return keep


def multiclass_nms(
    boxes,
    scores,
    nms_thr,
    score_thr,
    class_agnostic=True,
):
    if class_agnostic:
        nms_method = multiclass_nms_class_agnostic
    else:
        nms_method = multiclass_nms_class_aware

    return nms_method(boxes, scores, nms_thr, score_thr)

def multiclass_nms_class_aware(boxes, scores, nms_thr, score_thr):
    final_dets = []
    num_classes = scores.shape[1]

    for cls_ind in range(num_classes):
        cls_scores = scores[:, cls_ind]
        valid_score_mask = cls_scores > score_thr

        if valid_score_mask.sum() == 0:
            continue
        else:
            valid_scores = cls_scores[valid_score_mask]
            valid_boxes = boxes[valid_score_mask]
            keep = self._nms(valid_boxes, valid_scores, nms_thr)
            if len(keep) > 0:
                cls_inds = np.ones((len(keep), 1)) * cls_ind
                dets = np.concatenate(
                    [
                        valid_boxes[keep], valid_scores[keep, None],
                        cls_inds
                    ],
                    1,
                )
                final_dets.append(dets)

    if len(final_dets) == 0:
        return None

    return np.concatenate(final_dets, 0)


def multiclass_nms_class_agnostic(boxes, scores, nms_thr,
                                    score_thr):
    cls_inds = scores.argmax(1)
    cls_scores = scores[np.arange(len(cls_inds)), cls_inds]

    valid_score_mask = cls_scores > score_thr

    if valid_score_mask.sum() == 0:
        return None

    valid_scores = cls_scores[valid_score_mask]
    valid_boxes = boxes[valid_score_mask]
    valid_cls_inds = cls_inds[valid_score_mask]
    keep = nms(valid_boxes, valid_scores, nms_thr)

    dets = None
    if keep:
        dets = np.concatenate([
            valid_boxes[keep],
            valid_scores[keep, None],
            valid_cls_inds[keep, None],
        ], 1)

    return dets

'''Get model classification information'''	
def get_class(classes_path):
    with open(classes_path) as f:
        class_names = f.readlines()
    class_names = [c.strip() for c in class_names]
    return class_names
    
class_names = get_class(labels_file)
num_classes = len(class_names)

hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))

random.seed(0)
random.shuffle(colors)
random.seed(None)


'''Draw detection frame'''
def draw_bbox(image, bboxes, classes):
    """
    bboxes: [x_min, y_min, x_max, y_max, probability, cls_id] format coordinates.
    """
    image_h, image_w, _ = image.shape

    for i, bbox in enumerate(bboxes):
        coor = np.array(bbox[:4], dtype=np.int32)
        fontScale = 0.5
        score = bbox[4]
        class_ind = int(bbox[5])
        bbox_color = colors[class_ind]
        bbox_thick = int(0.6 * (image_h + image_w) / 600)
        c1, c2 = (coor[0], coor[1]), (coor[2], coor[3])
        cv2.rectangle(image, c1, c2, bbox_color, bbox_thick)
    return image



# ***********************************************************************
# Use VART APIs
# ***********************************************************************

dpu = overlay.runner
inputTensors = dpu.get_input_tensors()

outputTensors = dpu.get_output_tensors()
shapeIn = tuple(inputTensors[0].dims)

shapeOut0 = (tuple(outputTensors[0].dims)) # (1, 52, 52, 85)
shapeOut1 = (tuple(outputTensors[1].dims)) # (1, 26, 26, 85)
shapeOut2 = (tuple(outputTensors[2].dims)) # (1, 13, 13, 85)

outputSize0 = int(outputTensors[0].get_data_size() / shapeIn[0]) # 229840
outputSize1 = int(outputTensors[1].get_data_size() / shapeIn[0]) # 57460
outputSize2 = int(outputTensors[2].get_data_size() / shapeIn[0]) # 14365

input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
output_data = [np.empty(shapeOut0, dtype=np.float32, order="C"), 
               np.empty(shapeOut1, dtype=np.float32, order="C"),
               np.empty(shapeOut2, dtype=np.float32, order="C")]
image = input_data[0]


def run(input_image, section_i, display=False):
    input_shape=(416, 416)
    class_score_th=0.3
    nms_th=0.45
    nms_score_th=0.1

    # Pre-processing
    print(input_image.shape)
    image_size = input_image.shape[:2]
    image_height, image_width = input_image.shape[0], input_image.shape[1]
    image_data, ratio = preprocess(input_image, input_shape)
    
    # Fetch data to DPU and trigger it
    image[0,...] = image_data.reshape(shapeIn[1:])
    job_id = dpu.execute_async(input_data, output_data)
    dpu.wait(job_id)

    # Decode output from YOLOX-nano
    outputs = np.concatenate([output.reshape(1, -1, output.shape[-1]) for output in output_data], axis=1)
    bboxes, scores, class_ids = postprocess(
        outputs,
        input_shape,
        ratio,
        nms_th,
        nms_score_th,
        image_width,
        image_height,
    )
    
    #Draw boxes into image
    bboxes_with_scores_and_classes = []
    for i in range(len(bboxes)):
        bbox = bboxes[i].tolist() + [scores[i], class_ids[i]]
        bboxes_with_scores_and_classes.append(bbox)
    bboxes_with_scores_and_classes = np.array(bboxes_with_scores_and_classes)
    display = draw_bbox(input_image, bboxes_with_scores_and_classes, class_names)

    print("section: {}".format(section_i))
    # print("Boxes: {}".format(boxes))
    # print("Number of detected objects: {}".format(len(boxes)))
    # print("Scores: {}".format(scores))
    print("Details of detected objects: {}".format(class_ids))
    # print(" ")

    return bboxes, scores, class_ids
    

# ***********************************************************************
# Utility Functions (ROS2)
# ***********************************************************************

# Initialize GStreamer and ROS 2
Gst.init(None)
rclpy.init()
node_image = rclpy.create_node('image_publisher')
# Create publishers for each quadrant
img_publishers = {
    'q1': node_image.create_publisher(Image, 'camera/image/q1', 10)
}
bridge = CvBridge()

node_box = Node('bounding_box_visualizer')
box_publishers = node_box.create_publisher(MarkerArray, 'visualization_marker_array', 10)

# Function Rotate the coordinates
def rotate_coordinates(x, y, angle_degrees):
    # Convert angle from degrees to radians
    angle_radians = np.radians(angle_degrees)
    
    # Rotation matrix
    rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)], 
                                [np.sin(angle_radians), np.cos(angle_radians)]])
    
    # Original coordinate vector
    original_vector = np.array([x, y])
    
    # Compute the new coordinate vector
    new_vector = rotation_matrix.dot(original_vector)
    return new_vector[0], new_vector[1]


# Function to act as a node for publishing markers det: [x_min, y_min, x_max, y_max]
def publish_markers(publisher, node, detections, classes, section):
    marker_array = MarkerArray()

    if section == 'q1':  #front
        m_offset =[0, 0]
        theta = 0
        id_offset =0

    for i, det in enumerate(detections):
        # print("det: {}".format(det))
        # print("classes: {}".format(classes))
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = node.get_clock().now().to_msg()
        marker.ns = "yolo_boxes"
        marker.id = i + id_offset
        marker.action = Marker.ADD

        if classes[i] in (32, 51):  #Sports Ball or Orange
            marker.type = Marker.SPHERE
        else:
            marker.type = Marker.CUBE
        # marker.type = Marker.SPHERE

        p_adj = 20
        s_adj = 10
        tmp_marker_x = (det[0] + det[2]) / (2 * p_adj)
        tmp_marker_y = (det[1] + det[3]) / (2 * p_adj)
        x_rotated, y_rotated = rotate_coordinates(tmp_marker_x, tmp_marker_y, theta)
        marker.pose.position.x = x_rotated  + m_offset[0]
        marker.pose.position.y = y_rotated  + m_offset[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(det[2] - det[0]) / s_adj
        marker.scale.y = float(det[3] - det[1]) / s_adj
        marker.scale.z = float(det[3] - det[1]) / s_adj # Height of the box

        #print("det_class: {}".format(int(classes[i])))
        det_color = colors[int(classes[i])]
        # print("det_color: {}".format(det_color))
        marker.color.a = 0.5  # Transparency
        marker.color.r = float(det_color[2])
        marker.color.g = float(det_color[1])
        marker.color.b = float(det_color[0])
        # marker.lifetime = rclpy.duration.Duration(seconds=1)
        marker.lifetime = Duration(seconds=2).to_msg()
        marker_array.markers.append(marker)
    
    publisher.publish(marker_array)


# ***********************************************************************
# Main Program
# ***********************************************************************


# Define and create the GStreamer pipeline
pipeline = "v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480, framerate=30/1 ! videoconvert ! appsink"

# Initialize the VideoCapture object
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open the camera.")
else:
    print("The camera opened successfully.")


# Initialize and start the Motor-controller-thread
exit_flag = False
controller_thread = threading.Thread(target=continuous_controller)
controller_thread.start()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        start_time = time.time()
        # Get the height and width of the image
        height, width, _ = frame.shape


        # Split the image into 2 quadrants
        quadrants = {
            'q1': frame                  #front
        }
        # Apply YOLO object detection to each quadrant
        for key, img in quadrants.items():
            d_boxes, d_scores, d_classes = run(img, key, display)

            # Publish each quadrant as a separate ROS message
            img_publishers[key].publish(bridge.cv2_to_imgmsg(img, encoding="bgr8"))

            # Publish each BOX a separate ROS message
            publish_markers(box_publishers, node_box, d_boxes, d_classes, key)

            if (detect_target in d_classes) and (auto_move_flag == False):
                print("detect_flag: {}".format(detect_flag))
                detect_flag = key

        end_time = time.time()

        print("Total run time: {:.4f} seconds".format(end_time - start_time))
        print("Performance: {} FPS".format(1/(end_time - start_time)))
        print(" ")   

        if cv2.waitKey(1) & 0xFF == ord('q'):
            gpio_out.write(0x00,mask) #All_GPIO_off
            break   

finally:
    exit_flag = True  # Set the flag to end the thread
    controller_thread.join()  # Wait for the thread to ensure it has finished

    cap.release()
    cv2.destroyAllWindows()

# ***********************************************************************
# Clean up
# ***********************************************************************
pipeline.set_state(Gst.State.NULL)
rclpy.shutdown()

del overlay
del dpu
