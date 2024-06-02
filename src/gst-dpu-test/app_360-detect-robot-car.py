#!/usr/bin/env python
# -*- coding: utf-8 -*-


print(" ")
print("360-auto-detect-robot-car-test")
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

# ***********************************************************************
# input file names
# ***********************************************************************
cnn_xmodel  = os.path.join("./"        , "kr260_yolov3_tf2.xmodel")
labels_file = os.path.join("./img"     , "coco2017_classes.txt")

# ***********************************************************************
# Utility Functions (DPU)
# ***********************************************************************
anchor_list = [10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326]
anchor_float = [float(x) for x in anchor_list]
anchors = np.array(anchor_float).reshape(-1, 2)


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
colors = list(map(lambda x: 
                  (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), 
                  colors))
random.seed(0)
random.shuffle(colors)
random.seed(None)


'''resize image with unchanged aspect ratio using padding'''
def letterbox_image(image, size):
    ih, iw, _ = image.shape
    w, h = size
    scale = min(w/iw, h/ih)
    
    nw = int(iw*scale)
    nh = int(ih*scale)

    image = cv2.resize(image, (nw,nh), interpolation=cv2.INTER_LINEAR)
    new_image = np.ones((h,w,3), np.uint8) * 128
    h_start = (h-nh)//2
    w_start = (w-nw)//2
    new_image[h_start:h_start+nh, w_start:w_start+nw, :] = image
    return new_image


'''image preprocessing'''
def pre_process(image, model_image_size):
    image = image[...,::-1]
    image_h, image_w, _ = image.shape
 
    if model_image_size != (None, None):
        assert model_image_size[0]%32 == 0, 'Multiples of 32 required'
        assert model_image_size[1]%32 == 0, 'Multiples of 32 required'
        boxed_image = letterbox_image(image, tuple(reversed(model_image_size)))
    else:
        new_image_size = (image_w - (image_w % 32), image_h - (image_h % 32))
        boxed_image = letterbox_image(image, new_image_size)
    image_data = np.array(boxed_image, dtype='float32')
    image_data /= 255.
    image_data = np.expand_dims(image_data, 0) 	
    return image_data


def _get_feats(feats, anchors, num_classes, input_shape):
    num_anchors = len(anchors)
    anchors_tensor = np.reshape(np.array(anchors, dtype=np.float32), [1, 1, 1, num_anchors, 2])
    grid_size = np.shape(feats)[1:3]
    nu = num_classes + 5
    predictions = np.reshape(feats, [-1, grid_size[0], grid_size[1], num_anchors, nu])
    grid_y = np.tile(np.reshape(np.arange(grid_size[0]), [-1, 1, 1, 1]), [1, grid_size[1], 1, 1])
    grid_x = np.tile(np.reshape(np.arange(grid_size[1]), [1, -1, 1, 1]), [grid_size[0], 1, 1, 1])
    grid = np.concatenate([grid_x, grid_y], axis = -1)
    grid = np.array(grid, dtype=np.float32)

    box_xy = (1/(1+np.exp(-predictions[..., :2])) + grid) / np.array(grid_size[::-1], dtype=np.float32)
    box_wh = np.exp(predictions[..., 2:4]) * anchors_tensor / np.array(input_shape[::-1], dtype=np.float32)
    box_confidence = 1/(1+np.exp(-predictions[..., 4:5]))
    box_class_probs = 1/(1+np.exp(-predictions[..., 5:]))
    return box_xy, box_wh, box_confidence, box_class_probs


def correct_boxes(box_xy, box_wh, input_shape, image_shape):
    box_yx = box_xy[..., ::-1]
    box_hw = box_wh[..., ::-1]
    input_shape = np.array(input_shape, dtype = np.float32)
    image_shape = np.array(image_shape, dtype = np.float32)
    new_shape = np.around(image_shape * np.min(input_shape / image_shape))
    offset = (input_shape - new_shape) / 2. / input_shape
    scale = input_shape / new_shape
    box_yx = (box_yx - offset) * scale
    box_hw *= scale

    box_mins = box_yx - (box_hw / 2.)
    box_maxes = box_yx + (box_hw / 2.)
    boxes = np.concatenate([
        box_mins[..., 0:1],
        box_mins[..., 1:2],
        box_maxes[..., 0:1],
        box_maxes[..., 1:2]
    ], axis = -1)
    boxes *= np.concatenate([image_shape, image_shape], axis = -1)
    return boxes


def boxes_and_scores(feats, anchors, classes_num, input_shape, image_shape):
    box_xy, box_wh, box_confidence, box_class_probs = _get_feats(feats, anchors, classes_num, input_shape)
    boxes = correct_boxes(box_xy, box_wh, input_shape, image_shape)
    boxes = np.reshape(boxes, [-1, 4])
    box_scores = box_confidence * box_class_probs
    box_scores = np.reshape(box_scores, [-1, classes_num])
    return boxes, box_scores


'''Draw detection frame'''
def draw_bbox(image, bboxes, classes):
    """
    bboxes: [x_min, y_min, x_max, y_max, probability, cls_id] format coordinates.
    """
    num_classes = len(classes)
    image_h, image_w, _ = image.shape
    hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
    colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
    colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))

    random.seed(0)
    random.shuffle(colors)
    random.seed(None)

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


def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.

    # Arguments
        boxes: ndarray, boxes of objects.
        scores: ndarray, scores of objects.

    # Returns
        keep: ndarray, index of effective boxes.
    """
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2-x1+1)*(y2-y1+1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 1)
        h1 = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= 0.55)[0]  # threshold
        order = order[inds + 1]

    return keep


def draw_boxes(image, boxes, scores, classes):
    for i, bbox in enumerate(boxes):
        # if scores[i] >= 0.7:
        #     top = int(bbox[0])
        #     left = int(bbox[1])
        #     bottom = int(bbox[2])
        #     right = int(bbox[3])

        #     score, class_index = scores[i], classes[i]
        #     label = '{}: {:.4f}'.format(class_names[class_index], score) 
        #     color = tuple([color for color in colors[class_index]])
        #     cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        #     cv2.putText(image, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)  

        top = int(bbox[0])
        left = int(bbox[1])
        bottom = int(bbox[2])
        right = int(bbox[3])

        score, class_index = scores[i], classes[i]
        label = '{}: {:.4f}'.format(class_names[class_index], score) 
        color = tuple([color for color in colors[class_index]])
        cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        cv2.putText(image, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)  
    return image


def evaluate(yolo_outputs, image_shape, class_names, anchors):
    score_thresh = 0.2
    anchor_mask = [[6, 7, 8], [3, 4, 5], [0, 1, 2]]
    boxes = []
    box_scores = []
    input_shape = np.shape(yolo_outputs[0])[1 : 3]
    input_shape = np.array(input_shape)*32

    for i in range(len(yolo_outputs)):
        _boxes, _box_scores = boxes_and_scores(
            yolo_outputs[i], anchors[anchor_mask[i]], len(class_names), 
            input_shape, image_shape)
        boxes.append(_boxes)
        box_scores.append(_box_scores)
    boxes = np.concatenate(boxes, axis = 0)
    box_scores = np.concatenate(box_scores, axis = 0)

    mask = box_scores >= score_thresh
    boxes_ = []
    scores_ = []
    classes_ = []
    for c in range(len(class_names)):
        class_boxes_np = boxes[mask[:, c]]
        class_box_scores_np = box_scores[:, c]
        class_box_scores_np = class_box_scores_np[mask[:, c]]
        nms_index_np = nms_boxes(class_boxes_np, class_box_scores_np) 
        class_boxes_np = class_boxes_np[nms_index_np]
        class_box_scores_np = class_box_scores_np[nms_index_np]
        classes_np = np.ones_like(class_box_scores_np, dtype = np.int32) * c
        boxes_.append(class_boxes_np)
        scores_.append(class_box_scores_np)
        classes_.append(classes_np)
    boxes_ = np.concatenate(boxes_, axis = 0)
    scores_ = np.concatenate(scores_, axis = 0)
    classes_ = np.concatenate(classes_, axis = 0)

    return boxes_, scores_, classes_


# ***********************************************************************
# Prepare the Overlay and load the "cnn.xmodel"
# ***********************************************************************
from pynq_dpu import DpuOverlay
from pynq import Overlay
from pynq.lib import AxiGPIO

overlay = DpuOverlay("/home/ubuntu/gst-dpu-test/dpu.bit")
overlay.load_model(cnn_xmodel)
ol = overlay
# ***********************************************************************
# Use VART APIs
# ***********************************************************************

dpu = overlay.runner
inputTensors = dpu.get_input_tensors()

print(" ")
print(inputTensors)
print(" ")

outputTensors = dpu.get_output_tensors()
shapeIn = tuple(inputTensors[0].dims)

print(" ")
print(shapeIn)
print(" ")
shapeOut0 = (tuple(outputTensors[0].dims)) # (1, 13, 13, 75)
shapeOut1 = (tuple(outputTensors[1].dims)) # (1, 26, 26, 75)
shapeOut2 = (tuple(outputTensors[2].dims)) # (1, 52, 52, 75)

outputSize0 = int(outputTensors[0].get_data_size() / shapeIn[0]) # 12675
outputSize1 = int(outputTensors[1].get_data_size() / shapeIn[0]) # 50700
outputSize2 = int(outputTensors[2].get_data_size() / shapeIn[0]) # 202800

input_data = [np.empty(shapeIn, dtype=np.float32, order="C")]
output_data = [np.empty(shapeOut0, dtype=np.float32, order="C"), 
               np.empty(shapeOut1, dtype=np.float32, order="C"),
               np.empty(shapeOut2, dtype=np.float32, order="C")]
image = input_data[0]


     

# ***********************************************************************
# Utility Functions (PWM)
# ***********************************************************************
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

def car_rotation(pulse, move_time):
    set_motor_B_pwm(pulse, 'forward')
    set_motor_A_pwm(pulse, 'reverse')
    time.sleep(move_time)
    set_motor_A_pwm(0, 'coast')
    set_motor_B_pwm(0, 'coast')

def arm_up_down(move_time):
    gpio_out.write(0x23,mask) #arm_forward
    time.sleep(move_time)
    gpio_out.write(0x43,mask) #arm_reverse
    time.sleep(move_time)
    gpio_out.write(0x03,mask) #arm_free

# Dictionary to hold detection timers for each section
detection_timers = {1: None, 2: None, 3: None, 4: None}
# Flags to track if the action has been triggered for each section
action_triggered = {1: False, 2: False, 3: False, 4: False}

rotation_90_time = 1
arm_move_time = 2
rotation_pulse = 50
detect_time = 2

def start_timer(section):
    if detection_timers[section] is None:
        detection_timers[section] = time.time()

def reset_timers(section):
    if section in detection_timers:
        detection_timers[section] = None
        action_triggered[section] = False  # Reset the action trigger for the specific section

def check_timers_and_trigger_actions():
    for section, start_time in detection_timers.items():
        if start_time and (time.time() - start_time) >= detect_time:
            print("time: {}".format(time.time() - start_time)) 
            if not action_triggered[section]:
                execute_action_A(section)
                action_triggered[section] = True

def execute_action_A(section):

    print(f"Action A triggered for section {section}")
    if section == 1:  #front
        arm_up_down(arm_move_time)
        # arm_thread = threading.Thread(target=arm_up_down, args=(arm_move_time,))
        # arm_thread.start()
    elif section == 2:  #right
        car_rotation(rotation_pulse, rotation_90_time)    #90° rotation
        # rotation_thread = threading.Thread(target=car_rotation, args=(rotation_pulse, rotation_90_time))
        # rotation_thread.start()
    elif section == 3:  #back
        car_rotation(rotation_pulse, 2*rotation_90_time)    #180° rotation
        # rotation_thread = threading.Thread(target=car_rotation, args=(rotation_pulse, 2*rotation_90_time))
        # rotation_thread.start()
    elif section == 4:  #left
        car_rotation(rotation_pulse, 3*rotation_90_time)    #270° rotation
        # rotation_thread = threading.Thread(target=car_rotation, args=(rotation_pulse, 3*rotation_90_time))
        # rotation_thread.start()

def run(input_image, section_i, display=False):
    # Read input image
    #input_image = cv2.imread(os.path.join(image_folder, original_images[image_index]))
    #input_image = cv2.imread(image_index)

    # Pre-processing
    print(input_image.shape)
    image_size = input_image.shape[:2]
    image_data = np.array(pre_process(input_image, (416, 416)), dtype=np.float32)
    
    # Fetch data to DPU and trigger it
    image[0,...] = image_data.reshape(shapeIn[1:])
    job_id = dpu.execute_async(input_data, output_data)
    dpu.wait(job_id)
    
    # Retrieve output data
    conv_out0 = np.reshape(output_data[0], shapeOut0)
    conv_out1 = np.reshape(output_data[1], shapeOut1)
    conv_out2 = np.reshape(output_data[2], shapeOut2)
    yolo_outputs = [conv_out0, conv_out1, conv_out2]
    
    # Decode output from YOLOv3
    boxes, scores, classes = evaluate(yolo_outputs, image_size, class_names, anchors)
    
    if display:
        _ = draw_boxes(input_image, boxes, scores, classes)


    print("section: {}".format(section_i))
    print("Number of detected objects: {}".format(len(boxes)))
    print("Scores: {}".format(scores))
    print("Details of detected objects: {}".format(classes))
    print(" ")
    
    detect_target = 0 # Human(0), Sports ball(32)
    print("detect_target: {}".format(detect_target))
    print("detection_timers: {}".format(detection_timers))
    print("action_triggered: {}".format(action_triggered))

    # if detect_target not in classes or (detect_target in classes and section_i is not None):
    if detect_target not in classes:
        reset_timers(section_i)

 
    if detect_target in classes:
        if section_i == 1:  #front
            if not all(action_triggered.values()):
                gpio_out.write(0x1F,mask) #All_led_on
            start_timer(section_i)
        elif section_i == 2:  #right
            if not all(action_triggered.values()):
                gpio_out.write(0x13,mask) #Red_led_on
            start_timer(section_i)
        elif section_i == 3:  #back
            if not all(action_triggered.values()):
                gpio_out.write(0x0B,mask) #Green_led_on
            start_timer(section_i)
        elif section_i == 4:  #left
            if not all(action_triggered.values()):
                gpio_out.write(0x07,mask) #Blue_led_on
            start_timer(section_i)
        
        # check_timers_and_trigger_actions()  
    # else:
    #     reset_timers()

    check_timers_and_trigger_actions()       


# Definition of the GStreamer pipeline (software)
pipeline = "thetauvcsrc mode=2K ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink"

# Initialize the VideoCapture object
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open the camera.")
else:
    print("The camera opened successfully.")

# Display True/False
display=True

# LED(GPIO)_set
gpio_0_ip = ol.ip_dict['axi_gpio_0']
gpio_out = AxiGPIO(gpio_0_ip).channel1
mask = 0xffffffff
gpio_out.write(0x03,mask) #2*motor_driver_enable

# Dummy image
if display:
    empty_image = 255 * np.ones((480, 480, 3), dtype=np.uint8)
    for i in range(4):
        cv2.imshow(f"Section {i+1}", empty_image)
    cv2.waitKey(15000) #15s wait

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Get the height and width of the image
    height, width, _ = frame.shape

    # Shift the image to the right by 240 pixels (to center the front in Section 1)
    shift = width // 8
    frame_shifted = np.roll(frame, shift, axis=1)

    # Split the shifted image into 4 sections 480:480
    sections = [
        frame_shifted[360:840, :width // 4],
        frame_shifted[360:840, width // 4:width // 2],
        frame_shifted[360:840, width // 2:3 * width // 4],
        frame_shifted[360:840, 3 * width // 4:]
    ]

    # Display each section   
    for i, section in enumerate(sections):
        run(section, i+1, display)

        if display:
            cv2.imshow(f"Section {i+1}", section)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        gpio_out.write(0x00,mask) #All_GPIO_off
        break
    
    # gpio_out.write(0x00,mask) #All_GPIO_off



cap.release()
cv2.destroyAllWindows()

# ***********************************************************************
# Clean up
# ***********************************************************************
del overlay
del dpu
