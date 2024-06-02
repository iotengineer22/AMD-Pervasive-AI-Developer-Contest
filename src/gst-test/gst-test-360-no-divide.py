import cv2
import numpy as np
import time

# Definition of the GStreamer pipeline (software)
pipeline = "thetauvcsrc mode=2K ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink"
# Definition of the GStreamer pipeline (hardware)
#pipeline = "thetauvcsrc mode=2K ! h264parse ! omxh264dec ! queue ! videoconvert ! video/x-raw,format=BGR ! appsink"

# Initialize the VideoCapture object
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open the camera.")
else:
    print("The camera opened successfully.")

# Initialize variables for FPS calculation
frame_count = 0
start_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to receive the frame.")
        break

    # Display the full frame
    cv2.imshow("Full Frame", frame)

    # Update frame count
    frame_count += 1

    # Check the time every 100 frames
    if frame_count % 100 == 0:
        end_time = time.time()
        elapsed_time = end_time - start_time
        fps = frame_count / elapsed_time
        print("FPS:", fps)
        # Reset the timer and frame count
        frame_count = 0
        start_time = time.time()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Exiting...")
        break

# Post-processing
cap.release()
cv2.destroyAllWindows()

