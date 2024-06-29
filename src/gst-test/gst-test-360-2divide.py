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

while True:
    start_time = time.time()
    ret, frame = cap.read()
    end_time = time.time()
    if not ret:
        break

    # Get the height and width of the image
    height, width, _ = frame.shape

    # Shift the image to the right by 480 pixels
    shift = width // 4
    frame_shifted = np.roll(frame, shift, axis=1)

    # Split the shifted image into 4 sections
    sections = [
        frame_shifted[:, :width // 2],                  #front
        frame_shifted[:, width // 2:]                  #back
    ]

    # Display each section
    for i, section in enumerate(sections):
        cv2.imshow(f"Section {i+1}", section)

    
    print("360 Camera caputure time: {:.4f} seconds".format(end_time - start_time))
    print("Camera Performance: {} FPS".format(1/(end_time - start_time)))
    print(" ")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# Post-processing
cap.release()
cv2.destroyAllWindows()
