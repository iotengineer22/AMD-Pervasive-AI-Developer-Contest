import cv2
import numpy as np

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
    ret, frame = cap.read()
    if not ret:
        break

    # Get the height and width of the image
    height, width, _ = frame.shape

    # Shift the image to the right by 240 pixels (to center the front in Section 1)
    shift = width // 8
    frame_shifted = np.roll(frame, shift, axis=1)

    # Split the shifted image into 4 sections
    sections = [
        frame_shifted[360:840, :width // 4],
        frame_shifted[360:840, width // 4:width // 2],
        frame_shifted[360:840, width // 2:3 * width // 4],
        frame_shifted[360:840, 3 * width // 4:]
    ]

    # Display each section
    for i, section in enumerate(sections):
        cv2.imshow(f"Section {i+1}", section)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# Post-processing
cap.release()
cv2.destroyAllWindows()
