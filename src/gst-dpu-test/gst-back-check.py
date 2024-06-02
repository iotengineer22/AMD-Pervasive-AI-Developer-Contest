import cv2

# Retrieve OpenCV build information
build_info = cv2.getBuildInformation()

# Print the GStreamer related information
print(build_info)
print("GStreamer:")
print([line for line in build_info.split('\n') if 'GStreamer' in line][0])


cap = cv2.VideoCapture("videotestsrc ! videoconvert ! appsink")

while True:
    ret, img = cap.read()
    if not ret:
        break

    cv2.imshow("",img)
    key = cv2.waitKey(1)
    if key==27: #[esc] key
        break