import cv2

video_capture = cv2.VideoCapture(-1)
if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)


while True:
    ret, frame = video_capture.read()

    detector = cv2.aruco.ArucoDetector(dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))

    corners, ids, rejected = detector.detectMarkers(frame)

    cv2.aruco.drawDetectedMarkers(frame, corners=corners, ids=ids)
    # print(points)

    cv2.imshow("frame", frame)
    cv2.waitKey(1)  


