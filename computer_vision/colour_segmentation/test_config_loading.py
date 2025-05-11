import cv2

from connect4_cv import ConfigLoader, QuadSelector

QS = QuadSelector()
video_capture = cv2.VideoCapture(0)

if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)

ret, frame = video_capture.read()
if not ret:
    print("Failed to grab frame")

points = QS.select_points(frame)
# print(points if points else "Quit")

# let's write the config

config = {"points": points}

# ConfigLoader.write_json("./conf.json", config=config)

conf_loaded = ConfigLoader.load_json("./conf.json")
print(conf_loaded)
