import json
from time import perf_counter, sleep

import cv2

import connect4_cv

detector = connect4_cv.ConnectFourStateDetector()


video_capture = cv2.VideoCapture(0)
if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)
ret, frame = video_capture.read()
if not ret:
    print("Failed to grab frame")


start = perf_counter()
board_state = detector.detect(frame=frame)
end = perf_counter()

print(f"{end-start:.6f} seconds -- {1/(end-start):.1f} fps")


# print(board_state)

json_state = {"board_state": board_state}

with open(".board_state.json", mode="w", encoding="utf-8") as write_file:
    json.dump(json_state, write_file, indent=4)


# sleep(2)

ret, frame = video_capture.read()
start = perf_counter()
board_state = detector.detect(frame=frame)
end = perf_counter()

print(f"{end-start:.6f} seconds -- {1/(end-start):.1f} fps")


while True:
    ret, frame = video_capture.read()
    start = perf_counter()
    board_state = detector.detect(frame=frame)
    end = perf_counter()

    print(f"{end-start:.6f} seconds -- {1/(end-start):.1f} fps")
    sleep(1/10)
