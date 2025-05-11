import json
import multiprocessing
import os
import time
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


def loop_1():
    state_display = connect4_cv.StateDisplay()
    while True:
        state_display.update()

json_file = ".board_state.json"
json_temp = ".board_state.json.tmp"

def loop_2():
    while True:
        ret, frame = video_capture.read()
        # start = perf_counter()
        board_state = detector.detect(frame=frame)
        # end = perf_counter()

        # print(f"{end-start:.6f} seconds -- {1/(end-start):.1f} fps")

        json_state = {"board_state": board_state}

        with open(json_temp, mode="w", encoding="utf-8") as write_file:
            json.dump(json_state, write_file, indent=4)

        os.replace(json_temp, json_file)

        sleep(1 / 10)


def display_window_loop():
    while True:
        # Do non opencv stuff that might take time.
        time.sleep(1)
        print("main loop: Doing other work.")


if __name__ == "__main__":
    # Create process for each camera
    process_1 = multiprocessing.Process(target=loop_1)  # Camera index 0
    process_2 = multiprocessing.Process(target=loop_2)  # Camera index 1

    process_1.start()
    process_2.start()

    try:
        display_window_loop()
    except KeyboardInterrupt:
        print("program ended")

    process_1.join()
    process_2.join()
    print("all processes joined")
