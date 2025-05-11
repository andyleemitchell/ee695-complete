import json
import os
import sys
import time

import cv2

import connect4_cv
from connect4_cv.state_validator import StateValidator

json_file = ".board_state.json"

temp_json = json_file + ".tmp"

video_capture = cv2.VideoCapture(-1)
if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)

detector = connect4_cv.ConnectFourStateDetector()
validator = StateValidator()

def do_single_detection():
    ret, frame = video_capture.read()
    board_state = detector.detect(frame=frame)

    json_state = {"board_state": board_state}
    with open(temp_json, mode="w", encoding="utf-8") as write_file:
        json.dump(json_state, write_file, indent=4)
    os.replace(temp_json, json_file)
    time.sleep(1)

def main():
    while True:
        print(50*"-")
        start = time.perf_counter()
        ret, frame = video_capture.read()
        board_state = detector.detect(frame=frame)
        board_state, change_needed = validator.get_valid_state(board_state=board_state)
        end = time.perf_counter()
        print(f" Time for detection and validation: {end-start}s")
        print(50*"-")

        if change_needed:
            print("json file change needed.....")
            json_state = {"board_state": board_state}
            with open(temp_json, mode="w", encoding="utf-8") as write_file:
                json.dump(json_state, write_file, indent=4)
            os.replace(temp_json, json_file)

        time.sleep(1/20)


if __name__ == "__main__":
    do_single_detection()
    main()
