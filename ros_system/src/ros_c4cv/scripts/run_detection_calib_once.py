import json
import os
import sys
import time
from argparse import ArgumentParser

import cv2

import connect4_cv
import numpy as np

parser = ArgumentParser()
parser.add_argument(
    "-c",
    "--calibrate",
    help="run calibration before doing any detection",
    action="store_true",
    default=False,
)
parser.add_argument(
    "-f", "--file", help="file to store board state in", default=".board_state.json"
)
args = parser.parse_args()

temp_json = args.file + ".tmp"

video_capture = cv2.VideoCapture(-1)
if not video_capture.isOpened():
    msg = "Could not open the camera."
    raise RuntimeError(msg)

detector = connect4_cv.ConnectFourStateDetector(noise_lim=100)


def close_gaps_and_refine_circles(mask):
    # 1. Input Validation (Important!)
    if mask is None:
        raise ValueError("Input mask cannot be None.")
    if mask.ndim != 2:
        raise ValueError("Input mask must be a 2D grayscale image.")
    if mask.dtype != np.uint8:
        # Convert to uint8 if not already.  Handle potential clipping/overflow.
        mask = np.clip(mask, 0, 255).astype(np.uint8)
        # Alternatively, you could raise an error:
        # raise ValueError("Input mask must be of type np.uint8.")

    # 2. Initial Closing (Morphological)
    kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_small)

    # 3.  Find Contours (after initial closing)
    contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 4. Create a new mask for refined circles
    refined_mask = np.zeros_like(mask)

    # 5. Iterate through contours and fit circles
    for contour in contours:
        if len(contour) >= 5:
            (x, y), (MA, ma), angle = cv2.fitEllipse(contour)
            aspect_ratio = MA / ma if ma != 0 else 0
            if 0.7 < aspect_ratio < 1.3:
                radius = int((MA + ma) / 4)
                cv2.circle(refined_mask, (int(x), int(y)), radius, 255, -1)
            else:
                cv2.drawContours(refined_mask, [contour], 0, 255, -1)
        else:
            cv2.drawContours(refined_mask, [contour], 0, 255, -1)

    # 6. Final Closing (Optional)
    kernel_final = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    final_mask = cv2.morphologyEx(refined_mask, cv2.MORPH_CLOSE, kernel_final)

    return final_mask


def process_loop():
    # while True:
    ret, frame = video_capture.read()
    height, width, _ = frame.shape
    print(f"Image dimensions: {width}x{height}")

    start_detection_time = time.perf_counter()
    # small_frame = cv2.resize(frame, (frame.shape[1] // 4, frame.shape[0] // 4))
    # # small_frame = cv2.pyrMeanShiftFiltering(small_frame, sp=20, sr=20)
    # # small_frame = cv2.bilateralFilter(small_frame, 15, 200, 200)
    # # small_frame = cv2.GaussianBlur(small_frame, (5, 5), 0)

    # small_frame = cv2.pyrMeanShiftFiltering(small_frame, sp=20, sr=40)
    # small_frame = cv2.bilateralFilter(small_frame, d=15, sigmaColor=100, sigmaSpace=100)
    
    # Convert to 16 colors
    # Z = small_frame.reshape((-1, 3))
    # Z = np.float32(Z)
    # # Define criteria, number of clusters(K) and apply kmeans()
    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    # K = 10
    # ret, label, center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    # # Convert back into uint8 and make original image
    # center = np.uint8(center)
    # res = center[label.flatten()]
    # reduced_image = res.reshape((small_frame.shape))
    # # reduced_image = cv2.resize(reduced_image, (frame.shape[1], frame.shape[0]))

    # frame = cv2.resize(small_frame, (frame.shape[1], frame.shape[0]))
    # frame = cv2.GaussianBlur(src=frame, ksize=(25, 25), sigmaX=0)
    board_state = detector.detect(frame=frame)
    end_detection_time = time.perf_counter()
    print(f"Detection execution time: {end_detection_time - start_detection_time} seconds")

    # red_mask = close_gaps_and_refine_circles(detector.red_mask)
    # yellow_mask = close_gaps_and_refine_circles(detector.yellow_mask)
    # cv2.imshow("red mask morphed", red_mask)
    # cv2.imshow("yellow mask morphed", yellow_mask)

    cv2.imshow("red mask", detector.red_mask)
    cv2.imshow("yellow mask", detector.yellow_mask)
    cv2.imshow("frame", frame)
    # cv2.imshow("reduced image", reduced_image)
    cv2.waitKey(0)

    json_state = {"board_state": board_state}

    with open(temp_json, mode="w", encoding="utf-8") as write_file:
        json.dump(json_state, write_file, indent=4)

    os.replace(temp_json, args.file)

    # time.sleep(1/20)
    for row in board_state:
        print(row)


if args.calibrate:
    _, frame = video_capture.read()
    # small_frame = cv2.resize(frame, (frame.shape[1] // 4, frame.shape[0] // 4))
    # # small_frame = cv2.bilateralFilter(small_frame, 15, 200, 200)
    # small_frame = cv2.pyrMeanShiftFiltering(small_frame, sp=20, sr=40)
    # small_frame = cv2.bilateralFilter(small_frame, d=15, sigmaColor=100, sigmaSpace=100)

    # Z = small_frame.reshape((-1, 3))
    # Z = np.float32(Z)
    # # Define criteria, number of clusters(K) and apply kmeans()
    # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    # K = 10
    # ret, label, center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
    # # Convert back into uint8 and make original image
    # center = np.uint8(center)
    # res = center[label.flatten()]
    # reduced_image = res.reshape((small_frame.shape))

    # small_frame = cv2.pyrMeanShiftFiltering(small_frame, sp=20, sr=20)
    # frame = cv2.resize(small_frame, (frame.shape[1], frame.shape[0]))
    # frame = cv2.GaussianBlur(src=frame, ksize=(25, 25), sigmaX=0)
    detector.calibrate(frame=frame, force=True)
    print("saving configuration........")
    time.sleep(0.5)
    sys.exit()


if __name__ == "__main__":
    start_time = time.perf_counter()
    process_loop()
    end_time = time.perf_counter()
    print(f"Process loop execution time: {end_time - start_time} seconds")
