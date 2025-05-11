# use one of the methods to bootstrap the ground truth for each image

import os

import cv2

from colour_seg_method import ConnectFourStateDetector
from pathlib import Path

CURRENT_DIR = os.getcwd()+"/comparison_test"


images = [image for image in Path(CURRENT_DIR+"/images").glob("*.jpg")]

images = sorted(images, key=lambda p: int(p.stem))

print(str(images[0]))
# exit()

detector = ConnectFourStateDetector()

calibration_image = cv2.imread(str(images[20]))
detector.calibrate(frame=calibration_image)

# for image in images:
#     frame = cv2.imread(image)
#     detector.detect(frame=frame, output_file=f"{CURRENT_DIR}/truth/{str(image.stem)}.json")
#     print(f"Done image {image.stem}.")


