from pathlib import Path
import os
import cv2
from time import perf_counter
import json
from numpy import imag
from tqdm.auto import tqdm

from colour_seg_method import ConnectFourStateDetector
from yolo_method.yolo_detector import ConnectFourYoloDetector

CURRENT_DIR = os.getcwd()
TEST_DATA_DIR = CURRENT_DIR+"/comparison_test"
IMAGES_DIR = TEST_DATA_DIR+"/images"
GROUND_TRUTH_DIR = TEST_DATA_DIR+"/truth"
TEST_RESULTS_DIR = TEST_DATA_DIR+"/test_results"
# TEST_CONFIG = []

class CvComparisonTest:
    def __init__(self, test_name):
        self.test_name = test_name
        self.successes_count = 0
        self.oneoffs_count  = 0
        self.failures_count  = 0

        self.successful_images = []
        self.oneoff_images = []
        self.failing_images = []

        self.detection_times = []

        self.func = None

        self.init_test()

    def init_test(self):
        images = [image for image in Path(IMAGES_DIR).glob("*.jpg")]
        ground_truths = [json_file for json_file in Path(GROUND_TRUTH_DIR).glob("*.json")]

        self.images = sorted(images, key=lambda p: int(p.stem))
        self.ground_truths = sorted(ground_truths, key=lambda p: int(p.stem))
    
    def register_detection_func(self, function):
        self.func = function

    def _read_json_board_state(self, json_file):
        with open(json_file, encoding="utf-8") as read_file:
            return json.load(read_file)["board_state"]
        
    def _state_is_same(self, truth, detected):
        return truth == detected
    
    def _num_cells_diff(self, truth, detected):
        count = 0
        for i in range(len(truth)):
            for j in range(len(truth[i])):
                if truth[i][j] != detected[i][j]:
                    count += 1
                    self.move_made = (i, j)
        return count

    def _print_states(self, truth, detected):
        for i in range(len(truth)):
            for j in range(len(truth[i])):
                print(f"{truth[i][j]}/{detected[i][j]}", end="\t")
            print()


    def run_test(self):
        if self.func is None:
            print("Please register the detection function for this test first.")
            return
        
        self.num_detections = 0
        self.skipped_images = []
        for image, truth in zip(tqdm(self.images), self.ground_truths):
            # print(f"Doing detection on: {image.name} (ground truth in {truth.name})")
            test_frame = cv2.imread(str(image))
            start_detection_time = perf_counter()
            board_state = self.func(test_frame)
            end_detection_time = perf_counter()
            self.detection_times.append(end_detection_time-start_detection_time)
            if board_state is None:
                self.skipped_images.append(image.name)
                continue
            self.num_detections += 1
            # compare with ground truth...
            ground_truth = self._read_json_board_state(str(truth))
            if self._state_is_same(ground_truth, board_state):
                self.successes_count += 1
                self.successful_images.append(image.name)
            different_cells = self._num_cells_diff(ground_truth, board_state)
            if different_cells == 1:
                self.oneoffs_count += 1
                self.oneoff_images.append(image.name)
            elif different_cells > 1:
                self.failures_count += 1
                self.failing_images.append(image.name)
            
            # self._print_states(ground_truth, board_state)

        # print(f"detections num: {self.num_detections}")
        # print(self.skipped_images)

    def test_results(self, print_results=True):
        success_rate = round((self.successes_count / self.num_detections) * 100, 2)
        oneoff_rate = round((self.oneoffs_count / self.num_detections) * 100, 2)
        failure_rate = round((self.failures_count / self.num_detections) * 100, 2)
        average_time = sum(self.detection_times)/len(self.detection_times)
        average_fps = 1/average_time

        test_results = {
            "name":                 self.test_name,
            "detections":           self.num_detections,
            "successes":            self.successes_count,
            "success_rate":         success_rate,
            "failures":             self.failures_count,
            "failure_rate":         failure_rate,
            "oneoffs":              self.oneoffs_count,
            "oneoff_rate":          oneoff_rate,
            "avg_detection_time":   average_time,
            "avg_fps":              average_fps,
        }

        if print_results:
            print()
            print(19*"-" + " Results " + 19*"-")
            # print(47*"-")
            print(f" Test: {self.test_name}")
            print(47*"=")
            print(f" Total Successes:\t\t{self.successes_count:2.0f} ({success_rate}%)")
            print(f" Total One-Offs: \t\t{self.oneoffs_count:2.0f} ({oneoff_rate}%)")
            print(f" Total Failures: \t\t{self.failures_count:2.0f} ({failure_rate}%)")
            print(f" Avg time per detection: \t{round(average_time*1000, 2)}ms")
            print(f" Avg FPS:\t\t\t{round(average_fps, 1)}fps")
            print(47*"-")

        # print(self.failing_images)

        return test_results

if __name__ == "__main__":
    # Colour Segmentation
    colour_seg_1 = {
        "detector": ConnectFourStateDetector(),
        "name": "Default Colour Segmentation",
    }
    colour_seg_2 = {
        "detector": ConnectFourStateDetector(noise_lim=200),
        "name": "Colour Segmentation (noise_lim=200)",
    }
    colour_seg_3 = {
        "detector": ConnectFourStateDetector(noise_lim=3000),
        "name": "Colour Segmentation (noise_lim=3000)",
    }
    
    # YOLO
    yolo_1 = {
        "detector": ConnectFourYoloDetector(conf_thresh=0.9),
        "name": "YOLO (threshold=0.9)"
    }
    yolo_2 = {
        "detector": ConnectFourYoloDetector(conf_thresh=0.75),
        "name": "YOLO (threshold=0.75)"
    }
    yolo_3 = {
        "detector": ConnectFourYoloDetector(conf_thresh=0.4),
        "name": "YOLO (threshold=0.3)"
    }

    # Run Tests
    tests_to_run = [colour_seg_1, colour_seg_2, colour_seg_3, yolo_1, yolo_2, yolo_3]
    tests = []
    for i, test in enumerate(tests_to_run):
        tests.append(CvComparisonTest(test_name=test["name"]))
        tests[i].register_detection_func(test["detector"].detect)
        tests[i].run_test()

    # Display results
    for test in tests:
        results = test.test_results()
        results_file = TEST_RESULTS_DIR+"/"+\
            test.test_name.replace(" ", "_")\
                          .replace("(", "")\
                          .replace(")", "")\
                          .replace("=", "")\
                          .lower()\
                            +".json"
        with open(results_file, "w") as f:
            json.dump(results, f, sort_keys=False, indent=4)
