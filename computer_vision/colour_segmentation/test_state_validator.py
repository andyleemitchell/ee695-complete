import json

from connect4_cv.state_validator import StateValidator

import time

TEST_DIR = "./test-validator/"
BASE_STATE = TEST_DIR + "base_state.json"
VALID_1 = TEST_DIR + "valid_1_diff.json"
VALID_3 = TEST_DIR + "valid_3_diff.json"
INVALID = TEST_DIR + "invalid_diff.json"

state_validator = StateValidator(json_file=BASE_STATE)

def load_state_from_file(file):
    with open(file, encoding="utf-8") as read_file:
            return json.load(read_file)["board_state"]

def main():
    start = time.perf_counter()
    check_state = load_state_from_file(INVALID)
    valid_state = state_validator.get_valid_state(check_state)
    end = time.perf_counter()

    print(f"Elapsed time: {1000*(end-start)}ms")

if __name__ == "__main__":
    main()
