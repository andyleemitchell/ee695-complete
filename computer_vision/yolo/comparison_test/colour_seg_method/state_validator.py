import json
import sys
from pathlib import Path


class StateValidator:
    """
    need to check if only one thing has changed.
    check if change is valid, i.e. gravity.
    maybe check the average (?) of previous X states?
    return the valid state.

    note: right now the validator will fail when the board is cleared.
    can we do a check for this somehow? might be difficult to differentiate
    between clearing a single game piece and a false detection however.
    """

    DEFAULT_STATE_FILE = ".board_state.json"


    def __init__(self, json_file="", initial_state=None):
        if json_file:
            self.json_file = json_file
        else:
            self.json_file = self.DEFAULT_STATE_FILE

        if not Path(self.json_file).is_file():
            print("JSON output file doesn't exist. Please run the detection first.")
            if initial_state is None:
                print("No initial state provided either. Exiting.")
                sys.exit()

        if initial_state is None:
            self._read_json()
        else:
            self.board_state = initial_state
        self.initial_state = self.board_state
        self.valid_state = self.board_state
        self.comparison_state = self.valid_state
        self.move_made = None

    def _read_json(self):
        with open(self.json_file, encoding="utf-8") as read_file:
            self.board_state = json.load(read_file)["board_state"]

    def _one_move_made(self):
        count = 0
        for i in range(len(self.valid_state)):
            for j in range(len(self.valid_state[i])):
                if self.valid_state[i][j] != self.comparison_state[i][j]:
                    count += 1
                    self.move_made = (i, j)

        return count == 1

    def _is_move_valid(self):
        valid = False
        if self.move_made is not None:
            print(f"Move made: [{self.move_made[0]}][{self.move_made[1]}]")
        i, j = self.move_made
        old_cell = self.valid_state[i][j]
        new_cell = self.comparison_state[i][j]
        num_rows = len(self.valid_state)
        # check that we haven't replaced a counter
        if old_cell == "empty":
            print("Replacing an empty slot.")
            valid = True
        elif old_cell in ["red", "yellow"] and new_cell == "empty":
            # print("Valid counter has disappeared? Nuh-uh.")
            return False
        else:
            valid = False

        if i == num_rows - 1:
            print("New move on bottom row.")
            return True
        cell_under_move = self.valid_state[i+1][j]
        # check that we don't have a floating counter
        if cell_under_move != "empty":  # noqa: SIM108
            # print("New move is not floating.")
            valid = True
        else:
            # print("New move is floating.")
            valid = False

        return valid

    def _is_state_same(self):
        return self.valid_state == self.comparison_state

    def _is_new_state_valid(self):
        if self._is_state_same():
            # print("States are the same.")
            return False
        # print("States are different.")

        if not self._one_move_made():
            # print("More than one change.")
            return False
        # print("Only one change.")

        return self._is_move_valid()

    def reset_states(self):
        self.board_state = self.initial_state
        self.comparison_state = self.initial_state

    def get_valid_state(self, board_state):
        change_needed = False
        self.comparison_state = board_state
        if self._is_new_state_valid():
            print("Valid update to board state.")
            self.valid_state = self.comparison_state
            self.move_made = None
            change_needed = True
            return self.valid_state, change_needed
        print("Update to board state is not valid.")
        self.move_made = None
        return self.valid_state, change_needed
