import json
import multiprocessing
import os
import time
from argparse import ArgumentParser

import cv2

import connect4_cv

display = connect4_cv.StateDisplay()

def display_loop():
    while True:
        display.update()


if __name__ == "__main__":
    display_loop()
