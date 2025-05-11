import cv2
import numpy as np


class ColourThresholder:
    ENTER_KEYCODE = 13
    ESCAPE_KEYCODE = 27

    def __init__(self):
        self._setup_window()

    def _setup_window(self, window_name="Colour Thresholder"):
        self.window_name = window_name
        cv2.namedWindow(self.window_name)

        cv2.createTrackbar("h_min", self.window_name, 0, 179, self._nop)
        cv2.createTrackbar("s_min", self.window_name, 0, 255, self._nop)
        cv2.createTrackbar("v_min", self.window_name, 0, 255, self._nop)
        cv2.createTrackbar("h_max", self.window_name, 0, 179, self._nop)
        cv2.createTrackbar("s_max", self.window_name, 0, 255, self._nop)
        cv2.createTrackbar("v_max", self.window_name, 0, 255, self._nop)

        cv2.setTrackbarPos("h_max", self.window_name, 179)
        cv2.setTrackbarPos("s_max", self.window_name, 255)
        cv2.setTrackbarPos("v_max", self.window_name, 255)

    def _nop(self, _):
        """
        Need a callback function to create trackbars, but it doesn't have to do anything.
        """

    def _update_values(self):
        self.h_min = cv2.getTrackbarPos("h_min", self.window_name)
        self.s_min = cv2.getTrackbarPos("s_min", self.window_name)
        self.v_min = cv2.getTrackbarPos("v_min", self.window_name)
        self.h_max = cv2.getTrackbarPos("h_max", self.window_name)
        self.s_max = cv2.getTrackbarPos("s_max", self.window_name)
        self.v_max = cv2.getTrackbarPos("v_max", self.window_name)

        self.values = [
            (self.h_min, self.s_min, self.v_min),
            (self.h_max, self.s_max, self.v_max)
        ]

    def _do_threshold(self):
        lower_threshold = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
        upper_threshold = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
        mask = cv2.inRange(self.hsv_frame, lower_threshold, upper_threshold)
        self.output_frame = cv2.bitwise_and(self.frame, self.frame, mask=mask)

    def run(self, frame):
        self.frame = frame
        if self.frame is None:
            print("Error with frame.")

        self.output_frame = self.frame
        self.hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        shift = 50
        hsv_shifted = self.hsv_frame.copy()
        hsv_shifted[:, :, 0] = (hsv_shifted[:, :, 0] + shift) % 180
        self.hsv_frame = hsv_shifted.copy()

        while True:
            self._update_values()
            self._do_threshold()
            cv2.imshow(self.window_name, self.output_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") or key == self.ESCAPE_KEYCODE:
                cv2.destroyAllWindows()
                return None
            if key == self.ENTER_KEYCODE:
                cv2.destroyAllWindows()
                return self.values



if __name__ == "__main__":
    CT = ColourThresholder()
    video_capture = cv2.VideoCapture(0)

    if not video_capture.isOpened():
        msg = "Could not open the camera."
        raise RuntimeError(msg)

    ret, frame = video_capture.read()
    if not ret:
        print("Failed to grab frame")

    values = CT.run(frame=frame)
    print(values if values else "Quit")
