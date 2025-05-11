import cv2
import numpy as np


class QuadSelector:
    # """
    # Class to help with the selection of a quad from an image.
    # The quad can be any shape, and is draggable after creation.

    # TODO: Can this moved to a generic class that allows n-gon selection?
    #       We can then have derived classes for common gons, like tris and quads.
    # """

    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    DIST_THRESH = 10
    WIN_NAME = "QuadSelector"

    ENTER_KEYCODE = 13
    ESCAPE_KEYCODE = 27

    def __init__(self):
        # try:
        #     import cv2
        #     import numpy as np
        # except ImportError as e:
        #     print(f"Error importing a package. Please make sure it is installed. \n{e}")

        self.drawing = False  # Flag to track if we're actively drawing
        self.points = []  # List to store the four clicked points
        self.MAX_POINTS = 4  # Maximum points for a quad
        self.selected_point = None  # Index of the currently dragged point
        self.dragging = False  # Flag to indicate a point is being dragged

    def _points_to_nparray(self):
        """
        Converts points list of tuples to numpy array for use in polylines.
        """
        self.pts = np.array(self.points, np.int32)
        self.pts = self.pts.reshape((-1, 1, 2))

    def _dist_to_point(self, point, mouse):
        """
        Caclulates the pixel distance to a point from the current mouse location.
        """
        return np.sqrt((mouse[0] - point[0]) ** 2 + (mouse[1] - point[1]) ** 2)

    def draw_quad_cb(self, event, x, y, flags, param):
        """
        Callback function to draw quad on frame
        """
        temp_frame = self.frame.copy()

        # Draw circle at each point, and line from last point to current mouse pos
        # if less than 4 points
        for _, point in enumerate(self.points):
            cv2.circle(temp_frame, point, 5, self.GREEN, 1)
        if 0 < len(self.points) < self.MAX_POINTS:
            cv2.line(temp_frame, self.points[len(self.points) - 1], (x, y), self.GREEN, 1)

        # Draw quad between selected points
        if len(self.points) == self.MAX_POINTS:
            self._points_to_nparray()
            cv2.polylines(temp_frame, [self.pts], isClosed=True, color=self.RED, thickness=3)

        cv2.imshow(self.WIN_NAME, temp_frame)

        # Handle mouse events
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.points) < self.MAX_POINTS:
                self.points.append((x, y))
            elif len(self.points) == self.MAX_POINTS:
                for i, point in enumerate(self.points):
                    if self._dist_to_point(point, (x, y)) < self.DIST_THRESH:
                        self.selected_point = i
                        self.dragging = True

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.dragging:
                self.points[self.selected_point] = (x, y)
                temp_frame = self.frame.copy()
                self._points_to_nparray()
                cv2.polylines(temp_frame, [self.pts], isClosed=True, color=self.RED, thickness=3)

        elif event == cv2.EVENT_LBUTTONUP:
            if self.dragging:
                self.dragging = False
                temp_frame = self.frame.copy()
                self._points_to_nparray()
                cv2.polylines(temp_frame, [self.pts], isClosed=True, color=self.RED, thickness=3)
                cv2.imshow(self.WIN_NAME, temp_frame)

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.points = []
            self.selected_point = None
            self.dragging = False
            temp_frame = self.frame.copy()
            cv2.imshow(self.WIN_NAME, temp_frame)



    def select_points(self, frame):
        """
        Loads the QuadSelector on the frame. Returns the selected quad points.

        Press q or escape to return no points.
        Press enter to confirm the points and return them.

        Left click will add points. Once all four points have been added, the points are draggable
        with the left mouse key.
        Right click will clear all points.
        """
        self.frame = frame
        if self.frame is None:
            print("Failed to load image")

        # Create our windows to do selection in
        cv2.namedWindow(self.WIN_NAME)
        cv2.setMouseCallback(self.WIN_NAME, self.draw_quad_cb)
        cv2.imshow(self.WIN_NAME, self.frame)

        while True:
            key = cv2.waitKey(1) & 0xFF
            # TODO: other keys???
            if key == ord("q") or key == self.ESCAPE_KEYCODE:
                cv2.destroyAllWindows()
                return None
            if key == self.ENTER_KEYCODE:
                cv2.destroyAllWindows()
                return self.points


if __name__ == "__main__":
    QS = QuadSelector()
    video_capture = cv2.VideoCapture(0)

    if not video_capture.isOpened():
        msg = "Could not open the camera."
        raise RuntimeError(msg)

    ret, frame = video_capture.read()
    if not ret:
        print("Failed to grab frame")

    points = QS.select_points(frame)
    print(points if points else "Quit")

