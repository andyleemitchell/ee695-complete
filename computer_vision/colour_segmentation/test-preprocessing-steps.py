import time

import cv2
import numpy as np

import connect4_cv

def main():
    test_image = "./test-images/c4board.jpg"
    frame = cv2.imread(test_image)

    # Resize the image
    frame = cv2.resize(frame, (350, 300))

    # preprocessing:
    start_time = time.time()
    bilateral_filtered_image = cv2.bilateralFilter(frame, d=15, sigmaColor=100, sigmaSpace=100)
    end_time = time.time()
    print(f"Bilateral filtering took {end_time - start_time} seconds")


    # Convert to 16 colors
    Z = frame.reshape((-1, 3))
    Z = np.float32(Z)

    # Define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2)
    K = 6
    start_time = time.time()
    ret, label, center = cv2.kmeans(Z, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    # Convert back into uint8 and make original image
    center = np.uint8(center)
    res = center[label.flatten()]
    reduced_image = res.reshape((frame.shape))
    end_time = time.time()
    print(f"K-means clustering took {end_time - start_time} seconds")


    # Apply mean shift filtering
    start_time = time.time()
    mean_shift_image = cv2.pyrMeanShiftFiltering(frame, sp=20, sr=40)
    end_time = time.time()
    print(f"Mean shift filtering took {end_time - start_time} seconds")


    # Resize the images back to 700x600
    mean_shift_image = cv2.resize(mean_shift_image, (700, 600))
    bilateral_filtered_image = cv2.resize(bilateral_filtered_image, (700, 600))
    reduced_image = cv2.resize(reduced_image, (700, 600))

    cv2.imshow("mean_shift_image", mean_shift_image)
    cv2.imshow("bilateral_filtered_image", bilateral_filtered_image)
    cv2.imshow("reduced_image", reduced_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def detection(frame):
    detector.detect(frame=frame)

import cv2
import numpy as np

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


if __name__ == "__main__":
    detector = connect4_cv.ConnectFourStateDetector()
    # main()
    test_image = "./test-images/angle.jpg"
    frame = cv2.imread(test_image)

    # Resize the image
    start_time = time.time()
    frame = cv2.resize(frame, (700, 600))
    bilateral_filtered_image = cv2.bilateralFilter(frame, d=15, sigmaColor=200, sigmaSpace=200)
    detection(frame=frame)
    end_time = time.time()
    print(f"Detection took {end_time - start_time} seconds")

    red_mask = detector.red_mask
    yellow_mask = detector.yellow_mask
    frame_op = detector.frame
    frame_op = cv2.cvtColor(frame_op, cv2.COLOR_HSV2BGR)

    red_filtered = cv2.bitwise_and(frame_op, frame_op, mask=red_mask)
    yellow_filtered = cv2.bitwise_and(frame_op, frame_op, mask=yellow_mask)

    cv2.imshow("red_mask", red_mask)
    cv2.imshow("yellow_mask", yellow_mask)
    cv2.imshow("red_filtered", red_filtered)
    cv2.imshow("yellow_filtered", yellow_filtered)
    cv2.imshow("frame", frame_op)

    # start_time = time.time()
    # fixed_red_mask = close_gaps_and_refine_circles(red_mask)
    # fixed_yellow_mask = close_gaps_and_refine_circles(yellow_mask)
    # end_time = time.time()
    # print(f"Closing gaps and refining circles took {end_time - start_time} seconds")

    # cv2.imshow("red_mask", red_mask)
    # cv2.imshow("yellow_mask", yellow_mask)
    # cv2.imshow("fixed_red_mask", fixed_red_mask)
    # cv2.imshow("fixed_yellow_mask", fixed_yellow_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

