import datetime
import os

import cv2


def collect_images(output_dir, num_images=50):

    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize the webcam
    camera = cv2.VideoCapture(-1)  # 0 is usually the default webcam.  Try 1, 2, etc. if needed
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    if not camera.isOpened():
        print("Error: Could not open webcam.  Check device availability.")
        return

    image_count = 0

    try:
        while image_count < num_images:
            # Capture frame-by-frame
            ret, frame = camera.read()

            if not ret:
                print("Error: Failed to capture frame. Check webcam connection or settings.")
                break  # Exit the loop if no frame is read


            # Display the resulting frame
            cv2.imshow("Press Enter to Capture", frame)

            # Wait for 'Enter' key press (ASCII code 13)
            key = cv2.waitKey(1)  # Wait 1 millisecond.  Adjust if needed.

            if key == 13:  # Enter key pressed
                # Save the image
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                base_filename = f"image_{timestamp}_{image_count:04d}"
                image_filename = os.path.join(output_dir, f"{base_filename}.jpg")
                # frame = cv2.resize(frame, (640, 640))
                cv2.imwrite(image_filename, frame)
                print(f"Saved: {image_filename}")
                image_count += 1


            elif key == ord('q'):  # Press 'q' to quit early (optional)
                print("Quitting early.")
                break

    finally: # Important to release the camera resource even if errors occur
        # When everything done, release the capture
        camera.release()
        cv2.destroyAllWindows()

    print(f"Collected {image_count} images.")

if __name__ == "__main__":
    collect_images(output_dir="./dataset_acquisition/raw_images", num_images=4) # Collect 50 images in 'images' directory by default
    # Example usage to collect 25 images in a directory named 'my_dataset':
    # collect_images(output_dir="my_dataset", num_images=25)
