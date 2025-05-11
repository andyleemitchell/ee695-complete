import cv2
from ultralytics import YOLO
from time import perf_counter
import numpy as np

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)
WHITE = (255, 255, 255)

cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

if not cap.isOpened():
    print("error in opening camera")
    exit()

# Load your YOLOv8n model (ensure you provide the correct path to the .pt file)
model = YOLO('./weights/april16.pt')
# model = YOLO('./weights/march6th-online-ds.pt')
# model = YOLO('./weights/custom_dataset_27march.pt')
print("loaded model")
# Get the class names from the model
class_names = model.names
print("Possible classes:", class_names)

target_classes = ['Red Piece', 'Yellow Piece', 'No Piece']
target_class_ids = [class_id for class_id, class_name in class_names.items() if class_name in target_classes]

print(target_class_ids)

while True:
    ret, frame = cap.read()
    if not ret:
        print("error in getting frame")
        exit()

    # Load the image
    # image_path = 'images/c4-board1.jpg'
    # image = cv2.imread(image_path)
    image = frame
    im_height, im_width, _ = image.shape
    image = cv2.resize(image, (im_width // 2, im_height // 2))

    # Perform inference on the image
    results = model(image)

    # grid_rows = 6
    # grid_cols = 7
    # grid_state = np.zeros((grid_rows, grid_cols), dtype=object)  # 3x3 grid state (None means empty)

    # Extract detection details
    for result in results[0].boxes.data:  # Each detection (x, y, width, height, confidence, class_id)
        x1, y1, x2, y2, confidence, class_id = result
        
        if int(class_id) in target_class_ids and float(confidence) > 0.4:
            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)
            # Convert from center x, y to top-left and bottom-right corners
            # x1 = int(x_center - width / 2)
            # y1 = int(y_center - height / 2)
            # x2 = int(x_center + width / 2)
            # y2 = int(y_center + height / 2)

            # row = int(y1 // (im_height / grid_rows))
            # col = int(x1 // (im_width / grid_cols))
            # print(f"{row}-{col}")

            # grid_state[row, col] = {
            #     'class': int(class_id),
            #     'confidence': confidence
            # }

            # print(x_center, y_center, width, height, confidence, class_id)
            # print(x1, y1, x2, y2)
            
            # Draw the bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 1)  # Green box with thickness 2
            
            # Prepare the label with class ID and confidence
            name = class_names[int(class_id)]
            if name == 'No Piece':
                name = "E"
            elif name == 'Yellow Piece':
                name = "Y"
            elif name == 'Red Piece':
                name = "R"
            label = f"{name}"
            
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(image, (x1, y1 - 20), (x1 + text_width, y1), (0, 0, 0), -1)
            # Add label to the image
            cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # print("Grid state:")
    # for r in range(grid_rows):
    #     for c in range(grid_cols):
    #         print(f"Cell ({r},{c}): {grid_state[r, c]}")
    state = []

    confidence_threshold = 0.4

    # List of detections with (x, y) points (center coordinates of bounding boxes)
    detections = [((result[0]+result[2])//2, (result[1]+result[3])//2, result[4], result[5]) for result in results[0].boxes.data]
    filtered_detections = [d for d in detections if d[2] >= confidence_threshold]
    filtered_detections = [d for d in detections if int(d[3]) in target_class_ids]
    # Sort the detections by the y coordinate (ascending order)
    sorted_by_y = sorted(filtered_detections, key=lambda d: d[1])

    for row in range(6):
    # Get the top 7 detections with the smallest y (top row of the grid)
        top_row_detections = sorted_by_y[row*7+0:row*7+7]
        # print(top_row_detections)

        # Sort the top row detections by the x coordinate (ascending order)
        top_row_sorted_by_x = sorted(top_row_detections, key=lambda d: d[0])

        for x in top_row_sorted_by_x:
            name = class_names[int(x[3])]
            if name == 'No Piece':
                state.append("empty")
            elif name == 'Yellow Piece':
                state.append("yellow")
            elif name == 'Red Piece':
                state.append("red")
        #     print(class_names[int(x[3])], end="\t")
        # print("")
    display_image = np.zeros((100*6, 100*7, 3), np.uint8)
    cv2.rectangle(display_image, (0,0), (100*7, 100*6), BLUE, -1)
    if len(state) == 42:
        for row in range(6):
            for col in range(7):
                print(state[row*7 + col], end="\t")
            print()

        for y in range(6):
            for x in range(7):
                cell_x = 100*x+50
                cell_y = 100*y+50
                cell = state[y*7 + x]
                if cell == "red":
                    cv2.circle(display_image, (cell_x, cell_y), 40, RED, -1)
                elif cell == "yellow":
                    cv2.circle(display_image, (cell_x, cell_y), 40, YELLOW, -1)
                elif cell == "empty":
                    cv2.circle(display_image, (cell_x, cell_y), 40, WHITE, -1)

    # Display the annotated image
    cv2.imshow('Annotated Image', image)
    cv2.imshow('Detected state', display_image)

    # Save the annotated image if desired
    # cv2.imwrite('annotated_image_custom_boxes.jpg', image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
