import json
import os
import cv2
import numpy as np
import albumentations as A
from tqdm import tqdm
import copy
import sys
import shutil

DATASET = 'merged_dataset_20250416_170949'+'_augmented'
INPUT_IMAGE_DIR = './datasets/'+DATASET+'/images'
INPUT_JSON_PATH = './datasets/'+DATASET+'/result.json'

OUTPUT_RESIZED_IMAGE_DIR = './datasets/'+DATASET+'_resized/images'
OUTPUT_RESIZED_JSON_PATH = './datasets/'+DATASET+'_resized'+'/result.json'

TARGET_HEIGHT = 640
TARGET_WIDTH = 640

resize_transform = A.Compose([
    A.Resize(height=TARGET_HEIGHT, width=TARGET_WIDTH, interpolation=cv2.INTER_LINEAR)
], bbox_params=A.BboxParams(format='coco', label_fields=['category_ids'], min_visibility=0.0, min_area=1))

def load_coco_data(json_path):
    """Loads COCO annotation data from JSON file."""
    try:
        with open(json_path, 'r') as f:
            coco_data = json.load(f)
        print(f"Successfully loaded COCO data from {json_path}")
        return coco_data
    except Exception as e:
        print(f"An unexpected error occurred loading JSON: {e}")
        sys.exit(1)

def create_output_dirs(img_dir, json_path):
    try:
        img_output_dir = os.path.abspath(img_dir)
        json_output_dir = os.path.abspath(os.path.dirname(json_path))

        img_output_dir_orig = img_output_dir
        img_output_dir = img_output_dir_orig + "/images"

        print(f"Ensuring image output directory exists: {img_output_dir}")
        os.makedirs(img_output_dir, exist_ok=True)
        print(f"Ensuring annotation output directory exists: {json_output_dir}")
        os.makedirs(json_output_dir, exist_ok=True)

        test_img_file = os.path.join(img_output_dir, '.perm_test')
        try:
            with open(test_img_file, 'w') as f: f.write('test'); os.remove(test_img_file)
        except Exception as e:
            print(f"ERROR: No write permission for image directory: {img_output_dir}. Error: {e}")
            sys.exit(1)

        print(f"Output directories ensured and writable.")
        return img_output_dir_orig

    except Exception as e:
        print(f"FATAL ERROR: An error occurred during output directory setup: {e}")
        sys.exit(1)

def resize_coco_dataset(input_img_dir, input_json_path, output_img_dir, output_json_path, height, width):
    print(f"Starting COCO dataset resize to {width}x{height}...")

    coco_data = load_coco_data(input_json_path)
    abs_output_img_dir = create_output_dirs(output_img_dir, output_json_path)

    final_coco_data = {
        "info": coco_data.get("info", {}),
        "licenses": coco_data.get("licenses", []),
        "categories": coco_data.get("categories", []),
        "images": [],
        "annotations": []
    }

    image_id_to_anns = {}
    for ann in coco_data.get('annotations', []):
        img_id = ann['image_id']
        if img_id not in image_id_to_anns:
            image_id_to_anns[img_id] = []
        image_id_to_anns[img_id].append(ann)

    processed_count = 0
    skipped_count = 0

    for img_info in tqdm(coco_data.get('images', []), desc="Resizing images"):
        image_id = img_info['id']
        filename = img_info['file_name']
        input_image_path = os.path.join(input_img_dir, filename)
        output_image_path = os.path.join(abs_output_img_dir, filename)

        if not os.path.exists(input_image_path):
            print(f"\nWarning: Input image file not found: {input_image_path}. Skipping.")
            skipped_count += 1
            continue
        try:
            image = cv2.imread(input_image_path)
            if image is None:
                 print(f"\nWarning: Failed to load image {input_image_path}. Skipping.")
                 skipped_count += 1
                 continue
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print(f"\nWarning: Could not load/convert image {input_image_path}. Error: {e}. Skipping.")
            skipped_count += 1
            continue

        current_anns = image_id_to_anns.get(image_id, [])
        bboxes = [list(map(float, ann['bbox'])) for ann in current_anns]
        category_ids = [ann['category_id'] for ann in current_anns]
        original_ann_ids = [ann['id'] for ann in current_anns]

        try:
            resized_data = resize_transform(image=image, bboxes=bboxes, category_ids=category_ids)
            resized_image = resized_data['image']
            resized_bboxes = resized_data['bboxes']
            resized_category_ids = resized_data['category_ids']

            if resized_image is None:
                print(f"\nWarning: Resized image is None for {filename}. Skipping.")
                skipped_count += 1
                continue

            save_success = cv2.imwrite(output_image_path, cv2.cvtColor(resized_image, cv2.COLOR_RGB2BGR))

            if not save_success:
                print(f"\nWarning: Failed to save resized image {output_image_path}. Skipping annotation update for this image.")
                skipped_count += 1
                continue

            updated_img_info = copy.deepcopy(img_info)
            updated_img_info['width'] = resized_image.shape[1]
            updated_img_info['height'] = resized_image.shape[0]
            temp_filename = os.path.basename(updated_img_info['file_name'])
            updated_img_info['file_name'] = "images/"+temp_filename
            final_coco_data['images'].append(updated_img_info)

            if len(resized_bboxes) != len(original_ann_ids):
                 print(f"\nWarning: Number of bounding boxes changed after resize for {filename} (Original: {len(original_ann_ids)}, Resized: {len(resized_bboxes)}). Annotation ID matching might be incorrect if boxes were removed.")

            for i, (bbox, cat_id) in enumerate(zip(resized_bboxes, resized_category_ids)):
                ann_id = original_ann_ids[i] if i < len(original_ann_ids) else None

                if ann_id is None:
                     print(f"\nError: Could not determine original annotation ID for a resized box in {filename}. Skipping this annotation.")
                     continue

                x_min, y_min, w, h = map(lambda x: round(x, 2), bbox)
                w = max(0, w); h = max(0, h)
                area = round(w * h, 2)

                if area <= 0:
                    continue

                updated_ann = {
                    "id": ann_id,
                    "image_id": image_id,
                    "category_id": cat_id,
                    "bbox": [x_min, y_min, w, h],
                    "area": area,
                    "iscrowd": 0,
                    "segmentation": []
                }
                final_coco_data['annotations'].append(updated_ann)

            processed_count += 1

        except Exception as e:
            print(f"\nError processing image {filename}: {e}")
            skipped_count += 1
            continue

    print(f"\nSaving resized COCO annotations to {output_json_path}")
    try:
        with open(output_json_path, 'w') as f:
            json.dump(final_coco_data, f, indent=4)
        print(f"Successfully saved resized COCO annotations.")
        print("--- Resize Summary ---")
        print(f"Images processed and saved: {processed_count}")
        print(f"Images skipped (not found, load error, save error): {skipped_count}")
        print(f"Total images in output JSON: {len(final_coco_data['images'])}")
        print(f"Total annotations in output JSON: {len(final_coco_data['annotations'])}")

    except Exception as e:
        print(f"\n!!!!!!!! ERROR: Failed to save final resized JSON file !!!!!!!!")
        print(f"    Path: {output_json_path}")
        print(f"    Error Type: {type(e).__name__}, Error: {e}")


if __name__ == "__main__":
    if not os.path.isdir(INPUT_IMAGE_DIR):
        sys.exit(f"FATAL ERROR: Input image directory not found: {INPUT_IMAGE_DIR}")
    if not os.path.isfile(INPUT_JSON_PATH):
        sys.exit(f"FATAL ERROR: Input JSON file not found: {INPUT_JSON_PATH}")

    resize_coco_dataset(
        input_img_dir=INPUT_IMAGE_DIR,
        input_json_path=INPUT_JSON_PATH,
        output_img_dir=OUTPUT_RESIZED_IMAGE_DIR,
        output_json_path=OUTPUT_RESIZED_JSON_PATH,
        height=TARGET_HEIGHT,
        width=TARGET_WIDTH
    )
    print("\nDataset resizing process finished.")
