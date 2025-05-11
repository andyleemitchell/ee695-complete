import json
import os
import cv2
import numpy as np
import albumentations as A
from tqdm import tqdm
import random
import copy
import sys
import shutil

DATASET = 'merged_dataset_20250416_170949'

# --- Configuration ---
INPUT_IMAGE_DIR = './datasets/'+DATASET+'/images'
INPUT_JSON_PATH = './datasets/'+DATASET+'/result.json'

OUTPUT_IMAGE_DIR = './datasets/'+DATASET+'_augmented/images'
OUTPUT_JSON_PATH = './datasets/'+DATASET+'_augmented/result.json'

AUGMENTATIONS_PER_IMAGE = 2

transform = A.Compose([
    A.HorizontalFlip(p=0.5),
    A.Rotate(limit=20, p=0.5, border_mode=cv2.BORDER_CONSTANT, value=0),
    A.ShiftScaleRotate(shift_limit=0.1, scale_limit=0.1, rotate_limit=0, p=0.3, border_mode=cv2.BORDER_CONSTANT, value=0),
    A.RandomBrightnessContrast(brightness_limit=0.2, contrast_limit=0.2, p=0.5),
    A.GaussNoise(var_limit=(10.0, 50.0), p=0.3),
    A.Blur(blur_limit=(3, 7), p=0.3),
], bbox_params=A.BboxParams(format='coco', label_fields=['category_ids'], min_visibility=0.1, min_area=10))

def load_coco_data(json_path):
    try:
        with open(json_path, 'r') as f:
            coco_data = json.load(f)
        print(f"Successfully loaded COCO data from {json_path}")
        return coco_data
    except Exception as e:
        print(f"an error occurred loading JSON: {e}")
        sys.exit(1)


def create_output_dirs(img_dir, json_path):
    try:
        img_output_dir = os.path.abspath(img_dir)
        json_output_dir = os.path.abspath(os.path.dirname(json_path))

        print(f"Ensuring image output directory exists: {img_output_dir}")
        os.makedirs(img_output_dir, exist_ok=True)
        print(f"Ensuring annotation output directory exists: {json_output_dir}")
        os.makedirs(json_output_dir, exist_ok=True)

        if not os.path.isdir(img_output_dir):
             print(f"FATAL ERROR: Could not create or find image output directory: {img_output_dir}")
             sys.exit(1)
        if not os.path.isdir(json_output_dir):
             print(f"FATAL ERROR: Could not create or find JSON output directory: {json_output_dir}")
             sys.exit(1)

        test_img_file = os.path.join(img_output_dir, '.perm_test')
        try:
            with open(test_img_file, 'w') as f: f.write('test')
            os.remove(test_img_file)
        except Exception as e:
            print(f"ERROR: No write permission for image directory: {img_output_dir}. Error: {e}")
            sys.exit(1)

        print(f"Output directories ensured and writable: {img_output_dir}, {json_output_dir}")
        return img_output_dir

    except Exception as e:
        print(f"FATAL ERROR: An error occurred during output directory setup: {e}")
        sys.exit(1)


def get_max_ids(coco_data):
    max_img_id = 0
    for img in coco_data.get('images', []):
        try: max_img_id = max(max_img_id, int(img['id']))
        except (ValueError, TypeError): continue
    max_ann_id = 0
    for ann in coco_data.get('annotations', []):
        try: max_ann_id = max(max_ann_id, int(ann['id']))
        except (ValueError, TypeError): continue
    return max_img_id, max_ann_id


def augment_coco_dataset(input_img_dir, input_json_path, output_img_dir, output_json_path, augmentations_per_image):
    print("Starting COCO dataset augmentation...")

    coco_data = load_coco_data(input_json_path)
    abs_output_img_dir = create_output_dirs(output_img_dir, output_json_path)

    new_coco_data = copy.deepcopy(coco_data)

    print(f"Generating {augmentations_per_image} augmented versions per image...")
    print(f"Images will be saved to: {abs_output_img_dir}")

    new_images_added = []
    new_annotations_added = []
    copied_originals_count = 0
    skipped_originals_copy_count = 0
    augmented_images_count = 0
    skipped_load_count = 0

    current_img_id, current_ann_id = get_max_ids(coco_data)
    print(f"Starting new image IDs from: {current_img_id + 1}")
    print(f"Starting new annotation IDs from: {current_ann_id + 1}")

    image_id_to_anns = {}
    for ann in coco_data.get('annotations', []):
        img_id = ann['image_id']
        if img_id not in image_id_to_anns: image_id_to_anns[img_id] = []
        image_id_to_anns[img_id].append(ann)

    original_images = coco_data.get('images', [])

    for img_info in tqdm(original_images, desc="Processing images"):
        original_image_id = img_info['id']
        original_filename_from_json = img_info['file_name']

        base_filename_only = os.path.basename(original_filename_from_json)
        base_filename_part, file_ext = os.path.splitext(base_filename_only)
        if not file_ext: file_ext = '.jpg'

        input_image_path = os.path.join(input_img_dir, original_filename_from_json)

        output_original_image_path = os.path.join(abs_output_img_dir, base_filename_only)

        try:
            if not os.path.exists(input_image_path):
                print(f"\nWarning: Input image file not found: {input_image_path}. Skipping copy and augmentation.")
                skipped_load_count += 1
                continue

            if os.path.abspath(input_image_path) != os.path.abspath(output_original_image_path):
                 shutil.copy2(input_image_path, output_original_image_path)
                 copied_originals_count += 1
            else:
                 copied_originals_count += 1

        except Exception as e:
            print(f"\nWarning: Failed to copy original image {input_image_path} to {output_original_image_path}. Error: {e}")
            skipped_originals_copy_count += 1

        try:
            image = cv2.imread(input_image_path)
            if image is None:
                 print(f"\nWarning: Failed to load image {input_image_path}. Skipping augmentation.")
                 continue
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            print(f"\nWarning: Could not load/convert image {input_image_path}. Error: {e}. Skipping augmentation.")
            continue

        original_anns = image_id_to_anns.get(original_image_id, [])
        bboxes = [ann['bbox'] for ann in original_anns]
        category_ids = [ann['category_id'] for ann in original_anns]
        has_annotations = bool(bboxes)

        for i in range(augmentations_per_image):
            current_img_id += 1

            new_img_filename = f"{base_filename_part}_aug_{i+1}{file_ext}"
            new_img_filepath = os.path.join(abs_output_img_dir, new_img_filename)

            try:
                if has_annotations:
                    transformed = transform(image=image, bboxes=bboxes, category_ids=category_ids)
                    transformed_image = transformed['image']
                    transformed_bboxes = transformed['bboxes']
                    transformed_category_ids = transformed['category_ids']
                else:
                    temp_transform = A.Compose([t for t in transform.transforms if not isinstance(t, A.BboxParams)])
                    transformed = temp_transform(image=image)
                    transformed_image = transformed['image']
                    transformed_bboxes = []
                    transformed_category_ids = []

                if transformed_image is None:
                     print(f"\nERROR: transformed_image is None for {new_img_filename}. Skipping save.")
                     current_img_id -= 1
                     continue

                image_to_save = cv2.cvtColor(transformed_image, cv2.COLOR_RGB2BGR)
                save_success = cv2.imwrite(new_img_filepath, image_to_save)

                if not save_success:
                    print(f"!!!!!!!! FAILED TO SAVE AUGMENTED IMAGE !!!!!!!!: {new_img_filepath}")
                    current_img_id -= 1
                    continue
                else:
                    augmented_images_count += 1

                new_img_info = {
                    "id": current_img_id, "file_name": "images/"+new_img_filename, # Storing only filename
                    "width": transformed_image.shape[1], "height": transformed_image.shape[0],
                    "license": img_info.get("license"), "coco_url": img_info.get("coco_url", ""),
                    "flickr_url": img_info.get("flickr_url", ""), "date_captured": img_info.get("date_captured"),
                }
                new_images_added.append(new_img_info)

                for bbox, cat_id in zip(transformed_bboxes, transformed_category_ids):
                    current_ann_id += 1
                    x_min, y_min, w, h = map(lambda x: round(x, 2), bbox)
                    w = max(0, w); h = max(0, h)
                    area = round(w * h, 2)
                    if area <= 1:
                        current_ann_id -= 1; continue
                    new_ann = {
                        "id": current_ann_id, "image_id": current_img_id, "category_id": cat_id,
                        "bbox": [x_min, y_min, w, h], "area": area,
                        "iscrowd": 0, "segmentation": []
                    }
                    new_annotations_added.append(new_ann)

            except Exception as e:
                 print(f"\n!!!!!!!! ERROR during augmentation/saving loop for {base_filename_only}, Aug #{i+1} !!!!!!!!")
                 print(f"    Attempted save path: {new_img_filepath}")
                 print(f"    Error Type: {type(e).__name__}, Error: {e}")
                 if not any(img['id'] == current_img_id for img in new_images_added):
                      current_img_id -= 1
                 continue

    print("\nCombining original and augmented annotation data...")
    original_image_count = len(new_coco_data['images'])
    original_ann_count = len(new_coco_data['annotations'])

    new_coco_data['images'].extend(new_images_added)
    new_coco_data['annotations'].extend(new_annotations_added)

    final_image_count = len(new_coco_data['images'])
    final_ann_count = len(new_coco_data['annotations'])

    print(f"\nSaving final COCO annotations to {output_json_path}")
    try:
        with open(output_json_path, 'w') as f:
            json.dump(new_coco_data, f, indent=4)
        print(f"Successfully saved final COCO annotations.")
        print("--- Augmentation Summary ---")
        print(f"Original images processed: {len(original_images)}")
        print(f"Original images skipped (load/read error): {skipped_load_count}")
        print(f"Original images copied to output: {copied_originals_count}")
        print(f"Original images failed to copy: {skipped_originals_copy_count}")
        print(f"Augmented images successfully generated & saved: {augmented_images_count}")
        print(f"Augmented annotations generated: {len(new_annotations_added)}")
        print(f"Total images in new JSON: {final_image_count} (Original: {original_image_count}, Added: {len(new_images_added)})")
        print(f"Total annotations in new JSON: {final_ann_count} (Original: {original_ann_count}, Added: {len(new_annotations_added)})")


    except Exception as e:
        print(f"\n!!!!!!!! ERROR: Failed to save final augmented JSON file !!!!!!!!")
        print(f"    Path: {output_json_path}")
        print(f"    Error Type: {type(e).__name__}, Error: {e}")

if __name__ == "__main__":
    if not os.path.isdir(INPUT_IMAGE_DIR): sys.exit(f"FATAL ERROR: Input image directory not found: {INPUT_IMAGE_DIR}")
    if not os.path.isfile(INPUT_JSON_PATH): sys.exit(f"FATAL ERROR: Input JSON file not found: {INPUT_JSON_PATH}")
    if AUGMENTATIONS_PER_IMAGE < 0: sys.exit(f"FATAL ERROR: AUGMENTATIONS_PER_IMAGE cannot be negative.")

    augment_coco_dataset(
        input_img_dir=INPUT_IMAGE_DIR,
        input_json_path=INPUT_JSON_PATH,
        output_img_dir=OUTPUT_IMAGE_DIR,
        output_json_path=OUTPUT_JSON_PATH,
        augmentations_per_image=AUGMENTATIONS_PER_IMAGE
    )
    print("\nAugmentation process finished.")
