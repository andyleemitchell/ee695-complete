import supervision as sv
import ruamel.yaml
import albumentations as A

# exported dataset from label studio
# TODO do the unzip automatically

DATASET = 'merged_dataset_20250416_170949'+'_augmented'+'_resized'

LABEL_STUDIO_COCO_DIR = "./datasets/"+DATASET

# where to put the split dataset
OUTPUT_FOLDER = "./datasets/"+DATASET+"_split"

# import dataset
ds = sv.DetectionDataset.from_coco(images_directory_path=LABEL_STUDIO_COCO_DIR,
                                   annotations_path=LABEL_STUDIO_COCO_DIR+"/result.json")

# split dataset
ds_train, ds = ds.split(split_ratio=0.8, shuffle=True)
ds_valid, ds_test = ds.split(split_ratio=0.5, shuffle=True)


# AUGMENTATIONS
augmentation = A.Compose(
    transforms=[
        A.Resize(height=640, width=640, p=1),
    ],
    bbox_params=A.BboxParams(
        format="pascal_voc",
        label_fields=ds.classes
    )
)



# save all datasets
ds_train.as_yolo(
    images_directory_path=OUTPUT_FOLDER+"/train/images",
    annotations_directory_path=OUTPUT_FOLDER+"/train/labels",
    data_yaml_path=OUTPUT_FOLDER+"/data.yaml"
)
ds_test.as_yolo(
    images_directory_path=OUTPUT_FOLDER+"/test/images",
    annotations_directory_path=OUTPUT_FOLDER+"/test/labels"
)
ds_valid.as_yolo(
    images_directory_path=OUTPUT_FOLDER+"/valid/images",
    annotations_directory_path=OUTPUT_FOLDER+"/valid/labels"
)

def add_data_to_yaml(input_file, output_file):
    data_to_add = {
        'train': '../train/images',
        'val': '../valid/images',
        'test': '../test/images'
    }

    try:
        yaml = ruamel.yaml.YAML()
        yaml.preserve_quotes = True
        yaml.indent(mapping=2, sequence=2, offset=2)

        with open(input_file, 'r') as f:
            existing_data = yaml.load(f)

        updated_data = data_to_add.copy()
        updated_data.update(existing_data)



        with open(output_file, 'w') as f:
            yaml.dump(updated_data, f)
    except FileNotFoundError:
        print(f"Error: Input file '{input_file}' not found.")
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")


# add data
input_yaml_file = OUTPUT_FOLDER+"/data.yaml"
output_yaml_file = OUTPUT_FOLDER+"/data.yaml"

add_data_to_yaml(input_yaml_file, output_yaml_file)
