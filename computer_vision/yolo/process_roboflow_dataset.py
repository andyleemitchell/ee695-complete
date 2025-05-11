import supervision as sv
import ruamel.yaml
import albumentations as A

# exported dataset from roboflow
# TODO do the unzip automatically
ROBOFLOW_YOLO_DIR = "./dataset_acquisition/roboflow_data"

OUTPUT_FOLDER = "./dataset_acquisition/roboflow_data_combined"

# import dataset
ds_train = sv.DetectionDataset.from_yolo(
    images_directory_path=ROBOFLOW_YOLO_DIR+"/train/images",
    annotations_directory_path=ROBOFLOW_YOLO_DIR+"/train/labels",
    data_yaml_path=ROBOFLOW_YOLO_DIR+"/data.yaml"
)
ds_test = sv.DetectionDataset.from_yolo(
    images_directory_path=ROBOFLOW_YOLO_DIR+"/test/images",
    annotations_directory_path=ROBOFLOW_YOLO_DIR+"/test/labels",
    data_yaml_path=ROBOFLOW_YOLO_DIR+"/data.yaml"
)
ds_valid = sv.DetectionDataset.from_yolo(
    images_directory_path=ROBOFLOW_YOLO_DIR+"/valid/images",
    annotations_directory_path=ROBOFLOW_YOLO_DIR+"/valid/labels",
    data_yaml_path=ROBOFLOW_YOLO_DIR+"/data.yaml"
)


ds_merged = sv.DetectionDataset.merge([ds_train, ds_test, ds_valid])

ds_merged.as_coco(
    images_directory_path=OUTPUT_FOLDER+"/images",
    annotations_path=OUTPUT_FOLDER+"/result.json"
)
