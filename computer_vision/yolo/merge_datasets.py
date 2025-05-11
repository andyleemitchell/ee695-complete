import supervision as sv
import datetime
# import ruamel.yaml
# import albumentations as A


input_datasets = [
    "./dataset_acquisition/roboflow_data_combined",
    "./datasets/16-april"
]

# where to put the merged dataset
OUTPUT_FOLDER = "./datasets/merged_dataset_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


ds_roboflow = sv.DetectionDataset.from_coco(
    images_directory_path=input_datasets[0]+"/images",
    annotations_path=input_datasets[0]+"/result.json"
)

ds_label_studio = sv.DetectionDataset.from_coco(
    images_directory_path=input_datasets[1],
    annotations_path=input_datasets[1]+"/result.json"
)

datasets = [ds_roboflow, ds_label_studio]

ds_merged = sv.DetectionDataset.merge(datasets)

ds_merged.as_coco(
    images_directory_path=OUTPUT_FOLDER+"/images",
    annotations_path=OUTPUT_FOLDER+"/result.json"
)
