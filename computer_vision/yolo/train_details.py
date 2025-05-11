from ultralytics import YOLO

model = YOLO("./weights/april16.pt")

metrics = model.val(
    data="/home/andy/work/cv-nn-testing/datasets/merged_dataset_20250416_170949_augmented_resized_split/data.yaml",
    # save_json=True   
)

# print(model.names)

# print(metrics.box.class_result(0))

print()
print()
header = f"{'Class':>15}\t{'P':>9}\t{'R':>9}\t{'mAP50':>9}\t{'mAP50-95':>9}\t{'F1':>9}"
rows = []
total_p = total_r = total_map50 = total_map5095 = 0
for i in range(len(model.names)):
    results = metrics.box.class_result(i)
    total_p+=results[0]
    total_r+=results[1]
    total_map50+=results[2]
    total_map5095+=results[3]
    rows.append(f"{model.names[i]:>15}\t{results[0]:>9.3f}\t{results[1]:>9.3f}\t{results[2]:>9.3f}\t{results[3]:>9.3f}\t{metrics.box.f1[i]:>9.3f}")

rows.insert(0, f"{'all':>15}\t{total_p/len(model.names):>9.3f}\t{total_r/len(model.names):>9.3f}\t{total_map50/len(model.names):>9.3f}\t{total_map5095/len(model.names):>9.3f}")
print(header)
for row in rows:
    print(row)


with open("./yolo_train_data/data.txt", "w+") as f:
    f.write(header)
    f.write("\n")
    for row in rows:
        f.write(row)
        f.write("\n")

print(metrics.box.ap_class_index)
