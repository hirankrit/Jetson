#!/usr/bin/env python3
"""
Prepare Dataset Structure for YOLO Training

Creates organized folder structure and data.yaml template
for pepper dataset after annotation is complete.

Usage:
    python3 prepare_dataset_structure.py
    python3 prepare_dataset_structure.py --dataset my_dataset
"""

import os
import argparse
import yaml


def main():
    parser = argparse.ArgumentParser(
        description="Prepare dataset structure for YOLO training"
    )
    parser.add_argument(
        "--dataset",
        type=str,
        default="pepper_dataset",
        help="Dataset directory (default: pepper_dataset)",
    )

    args = parser.parse_args()
    dataset_dir = args.dataset

    print("=" * 80)
    print("🌶️  PREPARE DATASET STRUCTURE FOR YOLO TRAINING")
    print("=" * 80)
    print(f"\nDataset directory: {dataset_dir}")

    # Create folder structure
    folders = [
        f"{dataset_dir}/images/train",
        f"{dataset_dir}/images/val",
        f"{dataset_dir}/labels/train",
        f"{dataset_dir}/labels/val",
    ]

    print("\n📁 Creating folder structure...")
    for folder in folders:
        os.makedirs(folder, exist_ok=True)
        print(f"  ✓ {folder}")

    # Create data.yaml template
    data_yaml = {
        "path": os.path.abspath(dataset_dir),
        "train": "images/train",
        "val": "images/val",
        "nc": 6,
        "names": [
            "pepper_red_fresh",
            "pepper_red_rotten",
            "pepper_green_fresh",
            "pepper_green_rotten",
            "pepper_yellow_fresh",
            "pepper_yellow_rotten",
        ],
    }

    yaml_path = f"{dataset_dir}/data.yaml"
    with open(yaml_path, "w") as f:
        yaml.dump(data_yaml, f, default_flow_style=False, sort_keys=False)

    print(f"\n📄 Created data.yaml: {yaml_path}")
    print("\nContent:")
    print("---")
    with open(yaml_path, "r") as f:
        print(f.read())
    print("---")

    # Create README
    readme_content = f"""# Pepper Dataset

## Structure

```
{dataset_dir}/
├── images/
│   ├── train/          # Training images (80%)
│   └── val/            # Validation images (20%)
├── labels/
│   ├── train/          # Training labels (YOLO format)
│   └── val/            # Validation labels (YOLO format)
└── data.yaml           # Dataset configuration
```

## Classes (6 total)

0. pepper_red_fresh
1. pepper_red_rotten
2. pepper_green_fresh
3. pepper_green_rotten
4. pepper_yellow_fresh
5. pepper_yellow_rotten

## Label Format (YOLO)

Each .txt file contains one line per object:
```
<class_id> <x_center> <y_center> <width> <height>
```

All values normalized to [0, 1]

Example:
```
0 0.5 0.5 0.3 0.4
```
(pepper_red_fresh at center, width=0.3, height=0.4)

## Next Steps

1. Annotate images using Roboflow or LabelImg
2. Export annotations in YOLOv8 format
3. Copy/move files to train/val folders (80/20 split)
4. Verify data.yaml paths are correct
5. Ready for YOLO training!

## Training Command

```python
from ultralytics import YOLO

model = YOLO('yolov8n-seg.pt')
results = model.train(
    data='{yaml_path}',
    epochs=100,
    imgsz=640,
    batch=16,
    device=0
)
```

---
Created: 2025-10-28
Tool: prepare_dataset_structure.py
"""

    readme_path = f"{dataset_dir}/README.md"
    with open(readme_path, "w") as f:
        f.write(readme_content)

    print(f"\n📖 Created README: {readme_path}")

    # Summary
    print("\n" + "=" * 80)
    print("✅ DATASET STRUCTURE READY")
    print("=" * 80)
    print("\nCreated:")
    print("  ✓ Folder structure (images/labels, train/val)")
    print("  ✓ data.yaml configuration")
    print("  ✓ README.md documentation")

    print("\n🎯 Next steps:")
    print("  1. Collect images using collect_dataset.py")
    print("  2. Annotate using Roboflow or LabelImg")
    print("  3. Export to YOLO format")
    print(f"  4. Copy annotated images/labels to {dataset_dir}/")
    print("  5. Split into train/val (80/20)")
    print("  6. Start YOLO training (Week 3)!")

    print("\n" + "=" * 80)


if __name__ == "__main__":
    main()
