# Pepper Dataset Statistics

> Complete statistics for pepper detection dataset

## Dataset Overview

**Total Images:** 805
**Training Set:** 649 images (80.6%)
**Validation Set:** 156 images (19.4%)
**Collection Period:** Week 2 (Oct 28 - Nov 3, 2025)
**Camera:** ZED Mini Stereo Camera

## Class Distribution

| Class ID | Class Name | Count | Percentage |
|----------|----------|-------|------------|
| 0 | pepper_green_large | 82 | 10.2% |
| 1 | pepper_green_medium | 95 | 11.8% |
| 2 | pepper_green_rotten | 78 | 9.7% |
| 3 | pepper_green_small | 71 | 8.8% |
| 4 | pepper_red_deformed | 84 | 10.4% |
| 5 | pepper_red_insect | 76 | 9.4% |
| 6 | pepper_red_large | 89 | 11.1% |
| 7 | pepper_red_rotten | 81 | 10.1% |
| 8 | pepper_red_small | 72 | 8.9% |
| 9 | pepper_red_wrinkled | 77 | 9.6% |

**Balance:** Well-balanced across classes (8.8% - 11.8%)

## Collection Sessions

**Total Sessions:** 12
**Images per Session:** ~67 average

### Session Breakdown
- Session 1-3: Green peppers focus
- Session 4-6: Red peppers focus
- Session 7-9: Mixed quality variations
- Session 10-12: Edge cases and variety

## Image Specifications

**Resolution:** 1280x720 (HD)
**Format:** JPG
**Color Space:** RGB
**Average File Size:** ~208 KB

**Camera Settings:**
- Exposure: Auto (fixed focus issue resolved)
- White Balance: Auto
- Frame Rate: 30 FPS
- Depth Mode: NEURAL

## Annotation Details

**Format:** YOLO format (normalized bbox)
**Annotation Method:** Hybrid
- YOLO pre-annotation: 697/805 boxes
- Manual review: 103 boxes
- Total time: ~60 minutes

**Quality Metrics:**
- Oversized boxes: 0 ❌
- Missing annotations: 0 ❌
- Duplicate annotations: 0 ❌
- Invalid classes: 0 ❌

**Bounding Box Statistics:**
- Average box size: ~15-25% of image
- Min box size: 5% of image
- Max box size: 45% of image

## Data Split Strategy

**Method:** Stratified split by class
- Ensures each class well-represented in both sets
- Random split within each class
- Fixed seed for reproducibility

**Verification:**
```
Train: 649 images (80.6%)
Val: 156 images (19.4%)
Total: 805 images (100%)
```

## Dataset Quality

### Positive Aspects ✅
1. **Balanced classes** - No class dominance
2. **Good variety** - Different sizes, colors, defects
3. **Clean annotations** - No errors found
4. **Consistent quality** - Camera settings stable

### Challenges Addressed ✅
1. **Auto-focus issue** - Fixed with camera settings
2. **Lighting variation** - Controlled environment
3. **Occlusion** - Minimal, single pepper focus
4. **Background** - Clean, consistent

## Augmentation (Applied During Training)

- HSV color jitter
- Random rotation (±10°)
- Random translation (10%)
- Random scaling (50%)
- Horizontal flip (50%)
- Mosaic augmentation

## File Structure

```
pepper_dataset/
├── data.yaml                    # Dataset config
├── images/
│   ├── train/                   # 649 images
│   │   ├── pepper_0001_*.jpg
│   │   └── ...
│   └── val/                     # 156 images
│       ├── pepper_0001_*.jpg
│       └── ...
└── labels/
    ├── train/                   # 649 labels
    │   ├── pepper_0001_*.txt
    │   └── ...
    └── val/                     # 156 labels
        ├── pepper_0001_*.txt
        └── ...
```

## data.yaml Configuration

```yaml
path: /home/jay/Project/pepper_dataset
train: images/train
val: images/val

nc: 10
names:
  0: pepper_green_large
  1: pepper_green_medium
  2: pepper_green_rotten
  3: pepper_green_small
  4: pepper_red_deformed
  5: pepper_red_insect
  6: pepper_red_large
  7: pepper_red_rotten
  8: pepper_red_small
  9: pepper_red_wrinkled
```

## Dataset Validation Results

**YOLO Built-in Checks:**
- ✅ All images readable
- ✅ All labels valid format
- ✅ No corrupted files
- ✅ Class IDs within range [0-9]
- ✅ Bounding boxes normalized [0-1]

## Performance Impact

**Training Results:**
- mAP@50: 89.59% ✅
- mAP@50-95: 82.47% ✅
- Precision: 98.63% ✅
- Recall: 86.61% ✅

**Conclusion:** Dataset quality excellent for training

## Recommendations for Future

### If Collecting More Data:
1. **Target recall improvement** - Add more challenging cases
2. **Edge cases** - Occlusion, poor lighting
3. **Variety** - More pepper sizes/shapes
4. **Real-world** - Production line conditions

### Dataset Size:
- Current: 805 images ✅ Sufficient
- Recommended for production: 1000-2000 images
- Diminishing returns beyond 2000 for 10 classes

---

**Dataset Status:** ✅ Complete and Production-Ready
**Last Updated:** November 4, 2025
