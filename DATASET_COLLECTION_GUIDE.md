# 🌶️ Pepper Dataset Collection Guide

**Week 2: Dataset Collection & Annotation**
**Goal**: เก็บ dataset พริก 500-1000 ภาพ พร้อม labels สำหรับ YOLO training

---

## 📋 Overview

### Dataset Requirements
- **Total images**: 500-1000 ภาพ
- **Classes**: 6 classes
  - `pepper_red_fresh` (พริกแดงสด)
  - `pepper_red_rotten` (พริกแดงเน่า)
  - `pepper_green_fresh` (พริกเขียวสด)
  - `pepper_green_rotten` (พริกเขียวเน่า)
  - `pepper_yellow_fresh` (พริกเหลืองสด)
  - `pepper_yellow_rotten` (พริกเหลืองเน่า)

### Split Ratio
- **Training**: 80% (~400-800 images)
- **Validation**: 20% (~100-200 images)

---

## 🛠️ Tool: collect_dataset.py

### Features
- ✅ Stereo camera support (left camera = primary dataset)
- ✅ Optimized camera settings (exposure=30ms, gain=2)
- ✅ 3 save modes:
  - **Mode 1**: Left only (for YOLO training) - Default ⭐
  - **Mode 2**: Left + Right (stereo pair)
  - **Mode 3**: Left + Right + Depth (full data)
- ✅ Real-time preview with statistics
- ✅ Metadata logging (YAML format)

### Usage

**Basic (Mode 1 - Left only)**:
```bash
python3 collect_dataset.py
```

**Custom output directory**:
```bash
python3 collect_dataset.py --output my_dataset
```

**Custom camera settings**:
```bash
python3 collect_dataset.py --exposure 25 --gain 1.5
```

### Controls
- **'c'**: Capture image
- **'s'**: Toggle save mode (1/2/3)
- **'q'**: Quit and show summary

---

## 📸 Data Collection Strategy

### 1. Variety is Key! 🔑

**a) Quantity per arrangement**:
- **Single pepper** (1 ผล): 150-200 images
- **Small group** (3-5 ผล): 150-200 images
- **Large pile** (10+ ผล): 200-300 images

**b) Color distribution**:
- **Red**: 40% (~200-400 images)
- **Green**: 40% (~200-400 images)
- **Yellow**: 20% (~100-200 images)

**c) Quality distribution**:
- **Fresh**: 70% (~350-700 images)
- **Rotten/damaged**: 30% (~150-300 images)

**d) Orientation**:
- หงาย (stem up): 30%
- คว่ำ (stem down): 30%
- ข้าง (sideways): 40%

**e) Lighting conditions** (if possible):
- เช้า (morning): 30%
- เที่ยง (noon): 40%
- เย็น (evening): 30%

### 2. Collection Checklist ✅

**Session 1: Single Peppers (Day 6 - 150-200 images)**
- [ ] Red fresh: 50-70 images (different angles)
- [ ] Red rotten: 20-30 images
- [ ] Green fresh: 50-70 images
- [ ] Green rotten: 20-30 images
- [ ] Yellow fresh: 30-40 images
- [ ] Yellow rotten: 10-20 images

**Session 2: Small Groups (Day 6-7 - 150-200 images)**
- [ ] Mixed colors: 100 images (3-5 peppers per image)
- [ ] Single color groups: 50-100 images

**Session 3: Large Piles (Day 7 - 200-300 images)**
- [ ] Mixed quality piles: 100-150 images
- [ ] Fresh only piles: 50-100 images
- [ ] Include edge cases (overlapping, partially visible)

### 3. Tips for Good Dataset 💡

**DO ✅**:
- **Vary distance**: 23cm - 35cm from camera
- **Vary angles**: Rotate peppers, move camera slightly
- **Include occlusions**: Some peppers partially hidden (realistic)
- **Good lighting**: Use LED setup (Top, Left, Right)
- **Sharp images**: Wait for camera to stabilize before capture
- **Consistent background**: Use gray cloth (ผ้าสีเทา)

**DON'T ❌**:
- **Blur**: Don't move while capturing
- **Poor lighting**: Avoid shadows or overexposure
- **Too close/far**: Stay within 23-35cm range
- **Same pose**: Don't capture identical images
- **Inconsistent background**: Keep gray cloth setup

---

## 📂 Output Structure

```
pepper_dataset/
├── raw/
│   ├── left/          # Left camera images (for YOLO)
│   │   ├── pepper_0000_20251028_123456_789.jpg
│   │   ├── pepper_0001_20251028_123501_234.jpg
│   │   └── ...
│   ├── right/         # Right camera images (optional)
│   └── depth/         # Depth maps (optional)
│
└── metadata/
    └── collection_log.yaml  # Collection metadata
```

**Metadata includes**:
- Total images collected
- Collection date/time
- Camera settings (resolution, exposure, gain)
- Per-image metadata (timestamp, mode, files, depth coverage)

---

## 📊 Progress Tracking

### Daily Goals

**Day 6 (Session 1-2)**:
- Target: 300-400 images
- Focus: Single + Small groups
- Time: ~2-3 hours

**Day 7 (Session 3)**:
- Target: 200-300 images
- Focus: Large piles + edge cases
- Time: ~2-3 hours

### Milestone Checkpoint
After collecting 500+ images:
1. Review image quality
2. Check color/quality distribution
3. Verify variety (angles, distances, arrangements)
4. If needed, collect more specific cases

---

## 🎯 Next Steps (Day 8-10)

After collection is complete:

### Option 1: Roboflow (แนะนำ - ง่ายที่สุด) ⭐

**Steps**:
1. **Create account**: https://roboflow.com/
2. **Create new project**: "Pepper Sorting"
3. **Upload images**: Drag & drop from `pepper_dataset/raw/left/`
4. **Annotate**:
   - Draw bounding boxes around each pepper
   - Label with correct class (6 classes)
   - Use keyboard shortcuts for speed:
     - `1-6`: Quick class selection
     - `Space`: Next image
     - `Ctrl+Z`: Undo
5. **Review**: Check annotations for accuracy
6. **Generate**: Create dataset version
7. **Export**: Download as "YOLOv8" format

**Roboflow Benefits**:
- ✅ Web-based (no installation)
- ✅ Auto-save (cloud backup)
- ✅ Collaboration (share with team)
- ✅ Augmentation (auto-generate variations)
- ✅ Version control

### Option 2: LabelImg (offline)

**Installation**:
```bash
pip install labelImg
```

**Usage**:
```bash
labelImg pepper_dataset/raw/left
```

**Steps**:
1. Open directory: `pepper_dataset/raw/left/`
2. Click "Change Save Dir": Select `pepper_dataset/labels/`
3. Draw bounding boxes (keyboard: `w`)
4. Label class
5. Save (keyboard: `s`)
6. Next image (keyboard: `d`)

**Manual train/val split**:
```bash
# After annotation, split into train/val
mkdir -p pepper_dataset/images/train
mkdir -p pepper_dataset/images/val
mkdir -p pepper_dataset/labels/train
mkdir -p pepper_dataset/labels/val

# Randomly move 80% to train, 20% to val
# (use script or manual selection)
```

---

## 📝 Dataset Quality Checklist

Before proceeding to training (Week 3), verify:

- [ ] **Quantity**: 500-1000 images ✅
- [ ] **Classes**: All 6 classes represented
- [ ] **Balance**: No class <10% of total
- [ ] **Variety**: Different angles, distances, arrangements
- [ ] **Quality**: Sharp images, good lighting
- [ ] **Annotations**: Accurate bounding boxes + correct labels
- [ ] **Split**: 80/20 train/val ratio
- [ ] **Format**: YOLOv8 format ready
- [ ] **Metadata**: data.yaml created with correct paths

---

## 🚀 Expected Results

### Collection Phase (Day 6-7)
- **500-1000 raw images** collected
- **Metadata log** with collection statistics
- **Organized folders** ready for annotation

### Annotation Phase (Day 8-10)
- **All images annotated** with bounding boxes
- **Dataset exported** in YOLOv8 format
- **data.yaml** configured
- **Ready for training** (Week 3)

---

## 📞 Troubleshooting

### Issue: Camera not opening
**Solution**: Check camera connection, try:
```bash
ls /dev/video*
# Should see /dev/video0 and /dev/video1

# Test with view_camera.py
python3 view_camera.py
```

### Issue: Images too dark/bright
**Solution**: Adjust exposure/gain:
```bash
# Darker
python3 collect_dataset.py --exposure 20 --gain 1.5

# Brighter
python3 collect_dataset.py --exposure 40 --gain 3
```

### Issue: Blurry images
**Solution**:
- Wait 2-3 seconds after placing pepper before capturing
- Ensure steady hands
- Check focus (should be ~176.5 left, ~171 right)

### Issue: Not enough variety
**Solution**:
- Use checklist above
- Review collected images periodically
- Identify missing scenarios and collect specifically

---

## ✅ Success Criteria

Dataset is ready when:
1. ✅ 500-1000 images collected
2. ✅ All 6 classes well-represented
3. ✅ Good variety (orientations, quantities, lighting)
4. ✅ Sharp, clear images
5. ✅ All images annotated
6. ✅ Exported in YOLOv8 format
7. ✅ data.yaml configured correctly

**🎉 Ready for Week 3: YOLO Training!**

---

**Last Updated**: 2025-10-28
**Tool Version**: collect_dataset.py v1.0
**Camera Settings**: exposure=30ms, gain=2 (optimized)
