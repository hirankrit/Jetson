# 🚀 Isaac-agri-robot Repository Migration Plan

**Date:** November 6, 2025
**Old Repo:** `hirankrit/Jetson` → Archive
**New Repo:** `hirankrit/Isaac-agri-robot` → Production

---

## 🎯 Goals

1. **Clean structure** for production-ready code
2. **Focus on Isaac ROS** integration (ESS, Argus camera, DNN inference)
3. **Agricultural robotics** specialization (pepper sorting)
4. **Complete documentation** for reproducibility

---

## 📁 New Repository Structure

```
Isaac-agri-robot/
├── README.md                      # Project overview
├── LICENSE                        # MIT License
├── .gitignore                     # Python, Jetson, ROS2, datasets
├── requirements.txt               # Python dependencies
│
├── src/                           # Source code
│   ├── detection/                 # 2D Object Detection
│   │   ├── deploy_pepper_detection.py
│   │   ├── benchmark_tensorrt.py
│   │   └── README.md
│   │
│   ├── depth/                     # 3D Depth Estimation
│   │   ├── stereo_preprocess.py  # Auto preprocessing
│   │   ├── test_stereo_depth.py  # StereoSGBM pipeline
│   │   ├── capture_stereo_gst.sh # GStreamer capture
│   │   └── README.md
│   │
│   ├── integration/               # 3D Detection Pipeline
│   │   ├── yolo_depth_3d.py      # YOLO + Depth → 3D
│   │   └── README.md
│   │
│   └── isaac_ros/                 # Isaac ROS Integration
│       ├── ess_launch/            # ESS launch files
│       ├── argus_camera/          # Argus camera nodes
│       └── README.md
│
├── models/                        # Trained models
│   ├── detection/
│   │   ├── yolo11n_pepper.engine  # TensorRT
│   │   ├── yolo11n_pepper.pt      # PyTorch
│   │   └── README.md
│   └── calibration/
│       ├── stereo_calib.yaml      # Stereo calibration
│       └── README.md
│
├── data/                          # Dataset (gitignored)
│   ├── .gitkeep
│   └── README.md                  # Instructions to download
│
├── backup/                        # Backup folder (gitignored)
│   ├── datasets/                  # 805 pepper images
│   └── experiments/               # Old experiments
│
├── docs/                          # Documentation
│   ├── setup/                     # Installation guides
│   │   ├── jetson_setup.md
│   │   ├── isaac_ros_setup.md
│   │   └── camera_setup.md
│   │
│   ├── training/                  # Training documentation
│   │   ├── dataset_collection.md
│   │   ├── training_guide.md
│   │   └── results.md
│   │
│   ├── deployment/                # Deployment guides
│   │   ├── detection_guide.md
│   │   ├── depth_guide.md
│   │   └── integration_guide.md
│   │
│   └── hardware/                  # Hardware documentation
│       ├── camera_specs.md
│       ├── calibration_guide.md
│       └── jetson_specs.md
│
├── scripts/                       # Utility scripts
│   ├── capture_images.sh
│   ├── test_camera.sh
│   └── benchmark.sh
│
└── tests/                         # Unit tests
    ├── test_detection.py
    ├── test_depth.py
    └── test_integration.py
```

---

## 📝 Files to Copy from Old Repo

### ✅ Must Copy (Production-Ready)

**Detection:**
- `deploy_pepper_detection.py`
- `benchmark_tensorrt.py`
- `runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine`
- `runs/train/yolo11n_pepper_gpu_100epochs/weights/best.pt`

**Depth:**
- `stereo_preprocess.py` ⭐ NEW
- `test_stereo_depth.py` ⭐ NEW
- `capture_stereo_gst.sh` ⭐ NEW
- `improve_stereo_depth.py` ⭐ NEW
- `demo_auto_preprocess.py` ⭐ NEW

**Calibration:**
- `stereo_calib.yaml`
- `stereo_calib_12mm.yaml`
- `stereo_calib_48mm.yaml`
- `calibration_pattern_18mm.svg`
- `calibration_pattern_16mm.svg`

**Documentation:**
- `docs/setup/jetson_setup.md`
- `docs/setup/isaac_ros_ess_setup.md`
- `docs/training/final_results.md`
- `docs/deployment/deployment_guide.md`
- `CAMERA_CALIBRATION_GUIDE.md`
- `PATTERN_PRINTING_GUIDE.md`
- `ESS_TEST_SUMMARY.md`

**Dataset Config:**
- `data.yaml`

### 🔶 Optional Copy (Reference)

- Previous camera scripts (`view_camera.py`, `gstreamer_camera_node.py`)
- Theory documents (`THEORY_*.md`)
- Weekly reports (`WEEK*_REPORT.md`)

### ❌ Do NOT Copy

- Temporary test files (`test_*.jpg`, `demo_*.jpg`)
- Old experiments (`old_experiments/`)
- Archive folder (`archive/`)
- CVAT installation (`cvat/`)
- Large datasets (backup separately)
- Build artifacts (`__pycache__/`, `*.pyc`)
- Log files (`*.log`)

---

## 🔧 Migration Steps

### Step 1: Create New Repository on GitHub

```bash
# On GitHub:
1. Go to: https://github.com/new
2. Repository name: Isaac-agri-robot
3. Description: "Isaac ROS-based agricultural robot for pepper sorting using YOLO11n + stereo depth"
4. Public or Private: Your choice
5. Add README: ✅
6. Add .gitignore: Python
7. Add License: MIT
8. Create repository
```

### Step 2: Clone New Repository

```bash
cd ~
git clone https://github.com/hirankrit/Isaac-agri-robot.git
cd Isaac-agri-robot
```

### Step 3: Setup Structure

```bash
# Create directory structure
mkdir -p src/detection src/depth src/integration src/isaac_ros
mkdir -p models/detection models/calibration
mkdir -p data backup/datasets backup/experiments
mkdir -p docs/setup docs/training docs/deployment docs/hardware
mkdir -p scripts tests

# Create .gitkeep for empty folders
touch data/.gitkeep backup/.gitkeep
```

### Step 4: Copy Files

```bash
# Copy from old project
OLD_PROJECT="/home/jay/Project"

# Detection
cp $OLD_PROJECT/deploy_pepper_detection.py src/detection/
cp $OLD_PROJECT/benchmark_tensorrt.py src/detection/

# Depth (NEW!)
cp $OLD_PROJECT/stereo_preprocess.py src/depth/
cp $OLD_PROJECT/test_stereo_depth.py src/depth/
cp $OLD_PROJECT/improve_stereo_depth.py src/depth/
cp $OLD_PROJECT/demo_auto_preprocess.py src/depth/
cp $OLD_PROJECT/capture_stereo_gst.sh src/depth/
chmod +x src/depth/capture_stereo_gst.sh

# Models
cp $OLD_PROJECT/runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine models/detection/yolo11n_pepper.engine
cp $OLD_PROJECT/runs/train/yolo11n_pepper_gpu_100epochs/weights/best.pt models/detection/yolo11n_pepper.pt

# Calibration
cp $OLD_PROJECT/stereo_calib.yaml models/calibration/
cp $OLD_PROJECT/calibration_pattern_*.svg models/calibration/

# Config
cp $OLD_PROJECT/data.yaml data/

# Documentation
cp -r $OLD_PROJECT/docs/* docs/
cp $OLD_PROJECT/CAMERA_CALIBRATION_GUIDE.md docs/hardware/
cp $OLD_PROJECT/PATTERN_PRINTING_GUIDE.md docs/hardware/
cp $OLD_PROJECT/ESS_TEST_SUMMARY.md docs/setup/

# Backup dataset (if needed)
# cp -r $OLD_PROJECT/pepper_dataset backup/datasets/
```

### Step 5: Create README.md

See: `NEW_REPO_README.md`

### Step 6: Create .gitignore

See: `NEW_REPO_GITIGNORE.txt`

### Step 7: Create requirements.txt

See: `NEW_REPO_REQUIREMENTS.txt`

### Step 8: Initial Commit

```bash
git add .
git commit -m "feat: Initial commit - Pepper sorting robot with YOLO11n + stereo depth

- 2D Detection: YOLO11n (89.59% mAP@50)
- 3D Depth: StereoSGBM + Auto preprocessing (+359% improvement)
- Hardware: Jetson Orin Nano + IMX219 Stereo Camera
- Ready for Isaac ROS ESS integration

Week 4 Day 1 milestone achieved! 🎉"

git push origin main
```

### Step 9: Archive Old Repository

```bash
# On GitHub:
1. Go to: https://github.com/hirankrit/Jetson
2. Settings → Scroll down → Archive this repository
3. Type repository name to confirm
4. Archive

# Or keep it but add deprecation notice:
# Add to old repo README: "⚠️ ARCHIVED - See new repo: Isaac-agri-robot"
```

---

## 📊 Repository Comparison

| Feature | Old Repo (Jetson) | New Repo (Isaac-agri-robot) |
|---------|-------------------|----------------------------|
| **Name** | Generic | Specific (Isaac + Agriculture) |
| **Structure** | Mixed | Organized by component |
| **Files** | 88 untracked | Clean production code |
| **Focus** | General experiments | Production-ready system |
| **Documentation** | Scattered | Organized by topic |
| **Size** | Large (with experiments) | Compact (essentials only) |
| **Maintenance** | Archived | Active development |

---

## ✅ Checklist

### Before Migration
- [x] Update claude.md with Week 4 progress
- [ ] Create NEW_REPO_README.md
- [ ] Create NEW_REPO_GITIGNORE.txt
- [ ] Create NEW_REPO_REQUIREMENTS.txt

### During Migration
- [ ] Create Isaac-agri-robot on GitHub
- [ ] Clone new repository
- [ ] Setup directory structure
- [ ] Copy production files
- [ ] Create documentation
- [ ] Initial commit & push

### After Migration
- [ ] Archive old repository
- [ ] Update local development to use new repo
- [ ] Test all scripts in new structure
- [ ] Add CI/CD (optional)

---

## 🎯 Benefits of New Repository

1. ✅ **Professional presentation** for portfolio/resume
2. ✅ **Clear focus** on Isaac ROS + Agricultural robotics
3. ✅ **Better organization** for collaboration
4. ✅ **Cleaner history** (fresh start)
5. ✅ **Easier maintenance** (no legacy files)
6. ✅ **Production-ready** structure from day 1

---

## 📝 Notes

- **Dataset backup:** Store in `backup/datasets/` but don't commit to git
- **Large files:** Consider Git LFS for model weights
- **Documentation:** Keep comprehensive guides for reproducibility
- **Testing:** Add tests as system grows
- **CI/CD:** Can add GitHub Actions later for automated testing

---

**Next Steps:** Create supporting files (README, .gitignore, requirements.txt) and execute migration!
