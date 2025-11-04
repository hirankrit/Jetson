# 📁 Project Essential Files Guide

> Complete guide to essential files for Pepper Sorting Robot System

**Last Updated:** November 4, 2025
**Purpose:** Identify critical files needed for Week 4 and beyond

---

## 🎯 File Categories

Files are categorized by importance:
- ✅ **ESSENTIAL** - Cannot be deleted, critical for system
- ⚠️ **IMPORTANT** - Useful but can be regenerated
- 📚 **REFERENCE** - Archive/documentation, for reference only
- 🗑️ **EXPENDABLE** - Can be safely deleted

---

## ✅ ESSENTIAL FILES (Cannot Delete)

### 1. Production Model & Weights

**Location:** `runs/train/yolo11n_pepper_gpu_100epochs/`
**Size:** ~30 MB (core files)
**Status:** ✅ Critical

```
runs/train/yolo11n_pepper_gpu_100epochs/
├── weights/
│   ├── best.engine     ✅ ESSENTIAL (8.6M) - TensorRT production model
│   ├── best.pt         ✅ ESSENTIAL (5.3M) - PyTorch model (backup)
│   └── best.onnx       ⚠️ IMPORTANT (11M) - ONNX format (optional)
├── results.csv         ⚠️ IMPORTANT - Training metrics
└── confusion_matrix.png ⚠️ IMPORTANT - Performance analysis
```

**Why Essential:**
- `best.engine` - Production deployment (fastest)
- `best.pt` - Backup, can retrain or export to other formats
- Required for all Week 4+ work

**Backup Priority:** 🔴 HIGHEST

---

### 2. Dataset

**Location:** `pepper_dataset/`
**Size:** ~2.4 GB
**Status:** ✅ Critical

```
pepper_dataset/
├── data.yaml           ✅ ESSENTIAL - Dataset configuration
├── images/
│   ├── train/          ✅ ESSENTIAL (649 images)
│   └── val/            ✅ ESSENTIAL (156 images)
└── labels/
    ├── train/          ✅ ESSENTIAL (649 labels)
    └── val/            ✅ ESSENTIAL (156 labels)
```

**Why Essential:**
- Original dataset (805 images)
- Cannot be recreated (real-world data)
- Required for:
  - Retraining if needed
  - Validation testing
  - Performance benchmarking

**Backup Priority:** 🔴 HIGHEST

---

### 3. Production Deployment Scripts

**Main Scripts:**

```
✅ deploy_pepper_detection_improved.py  (Production deployment)
   - Professional logging
   - Multiple input modes
   - CSV export
   - Error handling
   - Type hints
   - Size: ~15 KB

✅ benchmark_tensorrt_improved.py       (Performance testing)
   - PyTorch vs TensorRT comparison
   - P95/P99 statistics
   - JSON output
   - Size: ~10 KB
```

**Why Essential:**
- Production-ready with best practices
- Required for Week 4 real-time testing
- All features tested and working

**Backup Priority:** 🟡 HIGH (can rewrite but time-consuming)

---

### 4. Core Configuration Files

```
✅ pepper_dataset/data.yaml     - Dataset config (class names, paths)
⚠️ stereo_calib.yaml           - Camera calibration (can recalibrate)
```

**Why Essential:**
- `data.yaml` - Required for model to load class names
- `stereo_calib.yaml` - Camera parameters (can recalibrate if lost)

**Backup Priority:** 🟡 HIGH

---

### 5. Core Documentation

```
✅ claude.md                    - Main quick reference (346 lines)
✅ docs/                        - Organized documentation
   ├── README.md               - Documentation index
   ├── setup/jetson_setup.md   - Setup guide
   ├── training/final_results.md - Training results
   ├── deployment/deployment_guide.md - Deployment guide
   └── code/CODE_IMPROVEMENTS.md - Best practices

✅ WEEK2_3_REPORT.md           - Week 2-3 comprehensive report
```

**Why Essential:**
- Quick onboarding for new team members
- Reference for setup/deployment
- Performance metrics documentation

**Backup Priority:** 🟡 HIGH (can rewrite but valuable)

---

## ⚠️ IMPORTANT FILES (Useful but Regenerable)

### 1. Helper Scripts

**Camera & Vision:**
```
⚠️ view_camera.py              - Camera testing/viewing
⚠️ collect_dataset.py          - Dataset collection tool
⚠️ stereo_calibration.py       - Camera calibration
```

**Why Important:**
- Useful for Week 4+ work
- Can be rewritten if needed
- Save time to keep

**Action:** Keep for Week 4

---

### 2. Original Deployment Scripts

```
⚠️ deploy_pepper_detection.py  - Original version
⚠️ benchmark_tensorrt.py       - Original version
```

**Why Keep:**
- Reference for comparison
- Backup if improved version has issues

**Action:** Keep in archive/

---

### 3. Calibration Files

```
⚠️ stereo_calib.yaml           - Current calibration
⚠️ stereo_calib_12mm.yaml      - 12mm baseline
⚠️ stereo_calib_48mm.yaml      - 48mm baseline
```

**Why Important:**
- Can recalibrate if lost
- Multiple baselines for testing

**Action:** Keep main one, archive others

---

### 4. Training Outputs (Other)

```
⚠️ runs/train/gpu_test_1epoch/      - GPU validation test
⚠️ runs/train/cpu_100epoch/         - CPU baseline
```

**Why Keep:**
- Performance comparison
- Historical reference

**Action:** Can delete if space needed

---

## 📚 REFERENCE FILES (Archive)

### 1. Full Documentation Archive

```
📚 archive/claude_md_archive/
   └── claude_full_2025-11-04.md    - Full history (4,546 lines)
```

**Why Keep:**
- Complete development history
- Reference for troubleshooting
- Learning resource

**Action:** Keep in archive

---

### 2. Documentation Files (Old)

```
📚 WEEK1_REPORT.md                  - Week 1 report
📚 WEEK1_SLIDES.md                  - Week 1 slides
📚 Various guides (20+ .md files)   - Setup/calibration guides
```

**Why Keep:**
- Historical reference
- Specific guides still useful

**Action:** Move to docs/archive/ if not in use

---

### 3. Old Experiments

```
📚 old_experiments/                 - Previous attempts (48 MB)
📚 archive/old_backups/             - Old dataset backups
```

**Why Keep:**
- Learning from past attempts
- Backup data

**Action:** Can delete if space critical

---

## 🗑️ EXPENDABLE FILES (Safe to Delete)

### 1. Test Files

```
🗑️ test_*.py (30+ files)           - Development test scripts
🗑️ debug_*.py                      - Debug scripts
🗑️ tune_*.py                       - Tuning scripts
🗑️ generate_*.py                   - Pattern generation
```

**Why Expendable:**
- Development/testing only
- Not needed for production
- Can recreate if needed

**Action:** ✅ Safe to delete

**Space Saved:** ~200 KB

---

### 2. Temporary Files

```
🗑️ __pycache__/                    - Python cache (24 KB)
🗑️ *.log files                     - Training logs
🗑️ training_100epochs.log
🗑️ build_monitor.log
🗑️ workflow_*.log
```

**Why Expendable:**
- Auto-generated
- Not needed after review

**Action:** ✅ Safe to delete

**Space Saved:** ~5 MB

---

### 3. Test Outputs

```
🗑️ detection_results/              - Test detection outputs (36 MB)
🗑️ test_detection_output.jpg
🗑️ test_improved_output.jpg
🗑️ test_output_*.mp4
🗑️ test_video.mp4
🗑️ tensorrt_benchmark_results.*    - Can regenerate
```

**Why Expendable:**
- Test results
- Can regenerate anytime

**Action:** ✅ Safe to delete

**Space Saved:** ~40 MB

---

### 4. Downloaded Wheels

```
🗑️ torch-2.4.0*.whl                - PyTorch wheel (999 MB)
🗑️ torch-2.5.0*.whl                - PyTorch wheel (770 MB)
🗑️ yolo11n.pt                      - Base model (5 MB)
🗑️ yolov8n.pt                      - Base model (6 MB)
```

**Why Expendable:**
- Already installed
- Can re-download if needed

**Action:** ✅ Safe to delete

**Space Saved:** ~1.7 GB

---

### 5. Unused Folders

```
🗑️ cvat/                           - CVAT attempt (456 MB)
🗑️ test_gpu/                       - GPU test files (15 MB)
```

**Why Expendable:**
- CVAT not used (chose LabelImg)
- GPU tests completed

**Action:** ✅ Safe to delete

**Space Saved:** ~470 MB

---

## 📊 Space Analysis

### Current Usage
```
Total Project Size: ~3.5 GB

Breakdown:
- pepper_dataset/     2.4 GB  ✅ ESSENTIAL
- cvat/               456 MB  🗑️ EXPENDABLE
- runs/train/          49 MB  ✅ ESSENTIAL (30MB) + ⚠️ IMPORTANT (19MB)
- old_experiments/     48 MB  📚 REFERENCE (can delete)
- detection_results/   36 MB  🗑️ EXPENDABLE
- archive/             21 MB  📚 REFERENCE (keep)
- wheels/            ~1.7 GB  🗑️ EXPENDABLE
- test files/         ~15 MB  🗑️ EXPENDABLE
- docs/               316 KB  ✅ ESSENTIAL
- scripts/            ~1 MB   ✅ ESSENTIAL + ⚠️ IMPORTANT
```

### Cleanup Potential
```
Can safely delete:
- cvat/              456 MB
- test_gpu/           15 MB
- detection_results/  36 MB
- old_experiments/    48 MB
- test outputs/       ~5 MB
- *.whl files       1.7 GB
- test_*.py files    ~1 MB
- __pycache__/       24 KB

Total Space Recoverable: ~2.26 GB (64% of total)
```

---

## 🎯 Essential Files Summary

### For Production (Week 4+)

**Minimum Required:**
```
1. runs/train/yolo11n_pepper_gpu_100epochs/weights/
   └── best.engine (8.6M)             ✅ CRITICAL

2. pepper_dataset/                     ✅ CRITICAL
   ├── data.yaml
   ├── images/ (805 images)
   └── labels/ (805 labels)

3. Scripts:
   ├── deploy_pepper_detection_improved.py   ✅ CRITICAL
   └── benchmark_tensorrt_improved.py        ✅ CRITICAL

4. Documentation:
   ├── claude.md                      ✅ CRITICAL
   └── docs/                          ✅ CRITICAL

Total: ~2.45 GB (essential only)
```

### Recommended to Keep

**Add these for full functionality:**
```
5. Helper Scripts:
   ├── view_camera.py                 ⚠️ IMPORTANT
   ├── collect_dataset.py             ⚠️ IMPORTANT
   └── stereo_calibration.py          ⚠️ IMPORTANT

6. Configuration:
   └── stereo_calib.yaml              ⚠️ IMPORTANT

7. Reports:
   ├── WEEK1_REPORT.md                📚 REFERENCE
   ├── WEEK2_3_REPORT.md              ✅ IMPORTANT
   └── CODE_IMPROVEMENTS.md           ✅ IMPORTANT

8. Archive:
   └── archive/                       📚 REFERENCE

Total: ~2.5 GB (recommended)
```

---

## 🧹 Cleanup Recommendations

### Immediate Cleanup (Safe)

```bash
# Delete temporary files
rm -rf __pycache__/
rm -f *.log

# Delete test outputs
rm -rf detection_results/
rm -f test_*.jpg test_*.mp4 test_video.mp4

# Delete downloaded wheels (already installed)
rm -f torch-*.whl
rm -f yolo*.pt  # Can re-download base models

# Delete unused attempts
rm -rf cvat/
rm -rf test_gpu/

# Space saved: ~2.26 GB
```

### Optional Cleanup (If Space Needed)

```bash
# Archive old experiments
tar -czf old_experiments_archive.tar.gz old_experiments/
rm -rf old_experiments/

# Remove extra training runs
rm -rf runs/train/gpu_test_1epoch/
rm -rf runs/train/cpu_100epoch/

# Space saved: Additional ~67 MB
```

### Keep in Archive

```bash
# These should be kept:
archive/claude_md_archive/          # Full history
pepper_dataset/                     # Critical data
runs/train/yolo11n_pepper_gpu_100epochs/  # Production model
docs/                               # Documentation
```

---

## 📦 Backup Strategy

### Priority 1: CRITICAL (Daily)
```
1. runs/train/yolo11n_pepper_gpu_100epochs/weights/
2. pepper_dataset/
3. deploy_pepper_detection_improved.py
4. benchmark_tensorrt_improved.py
5. pepper_dataset/data.yaml
```

### Priority 2: IMPORTANT (Weekly)
```
6. docs/
7. claude.md
8. WEEK2_3_REPORT.md
9. stereo_calib.yaml
10. Helper scripts (view_camera.py, collect_dataset.py)
```

### Priority 3: REFERENCE (Monthly)
```
11. archive/
12. All documentation (.md files)
13. Calibration files
```

---

## 🚀 For Week 4

### Files Needed for 3D Integration

**Essential:**
```
✅ best.engine                        - Detection model
✅ pepper_dataset/data.yaml          - Class names
✅ stereo_calib.yaml                 - Camera parameters
✅ view_camera.py                    - Camera testing
✅ deploy_pepper_detection_improved.py - Deployment base
```

**New Files to Create:**
```
🆕 deploy_3d_detection.py            - 3D detection with depth
🆕 stereo_depth_integration.py       - Depth map processing
🆕 coordinate_transform.py           - 2D to 3D conversion
```

**Can Delete Before Week 4:**
```
🗑️ All test_*.py files
🗑️ cvat/
🗑️ test_gpu/
🗑️ detection_results/
🗑️ *.whl files
🗑️ Base YOLO models (can re-download)

Space Saved: ~2.26 GB
```

---

## 📝 File Manifest

### Essential Files Checklist

```
Production:
☑ runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine
☑ runs/train/yolo11n_pepper_gpu_100epochs/weights/best.pt
☑ pepper_dataset/ (2.4 GB)
☑ pepper_dataset/data.yaml

Scripts:
☑ deploy_pepper_detection_improved.py
☑ benchmark_tensorrt_improved.py
☑ view_camera.py
☑ collect_dataset.py
☑ stereo_calibration.py

Configuration:
☑ stereo_calib.yaml

Documentation:
☑ claude.md
☑ docs/
☑ WEEK2_3_REPORT.md
☑ archive/claude_md_archive/

Total Essential Size: ~2.5 GB
Total Expendable: ~2.26 GB
```

---

## 🎯 Action Items

### Before Week 4

1. ✅ **Backup Critical Files**
   - Copy to external drive
   - Git push latest changes

2. ✅ **Clean Up Temporary Files**
   ```bash
   bash cleanup_project.sh  # See script below
   ```

3. ✅ **Verify Essential Files**
   ```bash
   bash verify_essential.sh  # See script below
   ```

4. ✅ **Document Current State**
   - This file ✅
   - Git commit

---

## 🔧 Cleanup Scripts

### cleanup_project.sh

```bash
#!/bin/bash
echo "🧹 Cleaning up project..."

# Remove Python cache
rm -rf __pycache__/
echo "✅ Removed __pycache__/"

# Remove log files
rm -f *.log
echo "✅ Removed log files"

# Remove test outputs
rm -rf detection_results/
rm -f test_*.jpg test_*.mp4 test_video.mp4
echo "✅ Removed test outputs"

# Remove wheels (already installed)
rm -f torch-*.whl yolo*.pt
echo "✅ Removed downloaded wheels"

# Remove unused folders
rm -rf cvat/ test_gpu/
echo "✅ Removed unused folders"

echo ""
echo "🎉 Cleanup complete!"
echo "Space saved: ~2.26 GB"
```

### verify_essential.sh

```bash
#!/bin/bash
echo "🔍 Verifying essential files..."

MISSING=0

# Check model
if [ -f "runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine" ]; then
    echo "✅ Production model found"
else
    echo "❌ Production model MISSING"
    MISSING=1
fi

# Check dataset
if [ -d "pepper_dataset" ] && [ -f "pepper_dataset/data.yaml" ]; then
    echo "✅ Dataset found"
else
    echo "❌ Dataset MISSING"
    MISSING=1
fi

# Check scripts
if [ -f "deploy_pepper_detection_improved.py" ]; then
    echo "✅ Deployment script found"
else
    echo "❌ Deployment script MISSING"
    MISSING=1
fi

# Check docs
if [ -f "claude.md" ] && [ -d "docs" ]; then
    echo "✅ Documentation found"
else
    echo "❌ Documentation MISSING"
    MISSING=1
fi

if [ $MISSING -eq 0 ]; then
    echo ""
    echo "🎉 All essential files present!"
else
    echo ""
    echo "⚠️  Some essential files are missing!"
fi
```

---

**Last Updated:** November 4, 2025
**Next Review:** Before Week 5
**Status:** ✅ Complete - Ready for Week 4 Cleanup
