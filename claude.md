# 🌶️ Pepper Sorting Robot System

> Vision-based robot system for sorting peppers by quality, size, and color

**Last Updated:** November 4, 2025 | Week 3 Complete ✅
**Full History:** [archive/claude_md_archive/claude_full_2025-11-04.md](archive/claude_md_archive/claude_full_2025-11-04.md) (4,546 lines)

---

## 📖 Table of Contents

1. [Project Overview](#-project-overview)
2. [Current Status](#-current-status)
3. [Quick Start](#-quick-start)
4. [Production Assets](#-production-assets)
5. [Software Stack](#️-software-stack)
6. [Documentation](#-documentation)
7. [Project Timeline](#-project-timeline)

---

## 🎯 Project Overview

**Goal:** Autonomous vision-based robot system for sorting peppers

**Approach:**
1. **2D Object Detection** - YOLO11n for pepper classification
2. **3D Position Estimation** - Stereo depth from ZED Mini camera
3. **Robot Control** - Coordinate-based pick & place

**Hardware:**
- **Compute:** Jetson Orin Nano Developer Kit (8GB)
- **Vision:** ZED Mini Stereo Camera
- **Robot:** TBD (Week 5-6)

**Project Duration:** 6 weeks (Week 3/6 complete)

---

## ✅ Current Status (Week 3 Complete!)

### Major Achievements 🎉

- ✅ **Dataset:** 805 images, 10 pepper classes, fully annotated
- ✅ **Training:** 100 epochs GPU (48 min), 89.59% mAP@50
- ✅ **Optimization:** TensorRT engine (1.29x speedup)
- ✅ **Deployment:** Production-ready scripts with best practices

### Performance Metrics

| Metric | Value | Status |
|--------|-------|--------|
| **mAP@50** | 89.59% | ⭐ Excellent |
| **mAP@50-95** | 82.47% | ⭐ Very Good |
| **Precision** | 98.63% | ⭐ Outstanding |
| **Recall** | 86.61% | ⭐ Good |
| **Inference (TensorRT)** | 45ms | ⚡ Real-time ready |
| **FPS** | 22.2 | ⚡ Real-time ready |

### Week 4 Goals

1. 🎯 Real-time detection testing
2. 🎯 3D stereo depth integration
3. 🎯 Robot arm coordinate system
4. 🎯 Initial ROS2 integration

---

## 🚀 Quick Start

### Prerequisites

- Jetson Orin Nano with JetPack 6.2.1
- Python 3.10+
- CUDA 12.6+
- Required packages installed (see [docs/setup/](docs/setup/))

### 1. Run Detection (Single Image)

```bash
python3 deploy_pepper_detection_improved.py \
  --model runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine \
  --image test.jpg
```

### 2. Batch Processing with CSV Export

```bash
python3 deploy_pepper_detection_improved.py \
  --folder images/ \
  --csv results.csv \
  --no-display
```

### 3. Video Processing

```bash
python3 deploy_pepper_detection_improved.py \
  --video input.mp4 \
  --skip-frames 2 \
  --save output.mp4
```

### 4. Performance Benchmark

```bash
python3 benchmark_tensorrt_improved.py \
  --runs 100 \
  --format json
```

**Expected Performance (Jetson Orin Nano):**
- TensorRT: 45ms avg, 22.2 FPS ⚡
- PyTorch: 58ms avg, 17.3 FPS
- Speedup: **1.29x**

---

## 📁 Production Assets

### Trained Model ⭐

**Location:** `runs/train/yolo11n_pepper_gpu_100epochs/`

**Files:**
- `weights/best.engine` (8.6M) - **TensorRT (recommended)** ⚡
- `weights/best.pt` (5.3M) - PyTorch model
- `weights/best.onnx` (11M) - ONNX format
- `results.csv` - Training metrics
- `results.png` - Training curves
- `confusion_matrix.png` - Per-class performance

### Dataset

**Location:** `pepper_dataset/`
- **Images:** 805 (649 train / 156 val)
- **Classes:** 10 pepper types
  - Green: large, medium, small, rotten
  - Red: large, small, deformed, wrinkled, insect, rotten
- **Format:** YOLO (normalized bbox)
- **Config:** `data.yaml`

### Deployment Scripts

**Main Scripts:**
- `deploy_pepper_detection_improved.py` - Production deployment ⭐
- `benchmark_tensorrt_improved.py` - Performance benchmarking

**Features:**
- ✅ Professional logging with timestamps
- ✅ Type hints throughout
- ✅ Comprehensive error handling
- ✅ Multiple output formats (txt/json/csv)
- ✅ CLI argument parsing
- ✅ Input validation

---

## 🛠️ Software Stack

### System
- **OS:** Ubuntu 22.04.5 LTS (Jetson Linux R36.4.4)
- **JetPack:** 6.2.1
- **Python:** 3.10.12

### Deep Learning
- **PyTorch:** 2.5.0a0+872d972e41.nv24.08 (NVIDIA build)
- **torchvision:** 0.20.0a0 (built from source with CUDA)
- **CUDA:** 12.6.68
- **cuDNN:** 9.3.0
- **TensorRT:** 10.3.0

### Computer Vision
- **Ultralytics:** 8.3.223 (YOLO11)
- **OpenCV:** 4.10.0
- **ZED SDK:** (for stereo camera)

### Development Tools
- Git, VSCode, Python tools
- Docker (for optional services)

---

## 📚 Documentation

> **📌 For detailed guides, see [docs/](docs/) folder**

### Quick Links

**Setup & Installation:**
- [Jetson Setup Guide](docs/setup/jetson_setup.md) - JetPack, CUDA, PyTorch

**Dataset:**
- [Dataset Statistics](docs/dataset/dataset_statistics.md) - 805 images, 10 classes

**Training:**
- [Training Results](docs/training/final_results.md) - 89.59% mAP@50 ⭐

**Deployment:**
- [Deployment Guide](docs/deployment/deployment_guide.md) - Production deployment ⭐

**Code:**
- [Code Improvements](docs/code/CODE_IMPROVEMENTS.md) - Best practices applied

**Development History:**
- [Complete Archive](archive/claude_md_archive/claude_full_2025-11-04.md) - Full 4,546 line history
- [Session Summary](docs/development_log/daily_notes/2025-11-04.md) - Nov 4 session
- [Documentation Index](docs/README.md) - All documentation

### Documentation Structure

```
docs/
├── README.md                      # Documentation index
├── setup/                         # Installation guides
│   └── jetson_setup.md           ✅
├── dataset/                       # Dataset documentation
│   └── dataset_statistics.md     ✅
├── training/                      # Training documentation
│   └── final_results.md          ✅
├── deployment/                    # Deployment guides
│   └── deployment_guide.md       ✅
├── code/                          # Code documentation
│   └── CODE_IMPROVEMENTS.md      ✅
├── development_log/               # Development history
│   └── daily_notes/
│       └── 2025-11-04.md         ✅
└── troubleshooting/               # Problem solving
```

---

## 📊 Project Timeline

### Week 1 (Oct 21-27): Planning & Initial Setup
- System architecture design
- Hardware selection
- Development environment setup

### Week 2 (Oct 28-Nov 3): Dataset Collection
- ✅ 805 images collected (12 sessions)
- ✅ 10 pepper classes defined
- ✅ Annotation pipeline established
- ✅ Quality verification complete

### Week 3 (Nov 3-4): Training & Optimization ✅
**Completed:**
- ✅ GPU setup (PyTorch 2.5.0 + torchvision from source)
- ✅ 100-epoch training (48 minutes on GPU)
- ✅ Model performance: 89.59% mAP@50
- ✅ TensorRT optimization (1.29x speedup)
- ✅ Production deployment scripts
- ✅ Code quality improvements

**Key Achievements:**
- 28x faster training (GPU vs CPU)
- Production-ready model
- Complete documentation structure

### Week 4 (Nov 5-11): 3D Integration (Planned)
- Real-time detection testing
- Stereo depth integration
- 3D position calculation
- Coordinate system calibration

### Week 5 (Nov 12-18): Robot Integration (Planned)
- Robot arm selection & setup
- ROS2 node development
- Pick & place testing
- System integration

### Week 6 (Nov 19-25): Testing & Deployment (Planned)
- End-to-end system testing
- Performance optimization
- Documentation finalization
- Final demo

---

## 🎯 Key Learnings (Week 3)

### Technical Achievements
1. **GPU Training Success**
   - 48 minutes for 100 epochs (vs 22 hours on CPU)
   - 28x speedup with proper PyTorch/CUDA setup

2. **Model Performance**
   - 89.59% mAP@50 - Excellent for production
   - 98.63% precision - Very few false positives
   - 45ms inference (TensorRT) - Real-time capable

3. **Best Practices Applied**
   - Professional logging with timestamps
   - Type hints throughout codebase
   - Comprehensive error handling
   - Multiple output formats (txt/json/csv)

### Problem Solving
1. **PyTorch on Jetson**
   - Used NVIDIA official builds (not community)
   - Built torchvision from source with CUDA
   - Proper CUBLAS version matching critical

2. **Dataset Quality**
   - 805 images sufficient for 10 classes
   - YOLO pre-annotation saved 75% time
   - Stratified train/val split important

---

## 🔗 Quick Links

- **GitHub:** [hirankrit/Jetson](https://github.com/hirankrit/Jetson)
- **Full History:** [archive/claude_full_2025-11-04.md](archive/claude_md_archive/claude_full_2025-11-04.md)
- **All Documentation:** [docs/](docs/)
- **Latest Session:** [docs/development_log/daily_notes/2025-11-04.md](docs/development_log/daily_notes/2025-11-04.md)

---

## 📞 Reference

**For detailed information:**
- **Setup issues:** See [docs/setup/jetson_setup.md](docs/setup/jetson_setup.md)
- **Training details:** See [docs/training/final_results.md](docs/training/final_results.md)
- **Deployment guide:** See [docs/deployment/deployment_guide.md](docs/deployment/deployment_guide.md)
- **Code improvements:** See [docs/code/CODE_IMPROVEMENTS.md](docs/code/CODE_IMPROVEMENTS.md)
- **Complete history:** See [archive/claude_md_archive/claude_full_2025-11-04.md](archive/claude_md_archive/claude_full_2025-11-04.md)

**Repository Structure:**
```
Project/
├── claude.md                          ⭐ This file (quick reference)
├── docs/                              📚 Organized documentation
├── archive/                           🗄️ Historical archive
├── runs/train/yolo11n_pepper_gpu_100epochs/  🎯 Production model
├── pepper_dataset/                    📊 Dataset (805 images)
├── deploy_pepper_detection_improved.py     🚀 Deployment script
├── benchmark_tensorrt_improved.py          ⚡ Benchmarking
└── [other project files...]
```

---

**Status:** ✅ Week 3 Complete - Model Ready for Production Deployment
**Next:** Week 4 - 3D Stereo Integration
**Model:** `runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine`
