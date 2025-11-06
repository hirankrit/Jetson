# 🤖 Isaac-agri-robot

> **Isaac ROS-based Agricultural Robot for Pepper Sorting**
>
> Vision system combining YOLO11n object detection with stereo depth estimation for autonomous pepper sorting by quality, size, and color.

[![Platform](https://img.shields.io/badge/Platform-NVIDIA%20Jetson%20Orin%20Nano-76B900?logo=nvidia)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)
[![Isaac ROS](https://img.shields.io/badge/Isaac%20ROS-ESS%20Depth-76B900?logo=ros)](https://nvidia-isaac-ros.github.io/)
[![YOLO11n](https://img.shields.io/badge/YOLO-11n-00FFFF?logo=yolo)](https://github.com/ultralytics/ultralytics)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python)](https://www.python.org/)

---

## 📖 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Hardware](#-hardware)
- [Quick Start](#-quick-start)
- [Performance](#-performance)
- [Documentation](#-documentation)
- [Project Timeline](#-project-timeline)
- [License](#-license)

---

## 🎯 Overview

This project implements an end-to-end vision system for agricultural robotics, specifically designed for pepper sorting applications. It combines:

1. **2D Object Detection** using YOLO11n for pepper classification (10 classes)
2. **3D Depth Estimation** using stereo vision (StereoSGBM + Isaac ROS ESS)
3. **3D Coordinate Calculation** for robot pick-and-place operations

### System Pipeline

```
Camera → 2D Detection → Depth Estimation → 3D Coordinates → Robot Control
 (IMX219)  (YOLO11n)    (StereoSGBM/ESS)   (Triangulation)   (TBD)
```

---

## ✨ Features

### 2D Object Detection
- ✅ **YOLO11n** model trained on 805 pepper images
- ✅ **10 Classes**: green (large/medium/small/rotten), red (large/small/deformed/wrinkled/insect/rotten)
- ✅ **89.59% mAP@50** accuracy
- ✅ **TensorRT optimization** (1.29x speedup, 22 FPS on Jetson Orin Nano)
- ✅ **Production-ready** deployment scripts

### 3D Depth Estimation
- ✅ **Stereo Vision** using IMX219 dual camera (60mm baseline)
- ✅ **Auto Preprocessing** for cheap cameras (+359% improvement!)
- ✅ **StereoSGBM** (classical method) - baseline established
- 🔄 **Isaac ROS ESS** (DNN-based) - in progress (10x faster expected)
- ✅ **Calibrated** stereo system (59.95mm baseline)

### System Integration
- ✅ **Jetson Orin Nano** optimized (8GB)
- ✅ **CUDA 12.6** + TensorRT inference
- 🔄 **ROS2 Humble** integration (in progress)
- ✅ **Complete documentation** for reproducibility

---

## 🛠️ Hardware

| Component | Specification |
|-----------|---------------|
| **Compute** | NVIDIA Jetson Orin Nano Developer Kit (8GB) |
| **OS** | Ubuntu 22.04 LTS (JetPack 6.2.1) |
| **Camera** | IMX219-83 Stereo Camera (Dual 8MP, 60mm baseline, 83° FoV) |
| **Interface** | CSI-2 (direct to Jetson) |
| **Robot** | TBD (Week 5-6) |

**Cost:**
- Jetson Orin Nano: ~$500 USD
- IMX219 Stereo Camera: ~60 USD ($1,800 THB from China)
- **Total Vision System: ~$560 USD** 🎯

---

## 🚀 Quick Start

### Prerequisites

```bash
# System requirements
- NVIDIA Jetson Orin Nano with JetPack 6.2.1
- Python 3.10+
- CUDA 12.6+
- 8GB RAM minimum
```

### Installation

```bash
# 1. Clone repository
git clone https://github.com/hirankrit/Isaac-agri-robot.git
cd Isaac-agri-robot

# 2. Install dependencies
pip3 install -r requirements.txt

# 3. Download model weights (if not included)
# See docs/setup/model_download.md
```

### Usage

#### 1. Run 2D Detection (Single Image)

```bash
python3 src/detection/deploy_pepper_detection.py \
  --model models/detection/yolo11n_pepper.engine \
  --image test.jpg
```

#### 2. Capture Stereo Images

```bash
./src/depth/capture_stereo_gst.sh
```

#### 3. Compute Depth Map

```bash
python3 src/depth/test_stereo_depth.py \
  left_image.jpg right_image.jpg
```

#### 4. 3D Detection Pipeline (Coming Soon)

```bash
python3 src/integration/yolo_depth_3d.py \
  --model models/detection/yolo11n_pepper.engine \
  --left left.jpg --right right.jpg
```

---

## 📊 Performance

### Detection Performance

| Metric | Value | Status |
|--------|-------|--------|
| **mAP@50** | 89.59% | ⭐ Excellent |
| **mAP@50-95** | 82.47% | ⭐ Very Good |
| **Precision** | 98.63% | ⭐ Outstanding |
| **Recall** | 86.61% | ⭐ Good |
| **Inference (TensorRT)** | 45ms | ⚡ Real-time |
| **FPS** | 22.2 | ⚡ Real-time |

### Depth Estimation Performance

| Method | Valid Pixels | FPS | Status |
|--------|--------------|-----|--------|
| **StereoSGBM** (baseline) | 25-29% | 18 | ✅ Working |
| **StereoSGBM + Preprocessing** | 29% | 18 | ⭐ +359% improvement |
| **Isaac ROS ESS** (expected) | 80-90% | 178 | 🔄 In progress |

**Key Findings:**
- Auto preprocessing improves cheap camera by **+359%** (6.3% → 27.0% valid pixels)
- Lighting conditions account for **60-70%** of depth quality
- Isaac ROS ESS expected to be **10x faster** than StereoSGBM

---

## 📚 Documentation

### Setup Guides
- [Jetson Setup](docs/setup/jetson_setup.md) - JetPack, CUDA, PyTorch installation
- [Isaac ROS Setup](docs/setup/isaac_ros_ess_setup.md) - Isaac ROS ESS installation
- [Camera Setup](docs/hardware/camera_setup.md) - IMX219 configuration

### Hardware Documentation
- [Camera Calibration Guide](docs/hardware/calibration_guide.md) - Stereo calibration process
- [Pattern Printing Guide](docs/hardware/pattern_printing.md) - Calibration pattern preparation
- [Hardware Specifications](docs/hardware/specs.md) - Complete hardware specs

### Training & Deployment
- [Dataset Collection](docs/training/dataset_collection.md) - How to collect pepper images
- [Training Guide](docs/training/training_guide.md) - Model training process
- [Training Results](docs/training/results.md) - Performance analysis
- [Deployment Guide](docs/deployment/deployment_guide.md) - Production deployment

### Development
- [Stereo Preprocessing](src/depth/README.md) - Auto preprocessing library
- [3D Integration](src/integration/README.md) - YOLO + Depth pipeline
- [Testing](tests/README.md) - Unit tests

---

## 📅 Project Timeline

### Week 1-2: Planning & Dataset (Complete ✅)
- System architecture design
- Hardware selection & setup
- **805 images** collected across 10 pepper classes
- Annotation pipeline with CVAT

### Week 3: Training & Optimization (Complete ✅)
- GPU setup (PyTorch 2.5.0 on Jetson)
- 100-epoch training (48 min on GPU, 28x faster than CPU)
- **89.59% mAP@50** achieved
- TensorRT optimization (1.29x speedup)
- Production deployment scripts

### Week 4: 3D Integration (In Progress 🔄)
- **Day 1** (Nov 6):
  - ✅ Isaac ROS ESS installation (31/31 packages)
  - ✅ IMX219 stereo camera working
  - ✅ StereoSGBM baseline established
  - ✅ **Auto preprocessing library** (+359% improvement!)
- **Day 2-7** (Planned):
  - Camera re-calibration
  - Isaac ROS ESS testing
  - 3D coordinate calculation
  - System integration

### Week 5: Robot Integration (Planned)
- Robot arm selection & setup
- ROS2 node development
- Pick & place testing
- System integration

### Week 6: Testing & Deployment (Planned)
- End-to-end testing
- Performance optimization
- Documentation finalization
- Final demonstration

---

## 🔧 Key Components

### 1. Auto Preprocessing Library ⭐ NEW!

Optimized for cheap cameras (IMX219 1800 THB):

```python
from src.depth.stereo_preprocess import auto_preprocess, get_preset

# Simple usage (best for cheap cameras)
enhanced = auto_preprocess(image, method="denoise")

# Or use preset
settings = get_preset("cheap_camera")
enhanced = auto_preprocess(image, **settings)
```

**Results:**
- **+359% improvement** in valid pixels (6.3% → 27%)
- 5 methods: denoise, CLAHE, bilateral, combo, none
- 4 presets: cheap_camera, dark_scene, normal, high_quality

### 2. GStreamer Camera Capture

Direct camera capture without OpenCV GStreamer dependency:

```bash
./src/depth/capture_stereo_gst.sh output_prefix
```

### 3. Complete Stereo Depth Pipeline

```python
python3 src/depth/test_stereo_depth.py left.jpg right.jpg
```

Includes:
- Calibration loading
- Image rectification
- Disparity computation
- Depth calculation
- Visualization (5 output images)

---

## 🎓 Key Learnings

1. **GPU Training**: 28x speedup (48 min vs 22 hours)
2. **Cheap Cameras Work**: With preprocessing, 1800 THB camera performs well
3. **Lighting Matters**: 60-70% impact on depth quality
4. **TensorRT**: 1.29x speedup, essential for real-time
5. **Isaac ROS**: DNN-based depth is game-changer (10x expected improvement)

---

## 🤝 Contributing

Contributions welcome! Please read our contributing guidelines.

---

## 📄 License

MIT License - see [LICENSE](LICENSE) file

---

## 🙏 Acknowledgments

- **NVIDIA** - Isaac ROS, Jetson platform
- **Ultralytics** - YOLO11n implementation
- **OpenCV** - Computer vision algorithms
- **ROS2** - Robot Operating System

---

## 📞 Contact

- **Author**: Hirankrit
- **Repository**: [github.com/hirankrit/Isaac-agri-robot](https://github.com/hirankrit/Isaac-agri-robot)
- **Old Repository** (archived): [github.com/hirankrit/Jetson](https://github.com/hirankrit/Jetson)

---

## 📈 Status

**Current Status**: Week 4/6 - 3D Integration in progress

**Next Milestone**: 3D coordinate calculation from YOLO11n + stereo depth

**Production Ready**: 2D detection system ✅

**3D System**: In development 🔄

---

**Last Updated**: November 6, 2025

**Made with** ❤️ **for agricultural robotics**
