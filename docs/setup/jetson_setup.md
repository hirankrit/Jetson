# Jetson Orin Nano Setup Guide

> Complete setup guide for Jetson Orin Nano Developer Kit

## Hardware Specifications

**Jetson Orin Nano Developer Kit**
- GPU: 1024-core NVIDIA Ampere (32 Tensor Cores)
- CPU: 6-core ARM Cortex-A78AE v8.2
- Memory: 8GB LPDDR5
- Storage: 256GB NVMe SSD
- Power: 7-15W (configurable)

## System Information

**Operating System:**
- OS: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- Kernel: Linux 5.15.148-tegra
- Architecture: aarch64 (ARM64)

**Jetson Software:**
- Jetson Linux: R36.4.4 (JetPack 6.2.1 base)
- L4T: 36.4.4

## JetPack 6.2.1 Installation

### Why JetPack 6.2.1?

Initial system had Jetson Linux R36.4.4 but **nvidia-jetpack** metapackage was not installed, missing:
- CUDA Toolkit
- cuDNN
- TensorRT
- Development tools

### Installation Process

```bash
# Update package lists
sudo apt update

# Install JetPack metapackage
sudo apt install -y nvidia-jetpack
```

**Installation Summary:**
- Time: ~15 minutes
- Downloaded: 3,374 MB (113 packages)
- Disk usage: +7.9 GB

### Installed Components

**CUDA:**
- Version: 12.6.68-1 (Release 12.6, V12.6.68)
- Location: `/usr/local/cuda-12.6/`
- Toolkit: cuda-toolkit-12-6

**cuDNN:**
- Version: 9.3.0 (90300)
- Package: 9.3.0.75-1
- Library: libcudnn.so.9

**TensorRT:**
- Version: 10.3.0.30-1+cuda12.5
- Runtime: libnvinfer10 (tensorrt-libs)
- Development: tensorrt-dev

**Additional Tools:**
- OpenCV: 4.12.0 (upgraded from base 4.8.0)
- VPI (Vision Programming Interface): 3.2.4
- Nsight Tools (Compute, Systems, Graphics)

### CUDA Environment Setup

Add to `~/.bashrc`:
```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

Apply changes:
```bash
source ~/.bashrc
```

Verify:
```bash
nvcc --version
# Cuda compilation tools, release 12.6, V12.6.68
```

## PyTorch Installation

### Challenges Faced

**Issue:** Standard PyTorch doesn't support Jetson ARM64 architecture

**Solutions Attempted:**
1. ❌ PyTorch 2.4.0 (JP 6.0) - cuDNN version mismatch
2. ✅ **PyTorch 2.5.0 (JP 6.1/6.2)** - Working but needed torchvision fix
3. ✅ **torchvision from source** - Final solution

### Final Working Setup

**PyTorch 2.5.0 (NVIDIA Build)**
```bash
wget https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl

pip3 install --no-cache-dir torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
```

**Why NVIDIA Official Build?**
- Community builds don't fully support Jetson Orin Nano
- NVIDIA NGC builds are optimized for Jetson architecture
- Proper CUBLAS and cuDNN version compatibility
- Source: [NVIDIA PyTorch NGC](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch)

**torchvision 0.20.0 (Built from Source)**

Issue: Pre-built torchvision missing `torchvision::nms` operator on ARM64

**Why Build from Source?**

pip/conda don't provide pre-built wheels for:
- ARM64 (Jetson architecture)
- CUDA 12.6
- PyTorch 2.5.0 NVIDIA build

Building from source ensures:
- Full CUDA operator support (NMS, RoI operations)
- Compatible with Jetson GPU architecture (compute capability 8.7)
- Optimized for ARM64 + CUDA environment

**Build Process:**
```bash
# Install dependencies
sudo apt install -y libjpeg-dev zlib1g-dev libpng-dev

# Download source
git clone -b v0.20.0 https://github.com/pytorch/vision.git torchvision
cd torchvision

# Build with CUDA support
export TORCH_CUDA_ARCH_LIST="8.7"  # Jetson Orin Nano compute capability
export FORCE_CUDA=1                # Force CUDA compilation
export BUILD_CUDA_SOURCES=1        # Build CUDA-specific operators

pip3 install -e .
```

Build time: ~1-2 hours on Jetson Orin Nano

**Critical Build Parameters:**
- `TORCH_CUDA_ARCH_LIST="8.7"` - Matches Jetson Orin Nano GPU architecture
- `FORCE_CUDA=1` - Ensures CUDA support is compiled
- `BUILD_CUDA_SOURCES=1` - Builds CUDA-specific vision operators

### Verification

```python
import torch
import torchvision

print(f"PyTorch: {torch.__version__}")
# PyTorch: 2.5.0a0+872d972e41.nv24.08

print(f"torchvision: {torchvision.__version__}")
# torchvision: 0.20.0a0+afc54f7

print(f"CUDA available: {torch.cuda.is_available()}")
# CUDA available: True

print(f"CUDA version: {torch.version.cuda}")
# CUDA version: 12.6

print(f"cuDNN version: {torch.backends.cudnn.version()}")
# cuDNN version: 90300

print(f"GPU: {torch.cuda.get_device_name(0)}")
# GPU: Orin
```

**Additional Package Verification:**
```bash
# Check all versions
python3 --version
# Python 3.10.12

python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')"
# OpenCV: 4.12.0

python3 -c "from ultralytics import YOLO; import ultralytics; print(f'Ultralytics: {ultralytics.__version__}')"
# Ultralytics: 8.3.223

python3 -c "import numpy; print(f'numpy: {numpy.__version__}')"
# numpy: 1.26.4

nvcc --version
# Cuda compilation tools, release 12.6, V12.6.68

dpkg -l | grep tensorrt | grep "^ii" | head -1
# tensorrt 10.3.0.30-1+cuda12.5 arm64
```

## Performance Metrics

### GPU Training Performance

With this software stack, achieved significant performance improvements:

**Training Performance:**
- ✅ GPU Training: **48 minutes** (100 epochs, YOLO11n)
- ❌ CPU Training: **22 hours** (same configuration)
- ⚡ Speedup: **28x faster** with GPU

**Inference Performance:**
- TensorRT Engine: **45ms** average (22.2 FPS)
- PyTorch Model: **58ms** average (17.3 FPS)
- TensorRT Speedup: **1.29x**

**Model Accuracy:**
- mAP@50: 89.59%
- mAP@50-95: 82.47%
- Precision: 98.63%
- Recall: 86.61%

### Key Success Factors

**Critical Software Combinations:**

1. **PyTorch 2.5.0 (NVIDIA) + CUDA 12.6 + cuDNN 9.3.0**
   - Must use NVIDIA official build (not community builds)
   - Download from: NVIDIA NGC PyTorch containers
   - Ensures proper CUBLAS and cuDNN compatibility

2. **torchvision 0.20.0 (built from source)**
   - Must build from source code
   - Requires `TORCH_CUDA_ARCH_LIST="8.7"` for Orin Nano
   - Requires `FORCE_CUDA=1` and `BUILD_CUDA_SOURCES=1`

3. **TensorRT 10.3.0 + CUDA 12.6**
   - Included with JetPack 6.2.1
   - Used to optimize model (best.engine)
   - Achieves 1.29x speedup over PyTorch

**Why These Versions Matter:**

The combination of PyTorch 2.5.0 (NVIDIA build), CUDA 12.6, and cuDNN 9.3.0 is critical:
- Community PyTorch builds have CUBLAS compatibility issues
- torchvision pre-built wheels don't support ARM64 + CUDA 12.6
- TensorRT 10.3.0 requires exact CUDA version match

**Lessons Learned:**

✅ **What Worked:**
- PyTorch 2.5.0 from NVIDIA NGC (official NVIDIA build)
- Building torchvision from source with CUDA support
- CUDA 12.6 (bundled with JetPack 6.2.1)

❌ **What Didn't Work:**
- PyTorch 2.4.0 - CUBLAS compatibility issues
- Community PyTorch builds - Poor Jetson Orin Nano support
- pip install torchvision - No pre-built ARM64+CUDA wheel

## Power Mode Configuration

```bash
# List available power modes
sudo nvpmodel -q

# Set to maximum performance (15W)
sudo nvpmodel -m 0

# Set to balanced (10W)
sudo nvpmodel -m 1

# Check current mode
sudo nvpmodel -q --verbose
```

## Useful Commands

```bash
# Check Jetson stats
sudo tegrastats

# Check CUDA devices
nvidia-smi

# Check system info
cat /etc/nv_tegra_release

# Check installed packages
dpkg -l | grep nvidia

# Check disk usage
df -h

# Check memory
free -h
```

## Troubleshooting

### GPU Not Detected

Check CUDA installation:
```bash
ls -la /usr/local/cuda
nvcc --version
```

### cuDNN Version Mismatch

Check installed version:
```bash
dpkg -l | grep cudnn
```

### torchvision Import Error

Rebuild from source with CUDA support (see above)

## References

- [NVIDIA JetPack Documentation](https://developer.nvidia.com/embedded/jetpack)
- [Jetson Orin Nano Datasheet](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide)
- [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson)

---

**Status:** ✅ Fully configured and tested
**Last Updated:** Nov 4, 2025
