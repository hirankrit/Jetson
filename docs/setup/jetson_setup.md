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
- Version: 12.6.68-1
- Location: `/usr/local/cuda-12.6/`

**cuDNN:**
- Version: 9.3.0.75-1
- Library: libcudnn.so.9

**TensorRT:**
- Version: 10.3.0.30
- Runtime: libnvinfer10

**Additional Tools:**
- OpenCV: 4.8.0
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

**torchvision 0.20.0 (Built from Source)**

Issue: Pre-built torchvision missing `torchvision::nms` operator on ARM64

Solution: Build with CUDA support
```bash
# Install dependencies
sudo apt install -y libjpeg-dev zlib1g-dev libpng-dev

# Download source
git clone -b v0.20.0 https://github.com/pytorch/vision.git torchvision
cd torchvision

# Build with CUDA
export TORCH_CUDA_ARCH_LIST="8.7"  # Jetson Orin architecture
export FORCE_CUDA=1
export BUILD_CUDA_SOURCES=1

pip3 install -e .
```

Build time: ~1-2 hours on Jetson Orin Nano

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
