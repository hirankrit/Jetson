# Isaac ROS ESS - Build Complete & Test Guide

**Date:** November 6, 2025
**Status:** ✅ **BUILD SUCCESSFUL**
**Time:** ~2 hours total

---

## 🎉 Build Summary

### ✅ Successfully Completed

1. **Docker Environment**
   - Image: `isaac_ros_dev-aarch64` (6.5 GB)
   - Container: `isaac_ros_dev-aarch64-container` (running)

2. **Source Repositories** (6 repos, ~2.5 GB)
   - `isaac_ros_common`
   - `isaac_ros_dnn_stereo_depth`
   - `isaac_ros_nitros`
   - `isaac_ros_image_pipeline`
   - `isaac_ros_dnn_inference`
   - `isaac_ros_apriltag`

3. **ROS2 Packages Built** (31/31 successful)
   - Core: `isaac_ros_ess` ✅
   - Depth: `gxf_isaac_ess` ✅
   - Transport: `isaac_ros_nitros` ✅
   - All NITROS types (camera_info, image, disparity, etc.) ✅

4. **TensorRT ESS Models** ⭐
   ```
   Location: ~/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/

   Files:
   - ess.engine (34MB) - Main ESS model
   - light_ess.engine (34MB) - Lightweight variant
   - ess.onnx (66MB) - Source ONNX model
   - light_ess.onnx (65MB) - Source lightweight model
   - plugins/ - Custom TensorRT plugins
   ```

---

## 🔧 Build Issues Resolved

### 1. Magic Enum Dependency
**Problem:** `gxf_isaac_image_flip` failed - missing `magic_enum::magic_enum` target

**Solution:** Added to CMakeLists.txt:
```cmake
find_package(magic_enum REQUIRED)
```

Installed package:
```bash
apt-get install ros-humble-magic-enum
```

**File Modified:** `/workspaces/isaac_ros-dev/src/isaac_ros_image_pipeline/isaac_ros_gxf_extensions/gxf_isaac_image_flip/CMakeLists.txt`

### 2. EULA Acceptance
**Problem:** ESS model download requires EULA acceptance

**Solution:** Set environment variable:
```bash
export ISAAC_ROS_ACCEPT_EULA=1
```

### 3. Test Dependencies
**Problem:** `isaac_ros_image_proc` failed due to missing gtest

**Solution:** Disabled testing:
```bash
colcon build --cmake-args -DBUILD_TESTING=OFF
```

---

## 📊 Performance Expectations

| Metric | Value | Notes |
|--------|-------|-------|
| **FPS (ESS)** | ~178 FPS | DNN-based stereo depth |
| **FPS (SGBM)** | ~10-20 FPS | Classical method |
| **Speedup** | **~10x faster** | ESS vs StereoSGBM |
| **Latency** | <6ms per frame | On Jetson Orin Nano |
| **Accuracy** | High | DNN trained on diverse data |

---

## 🚀 How to Use Isaac ROS ESS

### Method 1: ROS2 Launch File (Recommended)

**Step 1: Start Docker Container**
```bash
cd ~/workspaces/isaac_ros-dev
docker start isaac_ros_dev-aarch64-container
docker exec -it isaac_ros_dev-aarch64-container bash
```

**Step 2: Source ROS2 Environment**
```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
```

**Step 3: Launch ESS Node**
```bash
ros2 launch isaac_ros_ess isaac_ros_ess.launch.py \
  engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/ess.engine \
  threshold:=0.4 \
  input_layer_width:=960 \
  input_layer_height:=576
```

**Step 4: Publish Stereo Images (in another terminal)**
```bash
# For ZED Mini (requires ZED ROS2 wrapper)
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedm

# OR use image publisher for test images
ros2 run image_publisher image_publisher_node <left_image.jpg>
ros2 run image_publisher image_publisher_node <right_image.jpg>
```

**Step 5: Visualize Output**
```bash
# View disparity image
ros2 run rqt_image_view rqt_image_view /disparity

# View depth
rviz2
```

---

### Method 2: Python API (Direct Use)

Coming soon - requires creating custom Python node that:
1. Loads ESS TensorRT engine
2. Processes stereo image pairs
3. Outputs disparity/depth maps

---

## 📁 Key Files & Locations

### Workspace
```
~/workspaces/isaac_ros-dev/
├── src/                           # Source repositories
├── build/                         # Build artifacts
├── install/                       # Installed packages
└── isaac_ros_assets/             # Models and assets
    └── models/dnn_stereo_disparity/
        └── dnn_stereo_disparity_v4.1.0_onnx/
            ├── ess.engine        # Main TensorRT engine ⭐
            ├── light_ess.engine  # Lightweight variant
            ├── ess.onnx          # Source ONNX model
            └── plugins/          # Custom plugins
```

### Launch Files
```
/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_depth/isaac_ros_ess/launch/
├── isaac_ros_ess.launch.py              # Basic ESS launch
├── isaac_ros_ess_core.launch.py         # Core functionality
├── isaac_ros_ess_depth.launch.py        # With depth output
├── isaac_ros_ess_realsense.launch.py    # For RealSense cameras
└── isaac_ros_argus_ess.launch.py        # For Argus cameras (Jetson native)
```

### Config Files
```
/workspaces/isaac_ros-dev/src/isaac_ros_dnn_stereo_depth/isaac_ros_ess/config/
```

---

## 🔍 Verification Commands

### Check TensorRT Engines
```bash
ls -lah ~/workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/
```

**Expected Output:**
```
-rw-r--r-- 1 root root  34M Nov  6 06:13 ess.engine
-rw-r--r-- 1 root root  34M Nov  6 06:15 light_ess.engine
```

### Verify Package Installation
```bash
docker exec isaac_ros_dev-aarch64-container bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /workspaces/isaac_ros-dev/install/setup.bash && \
   ros2 pkg list | grep isaac_ros_ess"
```

**Expected Output:**
```
isaac_ros_ess
isaac_ros_ess_models_install
```

### List Available Launch Files
```bash
docker exec isaac_ros_dev-aarch64-container bash -c \
  "source /opt/ros/humble/setup.bash && \
   source /workspaces/isaac_ros-dev/install/setup.bash && \
   ros2 launch isaac_ros_ess -s"
```

---

## 📝 Next Steps

### For Week 4 Project:

1. **✅ DONE:** Build Isaac ROS ESS packages
2. **⏳ TODO:** Test ESS with ZED Mini stereo images
3. **⏳ TODO:** Integrate ESS depth with YOLO11n detection
4. **⏳ TODO:** Create 3D detection pipeline:
   - YOLO11n: 2D bounding boxes + class
   - ESS: Depth map
   - Output: 3D coordinates (x, y, z) for each pepper

5. **⏳ TODO:** Benchmark performance:
   - ESS vs StereoSGBM speed
   - ESS vs StereoSGBM accuracy
   - End-to-end latency (detection + depth)

6. **⏳ TODO:** Build unified ROS2 node:
   ```
   Input: Stereo images from ZED Mini
   Processing: YOLO detection + ESS depth
   Output: 3D pepper positions + classifications
   ```

---

## 🐛 Troubleshooting

### Docker Container Not Running
```bash
cd ~/workspaces/isaac_ros-dev
docker start isaac_ros_dev-aarch64-container
```

### Can't Find TensorRT Engine
```bash
# Verify path inside container
docker exec isaac_ros_dev-aarch64-container ls -la \
  /workspaces/isaac_ros-dev/isaac_ros_assets/models/dnn_stereo_disparity/dnn_stereo_disparity_v4.1.0_onnx/
```

### ESS Node Fails to Start
1. Ensure ROS2 environment is sourced
2. Check TensorRT engine path is correct
3. Verify CUDA/TensorRT are available
4. Check input image size matches expectations

---

## 📚 References

- **Isaac ROS ESS Documentation:** https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth
- **Installation Guide:** `~/workspaces/isaac_ros-dev/docs/setup/isaac_ros_ess_setup.md`
- **Build Log:** `~/workspaces/isaac_ros-dev/build_complete.log`
- **Project Documentation:** `/home/jay/Project/docs/`

---

## ✅ Success Criteria Met

- [x] Docker environment setup
- [x] All dependencies resolved
- [x] 31/31 ROS2 packages built
- [x] TensorRT ESS engines created
- [x] Ready for integration testing

**Status: READY FOR TESTING** 🚀

---

**Next Session:** Test ESS with actual ZED Mini stereo images and integrate with YOLO11n pepper detection!
