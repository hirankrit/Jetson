# Changelog - Pepper Sorting Robot

## 2025-10-23

### Camera Setup Complete ✅

**Hardware:**
- ✅ IMX219 Stereo Camera enabled on Jetson Orin Nano
- ✅ Device tree merged successfully (merge_imx219_dtb.sh)
- ✅ Both cameras detected (/dev/video0, /dev/video1)
- ✅ GStreamer nvarguscamerasrc working @ 30 fps

**Software:**
- ✅ ROS2 Humble installed on host
- ✅ Created gstreamer_camera_node.py (ROS2 stereo node)
- ✅ Created stereo_camera.launch.py (ROS2 launch file)
- ✅ Created view_camera.py (Python camera viewer)

**Testing:**
- ✅ nvarguscamerasrc: Both cameras @ 1280×720, 30 fps
- ✅ ROS2 node: Publishing to 4 topics successfully
- ✅ Python viewer: Real-time display working

### Calibration Setup ✅

**Pattern Selection:**
- ✅ Chose **Asymmetric Circles Grid** (recommended for agriculture)
- Pattern: 5 rows × 13 columns, 18mm diagonal spacing
- Circle diameter: 14mm
- Physical size: ~234mm × 72mm (fits A4)

**Scripts Created:**
- ✅ capture_calibration.py - Auto-detect and capture calibration images
- ✅ stereo_calibration.py - Compute stereo calibration parameters
- ✅ CAMERA_CALIBRATION_GUIDE.md - Complete calibration guide

**Why Asymmetric Circles?**
- ✅ Better tolerance to lighting variations (natural light)
- ✅ Higher sub-pixel accuracy than checkerboard
- ✅ Unique pattern detection (no ambiguity)
- ✅ Ideal for small objects (peppers, fruits)

### Next Steps 🎯

1. **Print calibration pattern** from https://calib.io/
2. **Capture 20-30 calibration images** using capture_calibration.py
3. **Run stereo calibration** using stereo_calibration.py
4. **Test depth accuracy** and create Week 1 report

### Files Added

```
/home/jay/Project/
├── CAMERA_CALIBRATION_GUIDE.md     [NEW]
├── CHANGELOG.md                     [NEW]
├── capture_calibration.py           [NEW]
├── stereo_calibration.py            [NEW]
├── view_camera.py                   [NEW]
├── stereo_camera.launch.py          [NEW]
├── gstreamer_camera_node.py         [EXISTING]
├── setup_gstreamer_cameras.sh       [EXISTING]
├── install_ros2_humble.sh           [EXISTING]
└── merge_imx219_dtb.sh              [EXISTING]
```

### Documentation Updated

- ✅ claude.md - Added calibration workflow and pattern info
- ✅ docs/10_setup_guide.md - Updated camera setup section
- ✅ docs/11_vision_first_roadmap.md - Marked Day 1 complete

---

## Previous Updates

See git log for earlier changes.
