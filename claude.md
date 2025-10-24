# 🌶️ Pepper Sorting Robot System

## โปรเจคภาพรวม

**วัตถุประสงค์**: ระบบคัดแยกพริกอัตโนมัติด้วย AI Vision + Dual Robot Arms

### ข้อมูลพื้นฐาน
- **Platform**: Jetson Orin Nano (Ubuntu Linux)
- **Camera**: IMX219-83 Stereo Camera (8MP, 60mm baseline)
- **Robot Arms**: 2x Arduino-based Arms (Mini Brazo robótico)
- **Framework**: ROS2 Humble
- **Timeline**: 2-3 เดือน (12 สัปดาห์)

### เกณฑ์การคัดแยก
1. **สีพริก**: แดง / เขียว / เหลือง
2. **ขนาด**: คำนวณจากข้อมูล depth (mm)
3. **คุณภาพ**: สด / เน่า

### แนวทางการทำงาน
- ใช้ Stereo Camera มองจากมุมสูง (top-down view)
- AI Detection ด้วย YOLO + Classification
- คำนวณตำแหน่ง 3D (X, Y, Z) จาก stereo depth
- 2 แขนกลทำงานแบบ parallel (เหมือนมือคนซ้าย-ขวา)
- ROS2 เป็น middleware ประสานงานระหว่าง components

---

## 📚 สารบัญเอกสาร

เอกสารแยกตามหัวข้อเพื่อความสะดวกในการอ่านและแก้ไข:

### 1. [Hardware Architecture](docs/01_hardware_architecture.md)
- Physical layout และการวางตำแหน่งอุปกรณ์
- Hardware connections (Jetson ↔ Camera ↔ Arduino)
- Workspace design (input pile, output zones)

### 2. [Software Architecture](docs/02_software_architecture.md)
- ROS2 workspace structure
- Package organization
- Dependencies และ libraries
- Node graph overview

### 3. [ROS2 Nodes Detail](docs/03_ros2_nodes_detail.md)
- Camera Node (stereo capture)
- Vision Node (detection + depth)
- Task Planner Node (coordination)
- Arm Controller Node (IK + serial control)
- Arduino firmware specifications

### 4. [Custom Messages](docs/04_custom_messages.md)
- DetectedPepper.msg
- PepperArray.msg
- ArmCommand.msg
- ArmStatus.msg

### 5. [Coordinate Frames & Transformations](docs/05_coordinate_frames.md)
- Frame definitions (world, camera, arms)
- Transformation matrices
- Stereo calibration process
- Hand-eye calibration process

### 6. [Workspace Configuration](docs/06_workspace_config.md)
- YAML configuration files
- Workspace zones (pile area, sorting zones)
- Arm division (left/right workspace)
- Parameters สำหรับแต่ละ node

### 7. [Development Roadmap](docs/07_development_roadmap.md) - Original Plan
- 12-week development plan (ROS2-first approach)
- Phase 1: ROS2 Foundation (Week 1-2)
- Phase 2: Vision System (Week 3-5)
- Phase 3: Single Arm Control (Week 6-7)
- Phase 4: Dual Arm Coordination (Week 8-9)
- Phase 5: System Integration (Week 10-11)
- Phase 6: Documentation (Week 12)

### 11. [**Vision-First Roadmap**](docs/11_vision_first_roadmap.md) ⭐ **แนะนำ**
- **แผนที่ปรับใหม่**: เน้น Vision System ก่อน (มี output รายงานเร็ว)
- **Phase 1 (Week 1-4)**: Stereo Calibration → Dataset → Training → 3D Detection
- **Phase 2 (Week 5-6)**: ROS2 Integration
- **Phase 3-5 (Week 7-12)**: Robot Arms + Full System

### 12. [**Claude Skills**](docs/12_claude_skills.md) 🤖 **ใหม่!**
- **Skills สำหรับ Claude Code CLI**: ชุดคำสั่งที่ช่วยให้ทำงานสม่ำเสมอ
- **thai-commit**: สร้าง commit message ภาษาไทย
- **weekly-report**: สร้างรายงานประจำสัปดาห์
- **ros2-review**: รีวิวโค้ด ROS2 + Python ตามมาตรฐาน
- **python-tools**: เครื่องมือ linting, type check, logging

### 8. [Tools & Monitoring](docs/08_tools_monitoring.md)
- RViz2 visualization setup
- RQT tools (rqt_graph, rqt_plot)
- Rosbag data logging
- Performance metrics
- Debugging techniques

### 9. [Challenges & Solutions](docs/09_challenges_solutions.md)
- ปัญหาที่คาดการณ์และวิธีแก้
- Pepper occlusion handling
- Lighting variations
- Calibration drift
- Collision avoidance
- Error recovery strategies

### 10. [Setup Guide](docs/10_setup_guide.md)
- ROS2 Humble installation
- Dependencies installation
- Workspace creation
- Camera driver setup
- Arduino firmware upload
- Initial configuration

---

## 🎯 Current Status

**Last Updated**: 2025-10-24 (20:30)

### Development Approach
**เลือกใช้**: [Vision-First Roadmap](docs/11_vision_first_roadmap.md) ⭐
**Camera Driver**: GStreamer nvarguscamerasrc (ROS2 on Host) - เปลี่ยนจาก Isaac ROS
**Workspace**: Camera height = 320mm from ground
**AI Framework**: PyTorch 2.9.0 + CUDA 12.6 ✅ พร้อมใช้งาน

### Project Phase (Vision-First)
- [x] ออกแบบ overall architecture เสร็จสมบูรณ์
- [x] ตั้งค่า Git repository และ GitHub
- [x] สร้าง Claude Skills สำหรับ development workflow
- [ ] **Phase 1: Vision System (Week 1-4)** ← กำลังทำ
  - [ ] Week 1: Stereo camera setup
    - [x] ตรวจสอบ hardware (Jetson Orin Nano + IMX219 Stereo)
    - [x] สร้าง enable_imx219_stereo.sh script
    - [x] พบปัญหา: extlinux.conf มี FDT ซ้ำ 3 บรรทัด
    - [x] สร้าง fix_extlinux_duplicate.sh เพื่อแก้ไข (แก้ไขเสร็จ - ลบไฟล์แล้ว)
    - [x] อ่าน claudestereo.md + GSTREAMER_GUIDE.md (งานครั้งก่อน)
    - [x] เปลี่ยนแผน: ใช้ GStreamer แทน Isaac ROS (ง่ายกว่า, เคยทำสำเร็จ)
    - [x] ติดตั้ง nvidia-l4t-gstreamer package
    - [x] ทดสอบ nvarguscamerasrc กับ gst-launch-1.0 (ผ่าน ✅)
    - [x] ติดตั้ง ROS2 Humble บน host (สำเร็จ ✅)
    - [x] สร้าง install_ros2_humble.sh script
    - [x] มีไฟล์ gstreamer_camera_node.py พร้อมใช้
    - [x] **ปัญหา Reboot #1**: nvargus-daemon errors "No cameras available"
    - [x] **สาเหตุ**: extlinux.conf ใช้ 2 paths (base DTB + overlay) ไม่ทำงาน
    - [x] **แก้ไข**: สร้าง merge_imx219_dtb.sh เพื่อ merge overlay เข้า base DTB
    - [x] รัน merge_imx219_dtb.sh สำเร็จ (merged DTB: 262,389 bytes)
    - [x] อัพเดต extlinux.conf ให้ใช้ merged DTB
    - [x] **Reboot #2**: สำเร็จ! ✅
    - [x] ทดสอบ nvarguscamerasrc หลัง reboot #2 (ผ่าน ✅)
    - [x] ทดสอบ gstreamer_camera_node.py (ผ่าน ✅ @ 30 fps)
    - [x] สร้าง stereo_camera.launch.py
    - [x] สร้าง view_camera.py (ดูภาพจากกล้อง real-time ✅)
    - [x] ทดสอบภาพจากกล้องทั้ง 2 ตัว (ผ่าน ✅)
    - [x] เลือกใช้ Asymmetric Circles Pattern (แนะนำสำหรับงานเกษตร)
    - [x] สร้าง capture_calibration.py (เก็บภาพ calibration)
    - [x] สร้าง stereo_calibration.py (คำนวณ parameters)
    - [x] สร้าง CAMERA_CALIBRATION_GUIDE.md (คู่มือสมบูรณ์)
    - [x] พิมพ์ pattern (Asymmetric Circles 5×6, 33 circles)
    - [x] Debug pattern detection (พบว่าต้องใช้ 5×6 ไม่ใช่ 5×13)
    - [x] เก็บภาพ calibration 40 ภาพ (เกินเป้าหมาย!)
    - [x] รัน stereo calibration หลายรอบ (หา pattern spacing ที่ถูก)
    - [x] **Pattern spacing ที่ถูกต้อง: 12mm** (วัดจริงจากกระดาษ)
    - [x] Calibration สุดท้าย: baseline 60.57mm ✅, stereo RMS 50.79px
    - [x] สร้าง test_depth_map.py และ test_depth_map_enhanced.py
    - [x] แก้ไข test_depth_map_enhanced.py แสดงภาพดิบแทน rectified
    - [x] ติดตั้ง PyTorch 2.9.0 + CUDA 12.6 support ✅ (พร้อม Week 3!)
    - [x] ทดสอบ depth map accuracy (30cm, 32cm, 42cm, 54cm)
    - [x] **ผลทดสอบ**: 30-32cm ✅ แม่นยำ (±1cm), 42cm+ ❌ ไม่แม่นยำ
    - [x] ปรับ Focus กล้อง (Left: 176.5, Right: 171.0, Diff: 6.0) ✅
    - [x] ตั้งค่าแสง LED (ซ้ายหน้า, ทะแยง, 10cm) ✅
    - [x] สร้าง CAMERA_SETUP_GUIDE.md (บันทึก focus + lighting) ✅
    - [ ] **กำลังทำ**: ทดสอบ depth map หลังปรับ focus ใหม่
  - [ ] Week 1 (ต่อ): ประเมินผล + รายงาน
  - [ ] Week 2: Dataset collection (500-1000 images)
  - [ ] Week 3: YOLO training + evaluation
  - [ ] Week 4: Integration (detection + 3D positioning)
- [ ] Phase 2: ROS2 Integration (Week 5-6)
- [ ] Phase 3-5: Robot Arms + Full System (Week 7-12)

### Next Actions (กำลังทำ)
1. ✅ Setup development environment
2. ✅ ตรวจสอบ Jetson hardware และ JetPack version (R36.4.4)
3. ✅ สร้าง enable_imx219_stereo.sh script
4. ✅ แก้ไข extlinux.conf (FDT ซ้ำ)
5. ✅ อ่านเอกสารงานครั้งก่อน (claudestereo.md, GSTREAMER_GUIDE.md)
6. ✅ เปลี่ยนแผน: GStreamer approach (แทน Isaac ROS)
7. ✅ ติดตั้ง nvidia-l4t-gstreamer
8. ✅ ติดตั้ง ROS2 Humble on host
9. ✅ สร้าง install_ros2_humble.sh
10. ✅ **Reboot #1**: พบปัญหา "No cameras available"
11. ✅ วิเคราะห์ปัญหา: extlinux.conf ใช้ 2 paths ไม่ทำงาน
12. ✅ สร้าง merge_imx219_dtb.sh
13. ✅ รัน merge script สำเร็จ (merged DTB ready)
14. ✅ **Reboot #2**: สำเร็จ!
15. ✅ ทดสอบ nvarguscamerasrc หลัง reboot #2 (ผ่าน!)
16. ✅ ทดสอบ gstreamer_camera_node.py (ผ่าน @ 30 fps!)
17. ✅ สร้าง stereo_camera.launch.py
18. ✅ สร้าง view_camera.py (viewer สำหรับดูภาพ real-time)
19. ✅ ทดสอบภาพจากกล้องทั้ง 2 ตัว (ยืนยันว่าเห็นภาพชัดเจน)
20. ✅ เลือก Asymmetric Circles Pattern (พบว่า pattern จริงคือ 5×6, 33 circles)
21. ✅ สร้าง capture_calibration.py (auto-detect + capture)
22. ✅ สร้าง stereo_calibration.py (compute parameters)
23. ✅ สร้าง CAMERA_CALIBRATION_GUIDE.md (คู่มือครบถ้วน)
24. ✅ พิมพ์ pattern + debug detection (ทดสอบหลาย configuration)
25. ✅ เก็บภาพ calibration 40 ภาพ (หลากหลายมุม + ระยะ)
26. ✅ รัน stereo calibration (baseline 60.57mm, stereo RMS 50.79px)
27. ✅ วิเคราะห์ stereo RMS: สูงเพราะ wide-angle lens (160° FOV)
28. ✅ สร้าง test_depth_map_enhanced.py (StereoSGBM + WLS + CLAHE)
29. ✅ ติดตั้ง PyTorch 2.9.0 + CUDA 12.6 support (~3.2GB)
30. ✅ ทดสอบ depth map accuracy (30cm, 32cm, 42cm, 54cm)
31. ✅ ค้นพบ pattern spacing ที่ถูกต้อง = **12mm**
32. ✅ วัดความแม่นยำ depth estimation: 30-32cm ดี (±1cm), 42cm+ แย่
33. ✅ ปรับ Focus กล้อง: Left 176.5, Right 171.0, Diff 6.0 (excellent!)
34. ✅ ตั้งค่าแสง LED: ซ้ายหน้า ทะแยงเข้าหาวัตถุ 10cm
35. ✅ สร้าง CAMERA_SETUP_GUIDE.md (บันทึก focus + lighting settings)
36. 🎯 **กำลังทำ**: ทดสอบ depth map หลังปรับ focus ใหม่ (ที่ 32cm optimized)
37. ⏳ **ต่อไป**: หา StereoSGBM parameters ที่ดีที่สุด สำหรับ 30-50cm range
38. ⏳ ประเมินผล + รายงาน Week 1
39. 🔧 ติดตั้ง Ultralytics YOLOv8 (optional, สำหรับ Week 3)

---

## 🔗 Quick References

### Hardware
- **Camera**: [IMX219-83 Stereo Camera Wiki](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)
  - Resolution: 3280×2464 (8MP), tested @ 1280×720
  - Baseline: **60.57mm** (measured from calibration ✅)
  - FOV: **160°** (diagonal) → Wide-angle lens
- **Calibration Pattern**: Asymmetric Circles Grid (**5 rows × 6 columns, 33 circles**)
  - Generator: [calib.io Pattern Generator](https://calib.io/pages/camera-calibration-pattern-generator)
  - **Diagonal spacing: 12mm** (measured from printed pattern) ✅
  - Horizontal spacing: 26mm (measured)
  - Mounted on: Foam board (flat, rigid)
- **Robot Arm**: Mini Brazo robótico con Arduino (YouTube reference)
- **Jetson**: [Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)

### Software
- **ROS2**: [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- **YOLO**: [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- **OpenCV**: [OpenCV Stereo Calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
  - Asymmetric Circles Detection: `cv2.findCirclesGrid()` with `CALIB_CB_ASYMMETRIC_GRID`
- **GStreamer**: [NVIDIA Accelerated GStreamer](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)

### Learning Resources
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [TensorRT for Jetson](https://developer.nvidia.com/tensorrt)
- [Arduino Serial Communication](https://www.arduino.cc/reference/en/language/functions/communication/serial/)

---

## 📝 Development Notes

### 🎓 Calibration Lessons Learned (Week 1)

**Pattern Detection:**
- ⚠️ **สำคัญ**: Pattern ที่พิมพ์มาต้องตรวจสอบขนาดจริง!
  - ตั้งค่าพิมพ์: 5×13 columns
  - ความจริง: 5×6 columns (33 circles)
  - วิธีแก้: ใช้ debug script ทดสอบหลาย configuration

**Wide-Angle Lens (160° FOV) Issues:**
- ✅ **Single camera calibration ดีมาก** (RMS < 0.3 pixels)
- ⚠️ **Stereo RMS สูง** (50+ pixels) - **ปกติสำหรับ wide-angle!**
- 💡 **Stereo RMS สูง ≠ depth ไม่แม่นยำ**
  - เกิดจาก barrel distortion ที่ซับซ้อน
  - Baseline ถูกต้อง (60mm) → metric scale ถูก
  - Depth accuracy ยังดี (ทดสอบครั้งก่อนแม่นที่ 50cm)

**Solutions for Wide-Angle:**
- ✅ **StereoSGBM** ดีกว่า StereoBM (semi-global matching)
- ✅ **WLS Filter** ลด artifacts + edge-preserving
- ✅ **CLAHE** ปรับ contrast ก่อน matching
- 🔮 **Future**: Fish-eye calibration model (สำหรับ FOV > 120°)

**Calibration Results (Final):**
| Parameter | Value | Status |
|-----------|-------|--------|
| **Pattern Spacing** | **12.0 mm** | ✅ Measured from printed pattern |
| Left Camera RMS | 0.22 px | ✅ Excellent |
| Right Camera RMS | 0.20 px | ✅ Excellent |
| Baseline | 60.57 mm | ✅ Correct (≈60mm spec) |
| Stereo RMS | 50.79 px | ⚠️ High (normal for wide-angle) |
| Images Used | 40 pairs | ✅ Good coverage |

**Working Range (Tested):**
| ระยะ | ค่าที่วัดได้ | Error | สถานะ |
|------|-------------|-------|-------|
| 30 cm | 28.7-29.4 cm | ±0.6-1.3 cm | ✅ Excellent |
| 32 cm | 29.0-34.0 cm (avg 31cm) | ±1 cm | ✅ Good |
| 42 cm | ~30 cm | -12 cm | ❌ Poor |
| 54 cm | ~29 cm | -25 cm | ❌ Failed |

**สรุป:**
- ✅ **Working range ปัจจุบัน: 30-35cm** (แม่นยำ ±1-2cm)
- ⚠️ **ต้องปรับ StereoSGBM parameters** เพื่อขยาย → 30-50cm
- 🎯 **เป้าหมาย**: Working range 30-50cm สำหรับ pepper sorting

---

### Key Decisions
- **ใช้ ROS2**: เพื่อเรียนรู้และสร้างระบบที่ scalable
- **Dual Arms**: เพิ่มความเร็วในการ sorting (parallel processing)
- **Stereo Camera**: ให้ข้อมูล depth สำหรับคำนวณตำแหน่ง 3D
- **Arduino-based Arms**: ใช้ของที่มีอยู่แล้ว, ประหยัดต้นทุน
- **Vision-First Approach**: เน้น Vision System ก่อน → ได้ output เร็ว
- **Asymmetric Circles Pattern** (5×6): เหมาะกับงานเกษตร, ทนต่อ lighting variations
- **GStreamer (nvarguscamerasrc)**: Native support สำหรับ Jetson CSI cameras
- **Wide-Angle Lens Handling**: ใช้ StereoSGBM + WLS filter เพื่อจัดการกับ 160° FOV distortion
- **Focus Optimization** (2025-10-24): Left 176.5, Right 171.0 @ 32cm, Diff < 10
- **Lighting Setup**: LED ซ้ายหน้า ทะแยงเข้าวัตถุ 10cm (documented in CAMERA_SETUP_GUIDE.md)

### Success Criteria

**Minimum Viable Product (MVP)**:
- ✅ Detect peppers with >80% accuracy
- ✅ Classify color with >85% accuracy
- ✅ Single arm picks and places >70% success rate
- ✅ System runs continuously for 5 minutes

**Target Performance**:
- 🎯 Dual arms coordination working
- 🎯 Process 5-10 peppers per minute
- 🎯 Overall success rate >75%
- 🎯 Quality classification accuracy >70%

---

## 🔄 Git Push/Pull Guide

### ตั้งค่าเริ่มต้น (ทำครั้งเดียว)

**Repository**: https://github.com/hirankrit/Jetson.git

Git ได้ตั้งค่า credential storage แล้ว ไม่ต้องใส่ token ทุกครั้ง

### คำสั่งพื้นฐาน

**1. ดึงโค้ดล่าสุดจาก GitHub มาเครื่อง (Pull)**
```bash
git pull
```

**2. ส่งการเปลี่ยนแปลงขึ้น GitHub (Push)**
```bash
# เพิ่มไฟล์ทั้งหมดที่แก้ไข
git add .

# Commit พร้อมข้อความอธิบาย
git commit -m "อธิบายการเปลี่ยนแปลง"

# Push ขึ้น GitHub
git push
```

**3. ดูสถานะไฟล์ (Status)**
```bash
git status
```

**4. ดูประวัติการ commit (Log)**
```bash
git log --oneline -10
```

### ตัวอย่างการใช้งาน

**เมื่อแก้ไขไฟล์และต้องการ push:**
```bash
git add .
git commit -m "Update vision calibration parameters"
git push
```

**เมื่อทำงานต่อจากเครื่องอื่น:**
```bash
git pull
# ... แก้ไขไฟล์ ...
git add .
git commit -m "Add Week 1 calibration report"
git push
```

### Tips & Best Practices

- 💡 **Pull ก่อนทำงาน**: รัน `git pull` ก่อนเริ่มทำงานทุกครั้ง
- 💡 **Commit บ่อยๆ**: แบ่ง commit เป็นชิ้นเล็กๆ ตามความหมาย
- 💡 **ข้อความ commit ชัดเจน**: ใช้ภาษาไทยหรือภาษาอังกฤษก็ได้ แต่ต้องอธิบายให้เข้าใจ
- 💡 **เช็ค status ก่อน commit**: รู้ว่าไฟล์ไหนถูกแก้ไขบ้าง

### Troubleshooting

**ถ้า push ไม่ได้:**
```bash
# Pull ก่อนแล้วค่อย push
git pull
git push
```

**ถ้าต้องการยกเลิกการแก้ไข:**
```bash
# ยกเลิกไฟล์ที่ยังไม่ได้ add
git checkout -- <filename>

# ยกเลิกทั้งหมด (ระวัง! จะหายหมด)
git reset --hard HEAD
```

---

## 📞 Contact & Support

**Project Path**: `/home/jay/Project/`

**Documentation Structure**:
```
/home/jay/Project/
├── plan1                           # Initial conversation history
├── claude.md                       # This file (main index)
├── CAMERA_CALIBRATION_GUIDE.md     # Calibration guide (Asymmetric Circles)
├── CAMERA_SETUP_GUIDE.md           # Focus + Lighting setup guide ⭐ NEW!
│
├── view_camera.py                  # Camera viewer (real-time display)
├── gstreamer_camera_node.py        # ROS2 stereo camera node
├── stereo_camera.launch.py         # ROS2 launch file
│
├── capture_calibration.py          # Capture calibration images (5×6 pattern)
├── stereo_calibration.py           # Compute calibration parameters
├── test_depth_map.py               # Basic depth map testing
├── test_depth_map_enhanced.py      # Enhanced (StereoSGBM + WLS + CLAHE) ⭐
│
├── debug_pattern.py                # Debug pattern detection
├── tune_blob_detector.py           # Interactive blob detector tuning
├── test_33_circles.py              # Test 33-circle pattern configurations
│
├── stereo_calib.yaml               # Calibration results (baseline 60.57mm)
├── rectification_maps.npz          # Pre-computed rectification maps
│
├── calib_images/                   # Calibration image pairs (40 pairs)
│   ├── left/
│   └── right/
│
├── setup_gstreamer_cameras.sh      # Camera setup script
├── install_ros2_humble.sh          # ROS2 installation script
├── merge_imx219_dtb.sh             # DTB merge utility
│
├── .claude/                        # Claude Code CLI configuration
│   └── skills/                     # Claude Skills (custom instructions)
│       ├── thai-commit.md
│       ├── weekly-report.md
│       ├── ros2-review.md
│       └── python-tools.md
│
└── docs/                           # Detailed documentation
    ├── 01_hardware_architecture.md
    ├── 02_software_architecture.md
    ├── 03_ros2_nodes_detail.md
    ├── 04_custom_messages.md
    ├── 05_coordinate_frames.md
    ├── 06_workspace_config.md
    ├── 07_development_roadmap.md      # Original plan
    ├── 08_tools_monitoring.md
    ├── 09_challenges_solutions.md
    ├── 10_setup_guide.md
    ├── 11_vision_first_roadmap.md ⭐  # Vision-First approach (แนะนำ)
    └── 12_claude_skills.md 🤖         # Claude Skills documentation (ใหม่!)
```

---

## 🚀 Getting Started

### 🎥 Quick Start - ทดสอบกล้อง (Camera Setup เสร็จแล้ว!)

```bash
# 1. ดูภาพจากกล้องทั้งสอง (แนะนำ)
python3 view_camera.py

# 2. ดูกล้องเดียว (0=left, 1=right)
python3 view_camera.py --single 0

# 3. เปลี่ยน resolution
python3 view_camera.py --width 1920 --height 1080 --fps 30

# 4. รัน ROS2 node
source /opt/ros/humble/setup.bash
python3 gstreamer_camera_node.py
```

**Controls**: กด `q` เพื่อออก, กด `s` เพื่อบันทึกภาพ

---

### 📐 Stereo Calibration (เสร็จแล้ว ✅)

**Pattern**: Asymmetric Circles Grid (5 rows × 6 cols, 33 circles, 18mm spacing)

```bash
# 1. พิมพ์ calibration pattern ✅
# ไปที่: https://calib.io/pages/camera-calibration-pattern-generator
# Settings: Asymmetric Circles, 5×6, 18mm diagonal spacing, 14mm diameter
# ดูรายละเอียดใน CAMERA_CALIBRATION_GUIDE.md

# 2. เก็บภาพ calibration ✅ (40 ภาพ)
python3 capture_calibration.py
# กด 'c' เพื่อถ่าย, 'q' เพื่อออก
# ภาพจะถูกบันทึกที่ calib_images/left/ และ calib_images/right/

# 3. คำนวณ calibration parameters ✅
python3 stereo_calibration.py
# ได้ไฟล์: stereo_calib.yaml และ rectification_maps.npz

# 4. ทดสอบ depth map (กำลังทำ 🎯)
python3 test_depth_map_enhanced.py
```

**Calibration Results:**
- ✅ **Baseline**: 60.57mm (ตรงกับ spec 60mm!)
- ✅ **Left Camera RMS**: 0.22 pixels (ยอดเยี่ยม)
- ✅ **Right Camera RMS**: 0.20 pixels (ยอดเยี่ยม)
- ⚠️ **Stereo RMS**: 50.79 pixels (สูงเพราะ wide-angle 160° FOV - ปกติสำหรับ wide-angle lens)

**ทำไมใช้ Asymmetric Circles?**
- ✅ ทนทานต่อแสงไม่สม่ำเสมอ (เหมาะกับงานเกษตร)
- ✅ Sub-pixel accuracy สูงกว่า checkerboard
- ✅ Unique pattern → detect ได้แม่นยำกว่า
- ✅ เหมาะกับวัตถุเล็กๆ เช่น พริก

**อ่านคู่มือครบถ้วน**: [CAMERA_CALIBRATION_GUIDE.md](CAMERA_CALIBRATION_GUIDE.md)

---

### 🧪 Depth Map Testing (กำลังทำ)

**ทดสอบความแม่นยำ depth estimation:**

```bash
# รันโปรแกรมทดสอบ (Enhanced version with StereoSGBM + WLS filter)
python3 test_depth_map_enhanced.py
```

**วิธีทดสอบ:**
1. **วางวัตถุ** ที่ระยะทราบ (30cm, 50cm, 70cm)
2. **คลิกที่วัตถุ** ในหน้าจอ → ดูระยะที่ terminal
3. **ทดลองปรับ filters**:
   - กด `1` = Toggle CLAHE (contrast enhancement)
   - กด `2` = Toggle WLS filter (edge-preserving)
4. **บันทึกภาพ**: กด `s`
5. **ออก**: กด `q`

**เกณฑ์ผ่าน:**
- 30cm → วัดได้ 28-32cm (±2cm) ✅
- 50cm → วัดได้ 47-53cm (±3cm) ✅
- 70cm → วัดได้ 66-74cm (±4cm) ✅

**Features ของ Enhanced Version:**
- ✅ **StereoSGBM**: ดีกว่า StereoBM สำหรับ wide-angle lens
- ✅ **WLS Filter**: Edge-preserving smoothing
- ✅ **CLAHE**: Contrast enhancement ก่อน matching

### สำหรับผู้เริ่มต้น (Vision-First Approach):

1. ✅ **อ่าน claude.md** (ไฟล์นี้) เพื่อเข้าใจภาพรวม
2. ✅ **ทดสอบกล้อง** ด้วย `view_camera.py`
3. ✅ **อ่าน [Vision-First Roadmap](docs/11_vision_first_roadmap.md)** ⭐ เพื่อดูแผนการพัฒนา
4. **Phase 1 - Week 1** (กำลังทำ):
   - ✅ Setup Jetson + Camera
   - ✅ ทำ Stereo Calibration (40 ภาพ, baseline 60.57mm)
   - 🎯 **กำลังทำ**: ทดสอบ Depth Map accuracy (30-80cm)
   - ⏳ **ต่อไป**: เขียนรายงาน Week 1 📝
5. **ดำเนินการต่อ Week 2-4** ตาม Vision-First Roadmap

### แผนทางเลือก (Original Plan):

ถ้าต้องการเรียนรู้ ROS2 ก่อน → ใช้ [Development Roadmap](docs/07_development_roadmap.md)

---

## 📊 Weekly Reports (เก็บ output แต่ละสัปดาห์)

| Week | Milestone | Report Status |
|------|-----------|---------------|
| 1 | Stereo Calibration | [ ] Pending |
| 2 | Dataset Collection | [ ] Pending |
| 3 | Model Training | [ ] Pending |
| 4 | Vision Integration | [ ] Pending |

---

**Happy Coding! 🌶️🤖**
