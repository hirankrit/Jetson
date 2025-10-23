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

**Last Updated**: 2025-10-23

### Development Approach
**เลือกใช้**: [Vision-First Roadmap](docs/11_vision_first_roadmap.md) ⭐

### Project Phase (Vision-First)
- [x] ออกแบบ overall architecture เสร็จสมบูรณ์
- [x] ตั้งค่า Git repository และ GitHub
- [x] สร้าง Claude Skills สำหรับ development workflow
- [ ] **Phase 1: Vision System (Week 1-4)** ← กำลังทำ
  - [x] Week 1: Stereo camera setup
    - [x] ตรวจสอบ hardware (Jetson Orin Nano + IMX219 Stereo)
    - [x] สร้าง enable script สำหรับ dual camera
    - [x] สร้าง test script สำหรับ stereo camera
    - [ ] รัน setup script และ reboot
    - [ ] ทดสอบ capture ภาพจาก 2 กล้อง
  - [ ] Week 1 (ต่อ): Stereo calibration + depth map
  - [ ] Week 2: Dataset collection (500-1000 images)
  - [ ] Week 3: YOLO training + evaluation
  - [ ] Week 4: Integration (detection + 3D positioning)
- [ ] Phase 2: ROS2 Integration (Week 5-6)
- [ ] Phase 3-5: Robot Arms + Full System (Week 7-12)

### Next Actions (กำลังทำ)
1. ✅ Setup development environment
2. ✅ ตรวจสอบ Jetson hardware และ JetPack version (R36.4.4)
3. ✅ สร้าง enable_imx219_stereo.sh script
4. ✅ สร้าง test_stereo_camera.py script
5. ⏳ รัน enable script และ reboot Jetson
6. ⏳ ทดสอบ stereo camera capture
7. 🎯 Stereo calibration (collect 30+ checkerboard images)
8. 📊 Generate depth map และรายงานผล calibration
9. 📝 **Output**: รายงาน Week 1 (calibration quality + depth accuracy)

---

## 🔗 Quick References

### Hardware
- **Camera**: [IMX219-83 Stereo Camera Wiki](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)
- **Robot Arm**: Mini Brazo robótico con Arduino (YouTube reference)
- **Jetson**: [Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)

### Software
- **ROS2**: [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- **YOLO**: [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- **OpenCV**: [OpenCV Stereo Calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)

### Learning Resources
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [TensorRT for Jetson](https://developer.nvidia.com/tensorrt)
- [Arduino Serial Communication](https://www.arduino.cc/reference/en/language/functions/communication/serial/)

---

## 📝 Development Notes

### Key Decisions
- **ใช้ ROS2**: เพื่อเรียนรู้และสร้างระบบที่ scalable
- **Dual Arms**: เพิ่มความเร็วในการ sorting (parallel processing)
- **Stereo Camera**: ให้ข้อมูล depth สำหรับคำนวณตำแหน่ง 3D
- **Arduino-based Arms**: ใช้ของที่มีอยู่แล้ว, ประหยัดต้นทุน

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
├── plan1                    # Initial conversation history
├── claude.md               # This file (main index)
├── .claude/                # Claude Code CLI configuration
│   └── skills/             # Claude Skills (custom instructions)
│       ├── thai-commit.md
│       ├── weekly-report.md
│       ├── ros2-review.md
│       └── python-tools.md
└── docs/                   # Detailed documentation
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

### สำหรับผู้เริ่มต้น (Vision-First Approach):

1. **อ่าน claude.md** (ไฟล์นี้) เพื่อเข้าใจภาพรวม ✓
2. **อ่าน [Vision-First Roadmap](docs/11_vision_first_roadmap.md)** ⭐ เพื่อดูแผนการพัฒนา
3. **เริ่ม Phase 1 - Week 1**:
   - Setup Jetson + Camera (ดู [Setup Guide](docs/10_setup_guide.md))
   - ทำ Stereo Calibration (ดู [Coordinate Frames](docs/05_coordinate_frames.md))
   - ทดสอบ Depth Map
   - **เขียนรายงาน Week 1** 📝
4. **ดำเนินการต่อ Week 2-4** ตาม Vision-First Roadmap

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
