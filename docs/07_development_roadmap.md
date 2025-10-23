# 7. Development Roadmap (12 Weeks)

## Timeline Overview

| Phase | Weeks | Focus | Deliverable |
|-------|-------|-------|-------------|
| **Phase 1** | 1-2 | ROS2 Foundation | Working ROS2 workspace with demo nodes |
| **Phase 2** | 3-5 | Vision System | Detection + 3D positioning working |
| **Phase 3** | 6-7 | Single Arm Control | One arm picking peppers successfully |
| **Phase 4** | 8-9 | Dual Arm Coordination | Both arms working together |
| **Phase 5** | 10-11 | System Integration | Full system operational |
| **Phase 6** | 12 | Documentation & Polish | Production-ready system |

---

## Phase 1: ROS2 Foundation (Week 1-2)

### Week 1: Installation & Basic Concepts

**Goals:**
- Install ROS2 Humble on Jetson Orin Nano
- Understand ROS2 core concepts
- Create first publisher/subscriber

**Tasks:**

**Day 1-2: ROS2 Installation**
- [ ] Flash Jetson with Ubuntu 22.04
- [ ] Install ROS2 Humble Desktop
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete
```
- [ ] Setup environment: `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`
- [ ] Install colcon: `sudo apt install python3-colcon-common-extensions`

**Day 3-4: ROS2 Tutorials**
- [ ] Complete [CLI tools tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
  - Understanding nodes, topics, services
  - `ros2 topic list`, `ros2 topic echo`, etc.
- [ ] Complete [Python publisher/subscriber tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

**Day 5-7: Practice Project**
- [ ] Create workspace: `~/pepper_sorting_ws`
- [ ] Create package: `pepper_test`
- [ ] Implement: Temperature sensor simulator (publisher) + monitor (subscriber)
- [ ] Test: `ros2 run pepper_test sensor_node`

**Resources:**
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials for Beginners](https://www.youtube.com/watch?v=0aPbWsyENA8) (YouTube)

---

### Week 2: Intermediate ROS2

**Goals:**
- Learn launch files, parameters, TF2
- Understand message definitions
- Build multi-node system

**Tasks:**

**Day 8-10: Launch Files & Parameters**
- [ ] [Launch files tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ ] [Parameters tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [ ] Practice: Create launch file with multiple nodes and YAML config

**Day 11-12: Custom Messages**
- [ ] [Creating custom messages tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ ] Create `pepper_msgs` package
- [ ] Define all 4 custom messages (DetectedPepper, etc.)
- [ ] Build and test: `ros2 interface show pepper_msgs/msg/DetectedPepper`

**Day 13-14: TF2 Basics**
- [ ] [TF2 tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [ ] Understand coordinate frames
- [ ] Practice: Broadcast static transforms

**Deliverable:** Working ROS2 workspace with custom messages, launch files, and basic TF2 understanding

---

## Phase 2: Vision System (Week 3-5)

### Week 3: Camera Setup & Calibration

**Goals:**
- Get stereo camera working
- Perform calibration
- Generate depth maps

**Tasks:**

**Day 15-16: Camera Driver Installation**
- [ ] Install camera dependencies:
```bash
sudo apt install python3-opencv
pip3 install opencv-python numpy
```
- [ ] Test IMX219 camera on Jetson (GStreamer pipeline)
- [ ] Capture test images from both cameras

**Day 17-19: Stereo Calibration**
- [ ] Print checkerboard pattern (9x6, 25mm squares)
- [ ] Collect 20-30 calibration images
- [ ] Run OpenCV stereo calibration script
- [ ] Verify calibration quality (reprojection error < 0.5px)
- [ ] Save `stereo_calib.yaml`

**Day 20-21: Camera Node Implementation**
- [ ] Create `pepper_vision` package
- [ ] Implement `camera_node.py`
- [ ] Publish `/camera/left/image_raw` and `/camera/right/image_raw`
- [ ] Test: `ros2 topic hz /camera/left/image_raw` (should be ~30Hz)

**Resources:**
- [OpenCV Stereo Calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
- GStreamer for Jetson IMX219

---

### Week 4: Data Collection & Model Training

**Goals:**
- Collect pepper dataset
- Train YOLO model
- Achieve >80% detection accuracy

**Tasks:**

**Day 22-24: Dataset Collection**
- [ ] Setup lighting and camera
- [ ] Collect 500-1000 images of peppers:
  - Various colors (red, green, yellow)
  - Various qualities (fresh, rotten)
  - Various arrangements (single, pile)
  - Different lighting conditions
- [ ] Label dataset using [Roboflow](https://roboflow.com/) or LabelImg

**Day 25-27: Model Training**
- [ ] Install Ultralytics YOLO:
```bash
pip3 install ultralytics
```
- [ ] Prepare dataset in YOLO format:
```
dataset/
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── data.yaml
```
- [ ] Train YOLOv8 segmentation model:
```bash
yolo train data=peppers.yaml model=yolov8n-seg.pt epochs=100 imgsz=640
```
- [ ] Evaluate model performance
- [ ] Export to ONNX for Jetson optimization (optional TensorRT)

**Day 28: Testing**
- [ ] Test model on new images
- [ ] Verify detection confidence >80%
- [ ] Verify color classification accuracy >85%

---

### Week 5: Vision Node Integration

**Goals:**
- Integrate YOLO detection with depth
- Publish detected peppers with 3D positions
- Visualize in RViz2

**Tasks:**

**Day 29-31: Vision Node Implementation**
- [ ] Implement `vision_node.py`
- [ ] Integrate YOLO model inference
- [ ] Compute stereo disparity map (OpenCV SGBM)
- [ ] Convert disparity to depth
- [ ] Calculate 3D positions for each pepper
- [ ] Classify color and quality
- [ ] Publish `PepperArray` message

**Day 32-34: Testing & Tuning**
- [ ] Test with real peppers
- [ ] Tune SGBM parameters for good depth quality
- [ ] Verify 3D position accuracy (±5mm)
- [ ] Optimize performance (aim for 10Hz processing)

**Day 35: Visualization**
- [ ] Create RViz2 config
- [ ] Visualize detected peppers as markers
- [ ] Visualize depth map (optional)
- [ ] Test end-to-end: Camera → Vision → RViz

**Deliverable:** Vision system publishing accurate 3D positions of peppers

---

## Phase 3: Single Arm Control (Week 6-7)

### Week 6: Kinematics & Arduino Interface

**Goals:**
- Implement inverse kinematics
- Establish Arduino communication
- Move arm to target positions

**Tasks:**

**Day 36-38: Kinematics**
- [ ] Measure robot arm dimensions (link lengths)
- [ ] Implement forward kinematics (joint angles → end effector position)
- [ ] Implement inverse kinematics solver:
  - Try analytical solution first (if geometry is simple)
  - Fallback to numerical solver (scipy.optimize)
- [ ] Test IK: given XYZ, compute joint angles

**Day 39-41: Arduino Firmware**
- [ ] Design serial protocol (MOVE, HOME, STOP, STATUS)
- [ ] Write Arduino sketch: `arm_controller.ino`
- [ ] Test servo control independently
- [ ] Test serial communication: Python → Arduino

**Day 42: Integration Test**
- [ ] Send XYZ command from Python
- [ ] Compute IK → send serial command → verify arm moves correctly
- [ ] Test full range of workspace

---

### Week 7: Arm Controller Node & Calibration

**Goals:**
- Create ROS2 arm controller node
- Perform hand-eye calibration
- Execute pick & place successfully

**Tasks:**

**Day 43-45: Arm Controller Node**
- [ ] Create `pepper_control` package
- [ ] Implement `arm_controller_node.py`
- [ ] Subscribe to `/arm/left/command`
- [ ] Publish `/arm/left/status`
- [ ] Implement pick & place sequence logic

**Day 46-48: Hand-Eye Calibration**
- [ ] Place calibration target (e.g., checkerboard corner) at 10-15 positions
- [ ] Record camera detection (XYZ from vision)
- [ ] Move arm to touch target, record arm position
- [ ] Run calibration script to compute transformation matrix
- [ ] Verify calibration accuracy (<5mm error)
- [ ] Save `hand_eye_calib_left.yaml`

**Day 49: Testing**
- [ ] Test pick & place with peppers
- [ ] Measure success rate (aim for >70%)
- [ ] Tune gripper timing and approach height
- [ ] Debug failures (IK failures, gripper misses, etc.)

**Deliverable:** Single arm successfully picking and placing peppers

---

## Phase 4: Dual Arm Coordination (Week 8-9)

### Week 8: Task Planner & Workspace Division

**Goals:**
- Implement task planner node
- Define workspace zones for left/right arms
- Assign tasks intelligently

**Tasks:**

**Day 50-52: Task Planner Node**
- [ ] Create `pepper_planning` package
- [ ] Implement `task_planner_node.py`
- [ ] Subscribe to `/detected_peppers` and arm status topics
- [ ] Implement workspace division logic (left/right by X coordinate)
- [ ] Implement task assignment algorithm (closest first)
- [ ] Publish commands to both arms

**Day 53-55: Coordination Logic**
- [ ] Implement task tracking (prevent double-picking)
- [ ] Add priority queue for pending peppers
- [ ] Handle arm busy/idle state transitions
- [ ] Test with simulated arms (print commands)

**Day 56: Integration**
- [ ] Test with one real arm + planner
- [ ] Verify task assignment works correctly

---

### Week 9: Dual Arm Testing

**Goals:**
- Get both arms working simultaneously
- Ensure no collisions
- Optimize throughput

**Tasks:**

**Day 57-58: Second Arm Setup**
- [ ] Calibrate right arm (hand-eye calibration)
- [ ] Launch second arm controller node
- [ ] Verify both arms respond to commands

**Day 59-61: Dual Arm Testing**
- [ ] Test both arms picking from different zones
- [ ] Add collision avoidance (simple: don't enter overlap zone)
- [ ] Test concurrent picking
- [ ] Measure throughput (peppers per minute)

**Day 62-63: Optimization**
- [ ] Tune task assignment for better load balancing
- [ ] Optimize motion speed (increase speed_scale if safe)
- [ ] Reduce idle time between tasks
- [ ] Aim for 5-10 peppers/minute throughput

**Deliverable:** Dual arms working in coordination, sorting peppers

---

## Phase 5: System Integration (Week 10-11)

### Week 10: Full System Testing

**Goals:**
- End-to-end system testing
- Identify and fix bugs
- Measure performance

**Tasks:**

**Day 64-65: Launch System**
- [ ] Create `pepper_bringup` package
- [ ] Write `system.launch.py` (launches all nodes)
- [ ] Test full pipeline: Camera → Vision → Planner → Arms
- [ ] Verify all topics are publishing correctly

**Day 66-69: Stress Testing**
- [ ] Test with 20+ peppers in pile
- [ ] Run continuously for 30+ minutes
- [ ] Log failures and errors
- [ ] Identify bottlenecks (vision FPS, arm speed, etc.)

**Day 70: Bug Fixing**
- [ ] Fix critical bugs discovered during testing
- [ ] Handle edge cases (IK failures, gripper misses)
- [ ] Improve error recovery

---

### Week 11: Optimization & Robustness

**Goals:**
- Improve success rate to >75%
- Add error recovery mechanisms
- Optimize performance

**Tasks:**

**Day 71-73: Error Handling**
- [ ] Add retry logic for gripper failures
- [ ] Handle serial disconnections (auto-reconnect)
- [ ] Handle vision failures (skip frame if detection fails)
- [ ] Add emergency stop functionality

**Day 74-76: Performance Tuning**
- [ ] Profile vision node (optimize YOLO/stereo computation)
- [ ] Reduce latency between detection and pick
- [ ] Optimize IK solver speed
- [ ] Consider TensorRT for YOLO (2-3× speedup)

**Day 77: Validation**
- [ ] Run 10 test sessions (10 peppers each)
- [ ] Measure:
  - Success rate (picks / attempts)
  - Throughput (peppers / minute)
  - Classification accuracy (correct bin / total)
- [ ] Document results

**Deliverable:** Robust system with >75% success rate

---

## Phase 6: Documentation & Finalization (Week 12)

### Week 12: Polish & Documentation

**Goals:**
- Clean up code
- Write comprehensive documentation
- Record demo video
- Prepare for deployment

**Tasks:**

**Day 78-80: Code Cleanup**
- [ ] Refactor code (remove debug prints, organize functions)
- [ ] Add docstrings to all functions/classes
- [ ] Follow Python style guide (PEP 8)
- [ ] Add type hints

**Day 81-82: Documentation**
- [ ] Write package READMEs
- [ ] Document API for each node
- [ ] Create troubleshooting guide
- [ ] Write user manual (how to operate the system)

**Day 83: Demo Video**
- [ ] Record system in action
- [ ] Show: setup, calibration, operation, results
- [ ] Edit video (add annotations, metrics)
- [ ] Upload to YouTube/document

**Day 84: Final Testing & Handoff**
- [ ] Final end-to-end test
- [ ] Generate performance report
- [ ] Package all code and documentation
- [ ] Create deployment checklist

**Deliverable:** Production-ready system with full documentation

---

## Success Criteria Summary

### Minimum Viable Product (MVP)
- [x] Detect peppers with >80% accuracy
- [x] Classify color with >85% accuracy
- [x] Single arm picks and places >70% success rate
- [x] System runs continuously for 5 minutes without crashing

### Target Performance
- [x] Dual arms coordination working
- [x] Process 5-10 peppers per minute
- [x] Overall success rate >75%
- [x] Quality classification accuracy >70%

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| ROS2 learning curve too steep | Allocate extra time in Phase 1, follow tutorials strictly |
| Stereo calibration poor quality | Use checkerboard, collect many images, verify reprojection error |
| YOLO model underfits | Collect more diverse data, try different YOLO sizes (nano→small) |
| IK solver fails often | Implement numerical solver fallback, tune workspace limits |
| Arms collide | Add safety margins, test workspace division carefully |
| Low throughput | Optimize vision FPS, increase arm speed, improve task assignment |

---

**Development Roadmap Complete ✓**
