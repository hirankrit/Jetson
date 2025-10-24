# 10. Setup Guide

Complete installation and setup instructions for the Pepper Sorting Robot system.

---

## Prerequisites

### Hardware Required
- Jetson Orin Nano Developer Kit
- microSD card (64GB or larger, UHS-1 recommended)
- IMX219-83 Stereo Camera
- 2× Arduino Mega 2560 (or clones)
- 2× Robot arm kits (servos, structure)
- Power supplies (5V/4A for Jetson, 5V/3A for each Arduino)
- USB cables, mounting hardware

### Software Requirements
- Host computer (for initial setup)
- Internet connection
- Patience (setup takes 2-4 hours)

---

## Part 1: Jetson Orin Nano Setup

### Step 1: Flash Jetson with Ubuntu 22.04

**Option A: Using SDK Manager (Recommended)**

1. Download NVIDIA SDK Manager on host PC: https://developer.nvidia.com/sdk-manager
2. Connect Jetson via USB-C (recovery mode)
3. Flash JetPack 5.x with Ubuntu 22.04
4. Follow on-screen instructions

**Option B: Using Etcher (Simpler)**

1. Download JetPack image: https://developer.nvidia.com/embedded/downloads
2. Flash to microSD using Balena Etcher
3. Insert microSD into Jetson
4. Power on and follow setup wizard

---

### Step 2: Initial Jetson Configuration

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y build-essential cmake git curl wget nano

# Set Jetson to max performance
sudo nvpmodel -m 0  # Max performance mode
sudo jetson_clocks   # Max clock speeds

# Install jtop (monitoring tool)
sudo pip3 install -U jetson-stats
# Reboot required after jtop install
sudo reboot
```

---

### Step 3: Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-argcomplete python3-colcon-common-extensions

# Setup environment (add to ~/.bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Should output: ros2 cli version X.X.X
```

---

### Step 4: Install ROS2 Packages

```bash
# Install additional ROS2 packages
sudo apt install -y \
  ros-humble-vision-opencv \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  ros-humble-rviz2

# Install Python dependencies
pip3 install --upgrade pip
pip3 install \
  opencv-python \
  numpy \
  scipy \
  pyserial \
  transforms3d \
  ultralytics \
  torch torchvision
```

---

## Part 2: Camera Setup

### Step 5: Install Camera Drivers

**For IMX219 Stereo Camera on Jetson Orin Nano:**

```bash
# 1. Install GStreamer packages (if not already installed)
sudo apt install -y nvidia-l4t-gstreamer

# 2. Enable IMX219 stereo camera in device tree
# Use the setup script (see project root)
./setup_gstreamer_cameras.sh

# 3. Reboot to apply changes
sudo reboot

# 4. After reboot, verify camera devices
ls -la /dev/video*
# Should show: /dev/video0 and /dev/video1

# 5. Test left camera (sensor-id=0)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! 'video/x-raw,format=BGRx' ! videoconvert ! xvimagesink

# 6. Test right camera (sensor-id=1)
gst-launch-1.0 nvarguscamerasrc sensor-id=1 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
  nvvidconv ! 'video/x-raw,format=BGRx' ! videoconvert ! xvimagesink

# If cameras work, you should see live video
# Press Ctrl+C to stop
```

**Quick Test with Python Viewer:**

```bash
# View both cameras side-by-side
python3 view_camera.py

# View single camera
python3 view_camera.py --single 0  # Left camera
python3 view_camera.py --single 1  # Right camera

# Controls: 'q' to quit, 's' to save snapshot
```

### Step 6: Camera Calibration

```bash
# Create calibration directory
mkdir -p ~/pepper_sorting_ws/src/pepper_vision/calibration
cd ~/pepper_sorting_ws/src/pepper_vision/calibration

# Print checkerboard pattern (9x6, 25mm squares)
# https://github.com/opencv/opencv/blob/4.x/doc/pattern.png

# Run calibration script (see docs/05_coordinate_frames.md for script)
python3 stereo_calibration.py

# Move checkerboard around, press 'c' to capture image
# Capture 20-30 images from various angles
# Press 'q' when done

# Calibration will output: stereo_calib.yaml
```

---

## Part 3: Arduino Setup

### Step 7: Arduino IDE Installation

```bash
# Option 1: Install Arduino IDE on Jetson
sudo apt install arduino

# Option 2: Install on host PC (recommended)
# Download from: https://www.arduino.cc/en/software
```

### Step 8: Upload Firmware

```cpp
// arm_controller.ino (see docs/03_ros2_nodes_detail.md for full code)

#include <Servo.h>

Servo servos[6];  // 5 joints + 1 gripper
int servoPins[] = {3, 5, 6, 9, 10, 11};
int currentAngles[6] = {90, 90, 90, 90, 90, 45};

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(currentAngles[i]);
  }
  Serial.println("OK");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}

void handleCommand(String cmd) {
  if (cmd.startsWith("MOVE,")) {
    // Parse and move servos
    Serial.println("OK");
  } else if (cmd == "HOME") {
    // Go home
    Serial.println("OK");
  }
}
```

**Upload steps:**
1. Open Arduino IDE
2. Paste code above
3. Select board: Tools → Board → Arduino Mega 2560
4. Select port: Tools → Port → /dev/ttyUSB0 (or ttyACM0)
5. Click Upload
6. Repeat for second Arduino (port /dev/ttyUSB1)

### Step 9: Test Serial Communication

```bash
# Install screen for serial testing
sudo apt install screen

# Connect to Arduino
screen /dev/ttyUSB0 115200

# Type commands:
# MOVE,90,90,90,90,90,45
# HOME
# STATUS

# Should see "OK" responses
# Press Ctrl+A, then K to exit screen
```

---

## Part 4: ROS2 Workspace Setup

### Step 10: Create Workspace

```bash
# Create workspace
mkdir -p ~/pepper_sorting_ws/src
cd ~/pepper_sorting_ws/src

# Create packages
ros2 pkg create pepper_msgs --build-type ament_cmake
ros2 pkg create pepper_vision --build-type ament_python --dependencies rclpy sensor_msgs cv_bridge
ros2 pkg create pepper_control --build-type ament_python --dependencies rclpy geometry_msgs
ros2 pkg create pepper_planning --build-type ament_python --dependencies rclpy
ros2 pkg create pepper_bringup --build-type ament_python

# Create directory structure
cd ~/pepper_sorting_ws/src/pepper_msgs
mkdir msg

cd ~/pepper_sorting_ws/src/pepper_vision
mkdir -p pepper_vision/calibration launch config models

cd ~/pepper_sorting_ws/src/pepper_control
mkdir -p pepper_control/calibration launch config arduino

cd ~/pepper_sorting_ws/src/pepper_planning
mkdir -p pepper_planning launch config

cd ~/pepper_sorting_ws/src/pepper_bringup
mkdir -p launch config
```

### Step 11: Define Custom Messages

Create message files (see `docs/04_custom_messages.md`):

```bash
cd ~/pepper_sorting_ws/src/pepper_msgs/msg

# Create DetectedPepper.msg
cat > DetectedPepper.msg << 'EOF'
std_msgs/Header header
geometry_msgs/Point position
string color
string quality
float32 size_mm
float32 confidence
int32 id
bool is_reachable
EOF

# Create other messages (PepperArray.msg, ArmCommand.msg, ArmStatus.msg)
# ... (see docs/04_custom_messages.md)
```

Update `CMakeLists.txt` and `package.xml` (see docs/04_custom_messages.md).

### Step 12: Build Workspace

```bash
cd ~/pepper_sorting_ws

# Build messages first
colcon build --packages-select pepper_msgs

# Source workspace
source install/setup.bash

# Verify messages
ros2 interface list | grep pepper_msgs
# Should show:
# pepper_msgs/msg/ArmCommand
# pepper_msgs/msg/ArmStatus
# pepper_msgs/msg/DetectedPepper
# pepper_msgs/msg/PepperArray

# Build rest of packages (will add code later)
colcon build

# Add to ~/.bashrc for convenience
echo "source ~/pepper_sorting_ws/install/setup.bash" >> ~/.bashrc
```

---

## Part 5: System Configuration

### Step 13: Configure Workspace Parameters

```bash
cd ~/pepper_sorting_ws/src/pepper_bringup/config

# Create workspace_config.yaml (see docs/06_workspace_config.md)
nano workspace_config.yaml
# Paste configuration...

# Create vision_params.yaml
nano vision_params.yaml
# Paste configuration...

# Create arm_left_params.yaml
nano arm_left_params.yaml
# Paste configuration...

# Create arm_right_params.yaml
nano arm_right_params.yaml
# Paste configuration...
```

### Step 14: Copy Calibration Files

```bash
# Copy stereo calibration
cp ~/pepper_sorting_ws/src/pepper_vision/calibration/stereo_calib.yaml \
   ~/pepper_sorting_ws/src/pepper_bringup/config/

# Hand-eye calibration will be done later during development
```

---

## Part 6: Train YOLO Model

### Step 15: Dataset Preparation

```bash
# Create dataset directory
mkdir -p ~/pepper_dataset
cd ~/pepper_dataset

# Collect images (using camera_node or standalone script)
# Aim for 500-1000 images

# Label using Roboflow (https://roboflow.com/)
# Or use LabelImg:
pip3 install labelImg
labelImg

# Export in YOLO format:
# dataset/
#   ├── images/train/
#   ├── images/val/
#   ├── labels/train/
#   ├── labels/val/
#   └── data.yaml
```

### Step 16: Train Model

```bash
cd ~/pepper_dataset

# data.yaml example:
cat > data.yaml << 'EOF'
train: ./images/train
val: ./images/val

nc: 6  # Number of classes
names: ['pepper_red_fresh', 'pepper_red_rotten',
        'pepper_green_fresh', 'pepper_green_rotten',
        'pepper_yellow_fresh', 'pepper_yellow_rotten']
EOF

# Train YOLOv8 segmentation model
yolo train data=data.yaml model=yolov8n-seg.pt epochs=100 imgsz=640 batch=16

# Training will take 1-3 hours on Jetson
# Output: runs/segment/train/weights/best.pt

# Copy model to workspace
cp runs/segment/train/weights/best.pt \
   ~/pepper_sorting_ws/src/pepper_vision/models/yolo_pepper_v8.pt
```

---

## Part 7: Verification & Testing

### Step 17: Test Individual Components

```bash
# 1. Test ROS2 installation
ros2 topic list

# 2. Test camera
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! xvimagesink

# 3. Test Arduino
screen /dev/ttyUSB0 115200
# Type: HOME
# Should see: OK

# 4. Test YOLO model
python3 -c "
from ultralytics import YOLO
model = YOLO('~/pepper_sorting_ws/src/pepper_vision/models/yolo_pepper_v8.pt')
print('Model loaded successfully')
"
```

### Step 18: System Health Check

```bash
# Create health check script
cat > ~/check_system.sh << 'EOF'
#!/bin/bash
echo "=== Pepper Sorting Robot - System Check ==="
echo ""

echo "1. ROS2 Installation:"
ros2 --version && echo "✓ ROS2 OK" || echo "✗ ROS2 FAIL"

echo ""
echo "2. Camera (CSI):"
ls /dev/video* && echo "✓ Camera detected" || echo "✗ No camera"

echo ""
echo "3. Arduino (USB):"
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null && echo "✓ Arduino detected" || echo "✗ No Arduino"

echo ""
echo "4. Workspace:"
source ~/pepper_sorting_ws/install/setup.bash
ros2 interface list | grep pepper_msgs > /dev/null && echo "✓ Workspace built" || echo "✗ Workspace not built"

echo ""
echo "5. YOLO Model:"
test -f ~/pepper_sorting_ws/src/pepper_vision/models/yolo_pepper_v8.pt && echo "✓ Model exists" || echo "✗ Model missing"

echo ""
echo "=== System Check Complete ==="
EOF

chmod +x ~/check_system.sh
./check_system.sh
```

---

## Part 8: Launch System

### Step 19: Launch Nodes Individually (for testing)

```bash
# Terminal 1: Camera
source ~/pepper_sorting_ws/install/setup.bash
ros2 run pepper_vision camera_node

# Terminal 2: Vision
ros2 run pepper_vision vision_node

# Terminal 3: Left Arm
ros2 run pepper_control arm_controller_node --ros-args -p arm_id:=left

# Terminal 4: Right Arm
ros2 run pepper_control arm_controller_node --ros-args -p arm_id:=right

# Terminal 5: Task Planner
ros2 run pepper_planning task_planner_node
```

### Step 20: Launch Full System

```bash
# Once individual testing works, use launch file
ros2 launch pepper_bringup system.launch.py

# With RViz visualization
ros2 launch pepper_bringup system.launch.py use_rviz:=true
```

---

## Troubleshooting Common Setup Issues

### Issue: ROS2 command not found
**Solution:** Source the setup file
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Camera not detected
**Solution:** Check CSI cable connection, run `ls /dev/video*`

### Issue: Arduino not detected
**Solution:**
```bash
# Check USB connection
ls /dev/ttyUSB* /dev/ttyACM*

# Add user to dialout group
sudo usermod -aG dialout $USER
# Logout and login
```

### Issue: Colcon build fails
**Solution:**
```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Issue: Import error for custom messages
**Solution:**
```bash
# Rebuild and source workspace
colcon build --packages-select pepper_msgs
source install/setup.bash
```

---

## Next Steps

After successful setup:

1. **Calibration**: Perform hand-eye calibration (see `docs/05_coordinate_frames.md`)
2. **Development**: Start Phase 1 of development roadmap (see `docs/07_development_roadmap.md`)
3. **Testing**: Test individual components before full integration
4. **Monitoring**: Use RViz and rqt tools to monitor system (see `docs/08_tools_monitoring.md`)

---

## Useful Commands Reference

```bash
# ROS2 basics
ros2 node list
ros2 topic list
ros2 topic echo /detected_peppers
ros2 topic hz /camera/left/image_raw

# Build workspace
cd ~/pepper_sorting_ws
colcon build
source install/setup.bash

# Launch system
ros2 launch pepper_bringup system.launch.py

# Monitor performance
sudo jtop  # Jetson monitoring

# View logs
ros2 run rqt_console rqt_console

# Visualize
ros2 run rviz2 rviz2
```

---

**Setup Guide Complete ✓**

**🎉 Congratulations! Your system is now set up and ready for development.**

Refer to `docs/07_development_roadmap.md` to begin Phase 1: ROS2 Foundation.
