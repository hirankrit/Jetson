# 2. Software Architecture

## ROS2 Architecture Overview

### System Design Philosophy

```
┌─────────────────────────────────────────────────────────┐
│                    ROS2 Middleware                      │
│  (Communication, Coordination, Data Flow)               │
└─────────────────────────────────────────────────────────┘
           │              │              │
           ▼              ▼              ▼
    ┌──────────┐   ┌──────────┐   ┌──────────┐
    │ Vision   │   │ Planning │   │ Control  │
    │ Layer    │   │ Layer    │   │ Layer    │
    └──────────┘   └──────────┘   └──────────┘
```

**Key Principles**:
- **Modularity**: Each component is independent ROS2 node
- **Loose Coupling**: Nodes communicate via topics/services
- **Scalability**: Easy to add more arms or sensors
- **Reusability**: Packages can be reused in other projects

---

## ROS2 Workspace Structure

```
~/pepper_sorting_ws/
├── src/
│   ├── pepper_msgs/              # Custom message definitions
│   │   ├── msg/
│   │   │   ├── DetectedPepper.msg
│   │   │   ├── PepperArray.msg
│   │   │   ├── ArmCommand.msg
│   │   │   └── ArmStatus.msg
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── pepper_vision/            # Vision processing package
│   │   ├── pepper_vision/
│   │   │   ├── __init__.py
│   │   │   ├── camera_node.py
│   │   │   ├── vision_node.py
│   │   │   ├── stereo_processor.py
│   │   │   ├── pepper_detector.py
│   │   │   └── calibration/
│   │   │       ├── camera_calibration.py
│   │   │       └── stereo_calib.yaml
│   │   ├── launch/
│   │   │   └── vision.launch.py
│   │   ├── config/
│   │   │   └── vision_params.yaml
│   │   ├── models/
│   │   │   └── yolo_pepper_v8.pt
│   │   ├── setup.py
│   │   ├── package.xml
│   │   └── README.md
│   │
│   ├── pepper_control/           # Arm control package
│   │   ├── pepper_control/
│   │   │   ├── __init__.py
│   │   │   ├── arm_controller_node.py
│   │   │   ├── inverse_kinematics.py
│   │   │   ├── serial_interface.py
│   │   │   ├── trajectory_planner.py
│   │   │   └── calibration/
│   │   │       ├── hand_eye_calib.py
│   │   │       └── arm_params.yaml
│   │   ├── launch/
│   │   │   ├── arm_left.launch.py
│   │   │   └── arm_right.launch.py
│   │   ├── config/
│   │   │   ├── arm_left_params.yaml
│   │   │   └── arm_right_params.yaml
│   │   ├── arduino/
│   │   │   └── arm_controller.ino
│   │   ├── setup.py
│   │   ├── package.xml
│   │   └── README.md
│   │
│   ├── pepper_planning/          # Task planning & coordination
│   │   ├── pepper_planning/
│   │   │   ├── __init__.py
│   │   │   ├── task_planner_node.py
│   │   │   ├── workspace_manager.py
│   │   │   ├── collision_checker.py
│   │   │   └── priority_queue.py
│   │   ├── launch/
│   │   │   └── planner.launch.py
│   │   ├── config/
│   │   │   └── workspace_config.yaml
│   │   ├── setup.py
│   │   ├── package.xml
│   │   └── README.md
│   │
│   └── pepper_bringup/           # System launch & config
│       ├── launch/
│       │   ├── system.launch.py         # Launch all nodes
│       │   ├── vision_only.launch.py    # Test vision
│       │   ├── arms_only.launch.py      # Test arms
│       │   └── simulation.launch.py     # Test without hardware
│       ├── config/
│       │   ├── system_params.yaml
│       │   └── rviz_config.rviz
│       ├── package.xml
│       └── README.md
│
├── build/                        # Build artifacts (auto-generated)
├── install/                      # Installed packages (auto-generated)
├── log/                          # Build logs (auto-generated)
└── README.md
```

---

## ROS2 Node Graph

```
                    ┌─────────────────────┐
                    │  /camera_node       │
                    │  (stereo_capture)   │
                    └──────┬──────────────┘
                           │ publishes
                           ├─→ /camera/left/image_raw
                           ├─→ /camera/right/image_raw
                           └─→ /camera/camera_info
                                    │
                                    │ subscribes
                                    ▼
                    ┌─────────────────────────────┐
                    │  /vision_node               │
                    │  (detection + depth)        │
                    └──────┬──────────────────────┘
                           │ publishes
                           └─→ /detected_peppers
                                    │
                                    │ subscribes
                                    ▼
                    ┌─────────────────────────────┐
                    │  /task_planner_node         │
                    │  (coordination logic)       │
                    └──────┬──────────┬───────────┘
                           │          │
                 publishes │          │ publishes
                           ▼          ▼
            ┌──────────────────┐  ┌──────────────────┐
            │ /arm_left_node   │  │ /arm_right_node  │
            │ (IK + Serial)    │  │ (IK + Serial)    │
            └────┬─────────────┘  └─────────────┬────┘
                 │ subscribes           subscribes │
                 │ /arm/left/command    /arm/right/command
                 │                                  │
                 │ publishes                 publishes
                 │ /arm/left/status         /arm/right/status
                 └──────────┬─────────────────┬────┘
                            │                 │
                            │ subscribes      │ subscribes
                            └────────┬────────┘
                                     ▼
                            /task_planner_node
                            (monitors status)
```

---

## Topic & Message Flow

### Published Topics

| Topic | Message Type | Publisher | Rate | Description |
|-------|-------------|-----------|------|-------------|
| `/camera/left/image_raw` | `sensor_msgs/Image` | camera_node | 30Hz | Left camera raw image |
| `/camera/right/image_raw` | `sensor_msgs/Image` | camera_node | 30Hz | Right camera raw image |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera_node | 30Hz | Camera calibration info |
| `/detected_peppers` | `PepperArray` | vision_node | 10Hz | Detected peppers with 3D pos |
| `/arm/left/command` | `ArmCommand` | task_planner | ~1Hz | Command for left arm |
| `/arm/right/command` | `ArmCommand` | task_planner | ~1Hz | Command for right arm |
| `/arm/left/status` | `ArmStatus` | arm_left_node | 10Hz | Left arm current status |
| `/arm/right/status` | `ArmStatus` | arm_right_node | 10Hz | Right arm current status |

### Optional Debug Topics

| Topic | Message Type | Publisher | Description |
|-------|-------------|-----------|-------------|
| `/vision/debug_image` | `sensor_msgs/Image` | vision_node | Annotated detection image |
| `/vision/depth_map` | `sensor_msgs/Image` | vision_node | Depth visualization |
| `/planner/statistics` | `custom msg` | task_planner | Performance metrics |

---

## Package Dependencies

### System Dependencies

```bash
# ROS2 Humble packages
ros-humble-desktop
ros-humble-vision-opencv
ros-humble-cv-bridge
ros-humble-image-transport
ros-humble-tf2
ros-humble-tf2-ros
ros-humble-tf2-geometry-msgs

# Python dependencies
python3-pip
python3-opencv
python3-numpy
python3-scipy
```

### Python Package Dependencies

```bash
# Vision & AI
ultralytics          # YOLOv8
torch                # PyTorch (for YOLO)
torchvision
tensorrt             # Jetson optimization (optional)

# Serial communication
pyserial             # Arduino communication

# Math & utilities
numpy
scipy
opencv-python
transforms3d         # 3D transformations
```

### Package Dependencies Graph

```
pepper_bringup
  ├── pepper_vision
  │     └── pepper_msgs
  ├── pepper_control
  │     └── pepper_msgs
  └── pepper_planning
        ├── pepper_msgs
        ├── pepper_vision (indirect)
        └── pepper_control (indirect)
```

---

## Build System

### Using Colcon (ROS2 Build Tool)

```bash
# Initial build
cd ~/pepper_sorting_ws
colcon build

# Build specific package
colcon build --packages-select pepper_vision

# Build with symlink (for Python development)
colcon build --symlink-install

# Clean build
rm -rf build/ install/ log/
colcon build
```

### CMakeLists.txt (for pepper_msgs)

```cmake
cmake_minimum_required(VERSION 3.8)
project(pepper_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedPepper.msg"
  "msg/PepperArray.msg"
  "msg/ArmCommand.msg"
  "msg/ArmStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
```

### setup.py (for Python packages)

```python
from setuptools import setup

package_name = 'pepper_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vision.launch.py']),
        ('share/' + package_name + '/config', ['config/vision_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='jay@example.com',
    description='Vision processing for pepper sorting',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = pepper_vision.camera_node:main',
            'vision_node = pepper_vision.vision_node:main',
        ],
    },
)
```

---

## Launch File Strategy

### Main System Launch (`system.launch.py`)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    vision_pkg = get_package_share_directory('pepper_vision')
    control_pkg = get_package_share_directory('pepper_control')
    planning_pkg = get_package_share_directory('pepper_planning')

    return LaunchDescription([
        # Launch vision system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(vision_pkg, 'launch', 'vision.launch.py')
            )
        ),

        # Launch left arm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg, 'launch', 'arm_left.launch.py')
            )
        ),

        # Launch right arm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg, 'launch', 'arm_right.launch.py')
            )
        ),

        # Launch task planner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planning_pkg, 'launch', 'planner.launch.py')
            )
        ),

        # Optional: Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('pepper_bringup'),
                'config', 'rviz_config.rviz'
            )]
        ),
    ])
```

### Modular Launch Files

Each subsystem has its own launch file for testing:
- `vision.launch.py` - Camera + vision node only
- `arm_left.launch.py` - Left arm controller only
- `arm_right.launch.py` - Right arm controller only
- `planner.launch.py` - Task planner only

---

## Configuration Management

### Parameter Files (YAML)

Example: `vision_params.yaml`

```yaml
camera_node:
  ros__parameters:
    frame_rate: 30
    resolution_width: 1280
    resolution_height: 720
    auto_exposure: true
    calibration_file: "stereo_calib.yaml"

vision_node:
  ros__parameters:
    model_path: "models/yolo_pepper_v8.pt"
    confidence_threshold: 0.6
    depth_min: 0.2  # meters
    depth_max: 1.0  # meters
    stereo_algorithm: "SGBM"
    enable_visualization: true

    # Class mapping
    class_names:
      - "pepper_red"
      - "pepper_green"
      - "pepper_yellow"
```

### Loading Parameters in Nodes

```python
import rclpy
from rclpy.node import Node

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Declare and get parameters
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('model_path', '')

        self.confidence = self.get_parameter('confidence_threshold').value
        self.model_path = self.get_parameter('model_path').value

        self.get_logger().info(f'Confidence threshold: {self.confidence}')
```

---

## Data Flow Example

### Typical Execution Flow

1. **Camera Capture** (30 Hz):
   ```
   camera_node captures stereo pair
   → publishes /camera/left/image_raw
   → publishes /camera/right/image_raw
   ```

2. **Vision Processing** (10 Hz):
   ```
   vision_node receives images
   → runs YOLO detection
   → computes depth map
   → calculates 3D positions
   → publishes /detected_peppers
   ```

3. **Task Planning** (~1 Hz):
   ```
   task_planner receives /detected_peppers
   → checks arm statuses (/arm/left/status, /arm/right/status)
   → assigns pepper to available arm
   → publishes /arm/left/command or /arm/right/command
   ```

4. **Arm Execution** (~0.2 Hz per pick):
   ```
   arm_node receives /arm/left/command
   → computes inverse kinematics
   → sends serial commands to Arduino
   → executes pick & place
   → publishes /arm/left/status (idle → moving → picking → placing → idle)
   ```

---

## Communication Protocols

### ROS2 Topics (Publish/Subscribe)
- **Use for**: Continuous data streams (images, status updates)
- **QoS**: Reliable or Best Effort depending on criticality

### ROS2 Services (Request/Response)
- **Use for**: One-time requests (emergency stop, re-calibration)
- Not heavily used in this project (mostly topics)

### ROS2 Actions (Goal-based)
- **Use for**: Long-running tasks with feedback
- Could be used for arm movements (future enhancement)

---

## State Machine (Task Planner)

```
┌─────────┐
│  IDLE   │ ← System starts here
└────┬────┘
     │ /detected_peppers received
     ▼
┌─────────────┐
│ ASSIGNING   │ Check arm availability
└────┬────────┘
     │ Arm available
     ▼
┌─────────────┐
│ COMMANDING  │ Send /arm/*/command
└────┬────────┘
     │
     ▼
┌─────────────┐
│ MONITORING  │ Wait for arm completion
└────┬────────┘
     │ Status = idle
     ▼
┌─────────┐
│  IDLE   │ ← Back to idle, ready for next
└─────────┘
```

---

## Error Handling Strategy

### Vision Node
- **Camera disconnection**: Retry connection, publish warning
- **Detection failure**: Skip frame, log event
- **Invalid depth**: Filter out, don't publish

### Arm Controller Node
- **Serial timeout**: Retry command (max 3 times)
- **IK no solution**: Skip target, report error
- **Gripper failure**: Return home, flag for manual check

### Task Planner
- **No arms available**: Queue tasks
- **Pepper out of reach**: Mark as unreachable, skip
- **Collision detected**: Halt both arms, wait for clear

---

## Performance Considerations

### CPU/GPU Allocation (Jetson)

| Task | Resource | % Usage |
|------|----------|---------|
| Camera capture | CPU | 5-10% |
| YOLO inference | GPU | 40-60% |
| Stereo matching | GPU/CPU | 20-30% |
| ROS2 overhead | CPU | 5-10% |
| Arm control | CPU | 5% |
| **Total** | - | **~70-80% GPU, 40-50% CPU** |

**Optimization Tips**:
- Use TensorRT for YOLO (2-3× speedup on Jetson)
- Reduce camera resolution if needed (1280×720 instead of full res)
- Run vision_node at lower Hz (10 Hz sufficient)

---

## Development Workflow

### 1. Build Workflow
```bash
cd ~/pepper_sorting_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Testing Workflow
```bash
# Test individual nodes
ros2 run pepper_vision camera_node
ros2 run pepper_vision vision_node

# Test with launch file
ros2 launch pepper_vision vision.launch.py

# Monitor topics
ros2 topic list
ros2 topic echo /detected_peppers
```

### 3. Debugging Workflow
```bash
# Check node graph
rqt_graph

# View logs
ros2 run rqt_console rqt_console

# Record data for playback
ros2 bag record -a
```

---

## Next Steps

1. → See [ROS2 Nodes Detail](03_ros2_nodes_detail.md) for implementation details
2. → See [Custom Messages](04_custom_messages.md) for message definitions
3. → See [Development Roadmap](07_development_roadmap.md) for build sequence

---

**Software Architecture Complete ✓**
