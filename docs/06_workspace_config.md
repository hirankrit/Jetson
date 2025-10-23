# 6. Workspace Configuration

## Configuration Files Overview

All configuration parameters are stored in YAML files for easy modification without recompiling code.

```
pepper_bringup/config/
├── workspace_config.yaml       # Workspace zones and dimensions
├── vision_params.yaml          # Camera and vision settings
├── arm_left_params.yaml        # Left arm parameters
├── arm_right_params.yaml       # Right arm parameters
└── system_params.yaml          # Global system settings
```

---

## workspace_config.yaml

Defines physical workspace layout, zones, and boundaries.

```yaml
workspace:
  # World frame origin (at center of pile)
  origin:
    x: 0.0
    y: 0.0
    z: 0.0

  # Input pile area (where peppers are placed)
  pile_area:
    center: [0.0, 0.0, 0.0]     # meters
    radius: 0.15                 # meters (15cm diameter circle)
    height_range: [0.0, 0.05]    # Z range: 0-5cm

  # Sorting output zones (where sorted peppers go)
  sorting_zones:
    fresh_red:
      center: [-0.10, 0.25, 0.0]
      size: [0.08, 0.08, 0.05]   # X, Y, Z (width, depth, height)
      label: "Fresh Red"

    fresh_green:
      center: [0.0, 0.25, 0.0]
      size: [0.08, 0.08, 0.05]
      label: "Fresh Green"

    fresh_yellow:
      center: [0.10, 0.25, 0.0]
      size: [0.08, 0.08, 0.05]
      label: "Fresh Yellow"

    reject:
      center: [0.20, 0.25, 0.0]
      size: [0.10, 0.08, 0.05]
      label: "Reject (Rotten)"

  # Arm workspace division
  arm_workspaces:
    left:
      x_range: [-0.25, 0.0]      # Left half (X < 0)
      y_range: [-0.10, 0.30]
      z_range: [0.0, 0.30]
      priority_zone: [-0.15, -0.05]  # Preferred X range

    right:
      x_range: [0.0, 0.25]       # Right half (X > 0)
      y_range: [-0.10, 0.30]
      z_range: [0.0, 0.30]
      priority_zone: [0.05, 0.15]

  # Safety margins
  collision_margin: 0.05         # meters (5cm safety zone)
  boundary_margin: 0.02          # meters (2cm from workspace edge)

  # Task assignment
  overlap_zone: [-0.05, 0.05]   # X range where both arms can reach
  handoff_enabled: false         # Allow task handoff between arms
```

---

## vision_params.yaml

Vision system configuration.

```yaml
camera_node:
  ros__parameters:
    # Camera hardware
    device_id: 0
    frame_rate: 30
    resolution:
      width: 1280
      height: 720

    # Exposure settings
    auto_exposure: true
    exposure_value: 50           # if auto_exposure is false
    gain: 1.0

    # Calibration
    calibration_file: "config/stereo_calib.yaml"
    publish_debug: false

vision_node:
  ros__parameters:
    # YOLO detection
    model_path: "models/yolo_pepper_v8.pt"
    model_type: "yolov8n-seg"    # nano segmentation model
    confidence_threshold: 0.6
    iou_threshold: 0.4
    max_detections: 20

    # Classification
    color_classes:
      - "red"
      - "green"
      - "yellow"
    quality_classes:
      - "fresh"
      - "rotten"

    # Stereo depth
    depth_min: 0.20              # meters (minimum valid depth)
    depth_max: 1.00              # meters (maximum valid depth)
    stereo_algorithm: "SGBM"     # "SGBM" or "BM"

    # SGBM parameters
    num_disparities: 128         # Must be divisible by 16
    block_size: 11               # Odd number, typically 5-21
    min_disparity: 0
    uniqueness_ratio: 10
    speckle_window_size: 100
    speckle_range: 32
    disp12_max_diff: 1

    # Filtering
    filter_unreachable: true     # Filter peppers outside workspace
    min_size_mm: 20.0            # Minimum pepper size (mm)
    max_size_mm: 80.0            # Maximum pepper size (mm)

    # Performance
    processing_rate: 10          # Hz (process every Nth frame)
    enable_visualization: true   # Publish debug images
    enable_profiling: false      # Log processing times

    # ROS topics
    publish_debug_image: true
    publish_depth_map: false
```

---

## arm_left_params.yaml

Left arm configuration.

```yaml
arm_left_node:
  ros__parameters:
    # Identification
    arm_id: "left"
    arm_name: "Left Arm"

    # Serial communication
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    serial_timeout: 5.0          # seconds
    retry_attempts: 3

    # Kinematics
    dof: 5                       # Degrees of freedom (excluding gripper)
    link_lengths: [0.10, 0.10, 0.08, 0.05]  # meters (base to tip)

    # Joint limits (degrees)
    joint_limits:
      joint1: [0, 180]
      joint2: [0, 180]
      joint3: [0, 180]
      joint4: [0, 180]
      joint5: [0, 180]

    # Home position (degrees)
    home_position: [90, 90, 90, 90, 45]

    # Gripper
    gripper_open_angle: 30       # degrees
    gripper_close_angle: 90      # degrees
    gripper_force: 0.5           # Normalized [0-1] (if supported)

    # Motion parameters
    speed_scale: 0.8             # Default speed [0.1-1.0]
    acceleration: 1.0            # Acceleration factor
    smooth_motion: true          # Enable trajectory smoothing

    # Pick & place sequence
    approach_height: 0.05        # meters above target
    lift_height: 0.08            # meters to lift after pick
    gripper_close_delay: 0.5     # seconds to wait after closing
    gripper_open_delay: 0.3      # seconds to wait after opening

    # Calibration
    transform_file: "config/hand_eye_calib_left.yaml"

    # Safety
    workspace_limits:
      x: [-0.25, 0.05]
      y: [-0.10, 0.35]
      z: [0.0, 0.30]
    enable_workspace_check: true
    emergency_stop_enabled: true

    # Error handling
    max_ik_iterations: 100
    ik_tolerance: 0.001          # meters
    retry_on_failure: true
    max_task_retries: 2

    # Performance monitoring
    enable_statistics: true
    log_level: "info"            # "debug", "info", "warn", "error"
```

---

## arm_right_params.yaml

Right arm configuration (similar to left, with different serial port and calibration).

```yaml
arm_right_node:
  ros__parameters:
    arm_id: "right"
    arm_name: "Right Arm"
    serial_port: "/dev/ttyUSB1"
    # ... (rest same as left arm, except transform_file)
    transform_file: "config/hand_eye_calib_right.yaml"
    workspace_limits:
      x: [-0.05, 0.25]           # Right side workspace
      y: [-0.10, 0.35]
      z: [0.0, 0.30]
```

---

## system_params.yaml

Global system parameters.

```yaml
system:
  ros__parameters:
    # System identification
    system_name: "Pepper Sorting Robot"
    version: "0.1.0"

    # Operating mode
    mode: "auto"                 # "auto", "manual", "calibration", "test"
    enable_dual_arms: true       # Use both arms
    enable_safety_checks: true

    # Performance targets
    target_throughput: 6         # peppers per minute
    max_cycle_time: 10.0         # seconds per pepper
    idle_timeout: 30.0           # seconds before standby

    # Logging
    enable_rosbag: false         # Auto-record data
    log_directory: "~/pepper_logs"
    statistics_interval: 60      # seconds (publish stats every minute)

task_planner:
  ros__parameters:
    # Task assignment
    priority_mode: "closest"     # "closest", "best_quality", "fifo", "random"
    load_balancing: true         # Balance tasks between arms
    allow_queue_overflow: false

    # Queue management
    max_queue_size: 20
    queue_timeout: 30.0          # seconds (discard old tasks)

    # Coordination
    collision_check_enabled: true
    simultaneous_picks: true     # Both arms can pick at same time
    wait_for_completion: false   # Don't block on task completion

    # Sorting strategy
    sorting_strategy: "by_quality"  # "by_color", "by_quality", "by_size"
    prioritize_fresh: true       # Pick fresh peppers first

    # Performance
    update_rate: 2               # Hz (task assignment frequency)
    enable_replanning: true      # Replan if better task available
```

---

## stereo_calib.yaml

Camera calibration data (generated by calibration script).

```yaml
calibration:
  # Left camera intrinsics
  left:
    camera_matrix:
      rows: 3
      cols: 3
      data: [700.0, 0.0, 640.0,
             0.0, 700.0, 360.0,
             0.0, 0.0, 1.0]
    distortion_coefficients:
      rows: 1
      cols: 5
      data: [0.1, -0.05, 0.001, 0.001, 0.0]

  # Right camera intrinsics
  right:
    camera_matrix:
      rows: 3
      cols: 3
      data: [700.0, 0.0, 640.0,
             0.0, 700.0, 360.0,
             0.0, 0.0, 1.0]
    distortion_coefficients:
      rows: 1
      cols: 5
      data: [0.1, -0.05, 0.001, 0.001, 0.0]

  # Stereo extrinsics
  stereo:
    rotation_matrix:
      rows: 3
      cols: 3
      data: [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    translation_vector:
      rows: 3
      cols: 1
      data: [0.06, 0.0, 0.0]  # 60mm baseline

  # Rectification (from cv2.stereoRectify)
  rectification:
    Q_matrix:
      rows: 4
      cols: 4
      data: [1.0, 0.0, 0.0, -640.0,
             0.0, 1.0, 0.0, -360.0,
             0.0, 0.0, 0.0, 700.0,
             0.0, 0.0, 16.667, 0.0]  # 1/baseline
```

---

## hand_eye_calib_left.yaml

Hand-eye calibration for left arm.

```yaml
transformation:
  # 4x4 transformation matrix: T_camera_to_arm
  # Converts points from camera frame to arm base frame
  matrix:
    rows: 4
    cols: 4
    data: [0.0, -1.0, 0.0, 0.15,
           1.0, 0.0, 0.0, 0.5,
           0.0, 0.0, 1.0, -0.6,
           0.0, 0.0, 0.0, 1.0]

  # Calibration metadata
  calibration_date: "2025-10-22"
  reprojection_error: 0.004     # meters (4mm)
  num_samples: 15
  method: "eye-to-hand"
```

---

## Loading Parameters in Nodes

### Python Example

```python
import yaml
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Declare parameters with defaults
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('max_detections', 20)

        # Get parameter values
        self.confidence = self.get_parameter('confidence_threshold').value
        self.max_det = self.get_parameter('max_detections').value

        self.get_logger().info(f'Confidence: {self.confidence}')

# Launch with parameters
# ros2 run pkg node --ros-args --params-file config/params.yaml
```

### Launch File with Parameters

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pepper_vision'),
        'config',
        'vision_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='pepper_vision',
            executable='vision_node',
            name='vision_node',
            parameters=[config],
            output='screen'
        )
    ])
```

---

## Dynamic Reconfiguration

For parameters that need to be changed at runtime:

```bash
# List parameters
ros2 param list /vision_node

# Get parameter value
ros2 param get /vision_node confidence_threshold

# Set parameter value
ros2 param set /vision_node confidence_threshold 0.7

# Dump all parameters to file
ros2 param dump /vision_node > current_params.yaml
```

---

**Workspace Configuration Complete ✓**
