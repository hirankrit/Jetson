# 3. ROS2 Nodes Detail

## Overview

This document provides detailed specifications for each ROS2 node in the pepper sorting system.

---

## 1. Camera Node

**Package**: `pepper_vision`
**File**: `pepper_vision/camera_node.py`
**Purpose**: Capture stereo images from IMX219 camera and publish synchronized frames

### Publishers

| Topic | Type | Rate | QoS |
|-------|------|------|-----|
| `/camera/left/image_raw` | `sensor_msgs/Image` | 30Hz | Best Effort |
| `/camera/right/image_raw` | `sensor_msgs/Image` | 30Hz | Best Effort |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30Hz | Reliable |

### Parameters

```yaml
camera_node:
  ros__parameters:
    frame_rate: 30
    resolution_width: 1280
    resolution_height: 720
    auto_exposure: true
    exposure_value: 50  # if auto_exposure is false
    calibration_file: "config/stereo_calib.yaml"
    device_id: 0  # CSI camera device
```

### Implementation Outline

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Publishers
        self.pub_left = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        # Parameters
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('resolution_width', 1280)

        # Initialize camera (GStreamer pipeline for IMX219)
        self.init_camera()

        # Timer for capturing
        self.timer = self.create_timer(1.0 / self.frame_rate, self.capture_callback)

    def init_camera(self):
        # GStreamer pipeline for stereo camera
        # Left: sensor-id=0, Right: sensor-id=1
        pass

    def capture_callback(self):
        # Capture frames, convert to ROS Image, publish
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Key Functions

- `init_camera()`: Setup CSI camera interface using GStreamer
- `capture_callback()`: Capture and publish frames
- `load_calibration()`: Load camera calibration from YAML

---

## 2. Vision Node

**Package**: `pepper_vision`
**File**: `pepper_vision/vision_node.py`
**Purpose**: Detect peppers, classify them, and compute 3D positions

### Subscribers

| Topic | Type | Callback |
|-------|------|----------|
| `/camera/left/image_raw` | `sensor_msgs/Image` | `image_left_callback` |
| `/camera/right/image_raw` | `sensor_msgs/Image` | `image_right_callback` |

### Publishers

| Topic | Type | Rate |
|-------|------|------|
| `/detected_peppers` | `PepperArray` | 10Hz |
| `/vision/debug_image` | `sensor_msgs/Image` | 10Hz (optional) |

### Parameters

```yaml
vision_node:
  ros__parameters:
    model_path: "models/yolo_pepper_v8.pt"
    confidence_threshold: 0.6
    iou_threshold: 0.4
    depth_min: 0.2  # meters
    depth_max: 1.0
    stereo_algorithm: "SGBM"  # or "BM"
    enable_visualization: true

    # Stereo matching params
    num_disparities: 128
    block_size: 11
```

### Processing Pipeline

```python
class VisionNode(Node):
    def __init__(self):
        # 1. Load YOLO model
        self.model = YOLO(model_path)

        # 2. Initialize stereo matcher
        self.stereo_matcher = cv2.StereoSGBM_create(...)

        # 3. Setup message filters for synchronization
        from message_filters import ApproximateTimeSynchronizer, Subscriber

        self.sub_left = Subscriber(self, Image, '/camera/left/image_raw')
        self.sub_right = Subscriber(self, Image, '/camera/right/image_raw')

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right],
            queue_size=10,
            slop=0.05  # 50ms tolerance
        )
        self.sync.registerCallback(self.stereo_callback)

    def stereo_callback(self, msg_left, msg_right):
        # Convert ROS Image to OpenCV
        img_left = self.bridge.imgmsg_to_cv2(msg_left, "bgr8")
        img_right = self.bridge.imgmsg_to_cv2(msg_right, "bgr8")

        # 1. Run YOLO on left image
        results = self.model(img_left, conf=self.confidence_threshold)

        # 2. Compute disparity map
        disparity = self.stereo_matcher.compute(img_left_gray, img_right_gray)

        # 3. For each detected pepper:
        peppers = []
        for detection in results[0].boxes:
            # Get bounding box center
            x, y = detection.xyxy[0][:2]  # top-left
            w, h = detection.xyxy[0][2:] - detection.xyxy[0][:2]
            cx, cy = int(x + w/2), int(y + h/2)

            # Get depth from disparity at center point
            depth = self.disparity_to_depth(disparity[cy, cx])

            # Convert to 3D coordinates
            X, Y, Z = self.pixel_to_3d(cx, cy, depth)

            # Classify color and quality
            color = self.classify_color(img_left, detection)
            quality = self.classify_quality(img_left, detection)

            # Create DetectedPepper message
            pepper = DetectedPepper()
            pepper.position.x = X
            pepper.position.y = Y
            pepper.position.z = Z
            pepper.color = color
            pepper.quality = quality
            pepper.confidence = detection.conf

            peppers.append(pepper)

        # 4. Publish PepperArray
        msg = PepperArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.peppers = peppers
        self.pub_peppers.publish(msg)
```

### Helper Classes

**StereoProcessor** (`stereo_processor.py`):
```python
class StereoProcessor:
    def __init__(self, calibration_file):
        # Load stereo calibration parameters
        self.K_left, self.D_left = load_intrinsics('left')
        self.K_right, self.D_right = load_intrinsics('right')
        self.R, self.T, self.Q = load_extrinsics()

        # Create rectification maps
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(...)
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(...)

    def rectify(self, img_left, img_right):
        # Apply rectification
        rect_left = cv2.remap(img_left, self.map_left_x, self.map_left_y, ...)
        rect_right = cv2.remap(img_right, self.map_right_x, self.map_right_y, ...)
        return rect_left, rect_right

    def compute_disparity(self, rect_left, rect_right):
        # Compute disparity map
        return self.stereo_matcher.compute(rect_left, rect_right)

    def disparity_to_depth(self, disparity):
        # Z = (f * baseline) / disparity
        return (self.Q[2, 3] / (disparity + self.Q[3, 3]))
```

**PepperDetector** (`pepper_detector.py`):
```python
class PepperDetector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)

    def detect(self, image):
        # Run YOLO inference
        results = self.model(image)
        return results

    def classify_color(self, image, bbox):
        # Extract ROI and analyze color histogram
        # Return "red", "green", or "yellow"
        pass

    def classify_quality(self, image, bbox):
        # Analyze texture, spots, etc.
        # Return "fresh" or "rotten"
        pass
```

---

## 3. Task Planner Node

**Package**: `pepper_planning`
**File**: `pepper_planning/task_planner_node.py`
**Purpose**: Coordinate dual arms, assign tasks, avoid conflicts

### Subscribers

| Topic | Type |
|-------|------|
| `/detected_peppers` | `PepperArray` |
| `/arm/left/status` | `ArmStatus` |
| `/arm/right/status` | `ArmStatus` |

### Publishers

| Topic | Type |
|-------|------|
| `/arm/left/command` | `ArmCommand` |
| `/arm/right/command` | `ArmCommand` |
| `/planner/statistics` | (custom) |

### Parameters

```yaml
task_planner:
  ros__parameters:
    workspace_split: 0.0  # X=0 divides left/right
    priority_mode: "closest"  # or "best_quality", "fifo"
    collision_margin: 0.05  # meters
    max_queue_size: 20
    enable_coordination: true
```

### State Machine

```python
class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        # Tracking
        self.pending_peppers = []  # Queue of unassigned peppers
        self.assigned_peppers = {}  # {pepper_id: arm_name}
        self.arm_status = {
            'left': 'idle',
            'right': 'idle'
        }

        # Subscribers
        self.sub_peppers = self.create_subscription(
            PepperArray, '/detected_peppers', self.peppers_callback, 10
        )
        self.sub_left_status = self.create_subscription(
            ArmStatus, '/arm/left/status', self.left_status_callback, 10
        )
        self.sub_right_status = self.create_subscription(
            ArmStatus, '/arm/right/status', self.right_status_callback, 10
        )

        # Publishers
        self.pub_left_cmd = self.create_publisher(ArmCommand, '/arm/left/command', 10)
        self.pub_right_cmd = self.create_publisher(ArmCommand, '/arm/right/command', 10)

        # Timer for task assignment
        self.timer = self.create_timer(0.5, self.assign_tasks)

    def peppers_callback(self, msg):
        # Add new peppers to queue
        for pepper in msg.peppers:
            if pepper.id not in self.assigned_peppers:
                self.pending_peppers.append(pepper)

    def left_status_callback(self, msg):
        self.arm_status['left'] = msg.state
        # If completed, remove from assigned
        if msg.state == 'idle' and msg.current_pepper_id in self.assigned_peppers:
            del self.assigned_peppers[msg.current_pepper_id]

    def assign_tasks(self):
        # Check which arms are idle
        if self.arm_status['left'] == 'idle' and len(self.pending_peppers) > 0:
            # Assign pepper to left arm
            pepper = self.select_pepper_for_arm('left')
            if pepper:
                self.send_command('left', pepper)

        if self.arm_status['right'] == 'idle' and len(self.pending_peppers) > 0:
            pepper = self.select_pepper_for_arm('right')
            if pepper:
                self.send_command('right', pepper)

    def select_pepper_for_arm(self, arm_name):
        # Filter peppers in arm's workspace
        valid_peppers = [p for p in self.pending_peppers
                         if self.is_in_workspace(p, arm_name)]

        if not valid_peppers:
            return None

        # Sort by priority (closest, best quality, etc.)
        if self.priority_mode == 'closest':
            pepper = min(valid_peppers, key=lambda p: self.distance_to_arm(p, arm_name))
        else:
            pepper = valid_peppers[0]

        # Remove from queue
        self.pending_peppers.remove(pepper)
        self.assigned_peppers[pepper.id] = arm_name

        return pepper

    def send_command(self, arm_name, pepper):
        cmd = ArmCommand()
        cmd.pickup_position = pepper.position
        cmd.place_position = self.get_sorting_bin(pepper)
        cmd.pepper_id = pepper.id
        cmd.sorting_category = f"{pepper.quality}_{pepper.color}"
        cmd.command_type = "pick_place"

        if arm_name == 'left':
            self.pub_left_cmd.publish(cmd)
        else:
            self.pub_right_cmd.publish(cmd)
```

---

## 4. Arm Controller Node

**Package**: `pepper_control`
**File**: `pepper_control/arm_controller_node.py`
**Purpose**: Control one robot arm (run 2 instances for dual arms)

### Subscribers

| Topic | Type |
|-------|------|
| `/arm/{left\|right}/command` | `ArmCommand` |

### Publishers

| Topic | Type | Rate |
|-------|------|------|
| `/arm/{left\|right}/status` | `ArmStatus` | 10Hz |

### Parameters

```yaml
arm_left_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baud_rate: 115200
    arm_id: "left"
    dof: 5
    servo_limits: [0, 180, 0, 180, 0, 180, 0, 180, 0, 180]
    gripper_close_angle: 90
    gripper_open_angle: 30
    home_position: [90, 90, 90, 90, 45]

    # Kinematics
    link_lengths: [0.10, 0.10, 0.08, 0.05]  # meters
    transform_matrix: "config/hand_eye_calib_left.yaml"
```

### Implementation

```python
class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('arm_id', 'left')

        arm_id = self.get_parameter('arm_id').value

        # Serial interface
        self.serial = SerialInterface(serial_port, baud_rate)

        # Inverse kinematics solver
        self.ik_solver = InverseKinematics(link_lengths)

        # State
        self.state = 'idle'
        self.current_pepper_id = 0

        # Subscribers
        self.sub_cmd = self.create_subscription(
            ArmCommand, f'/arm/{arm_id}/command', self.command_callback, 10
        )

        # Publishers
        self.pub_status = self.create_publisher(ArmStatus, f'/arm/{arm_id}/status', 10)

        # Status publishing timer
        self.timer = self.create_timer(0.1, self.publish_status)

    def command_callback(self, msg):
        if msg.command_type == 'pick_place':
            self.execute_pick_place(msg)
        elif msg.command_type == 'home':
            self.go_home()
        elif msg.command_type == 'stop':
            self.emergency_stop()

    def execute_pick_place(self, cmd):
        self.state = 'moving'
        self.current_pepper_id = cmd.pepper_id

        # Convert world coordinates to arm coordinates
        pickup_arm = self.transform_world_to_arm(cmd.pickup_position)
        place_arm = self.transform_world_to_arm(cmd.place_position)

        # Pick sequence
        self.state = 'picking'
        self.pick(pickup_arm)

        # Place sequence
        self.state = 'placing'
        self.place(place_arm)

        # Return home
        self.go_home()
        self.state = 'idle'
        self.current_pepper_id = 0

    def pick(self, position):
        # 1. Move above pickup position
        above_pos = position.copy()
        above_pos.z += 0.05  # 5cm above
        self.move_to(above_pos, gripper_open=True)

        # 2. Lower down
        self.move_to(position, gripper_open=True)

        # 3. Close gripper
        self.gripper_close()

        # 4. Lift up
        self.move_to(above_pos, gripper_open=False)

    def move_to(self, position, gripper_open=True):
        # Solve IK
        angles = self.ik_solver.solve(position)

        if angles is None:
            self.get_logger().error(f'IK failed for position {position}')
            return False

        # Send to Arduino
        gripper_angle = self.gripper_open_angle if gripper_open else self.gripper_close_angle
        cmd = f"MOVE,{angles[0]},{angles[1]},{angles[2]},{angles[3]},{angles[4]},{gripper_angle}\n"

        self.serial.send(cmd)
        response = self.serial.wait_for_response(timeout=5.0)

        if response != 'OK':
            self.get_logger().error(f'Arduino error: {response}')
            return False

        return True

    def publish_status(self):
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.current_pepper_id = self.current_pepper_id
        # ... fill other fields
        self.pub_status.publish(msg)
```

---

## 5. Arduino Firmware

**File**: `pepper_control/arduino/arm_controller.ino`

### Command Protocol

**Commands from Jetson to Arduino**:
```
MOVE,<a1>,<a2>,<a3>,<a4>,<a5>,<gripper>\n
HOME\n
STOP\n
STATUS\n
```

**Responses from Arduino to Jetson**:
```
OK\n
ERROR,<message>\n
STATUS,<state>,<a1>,<a2>,<a3>,<a4>,<a5>,<gripper>\n
```

### Arduino Code Outline

```cpp
#include <Servo.h>

Servo servos[6];  // 5 joints + 1 gripper
int servoPins[] = {3, 5, 6, 9, 10, 11};
int targetAngles[6];
int currentAngles[6];

void setup() {
  Serial.begin(115200);

  // Attach servos
  for (int i = 0; i < 6; i++) {
    servos[i].attach(servoPins[i]);
    targetAngles[i] = 90;
    currentAngles[i] = 90;
  }

  Serial.println("OK");  // Ready signal
}

void loop() {
  // Check for incoming commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  // Smooth servo movement
  updateServos();
}

void handleCommand(String cmd) {
  if (cmd.startsWith("MOVE,")) {
    parseAngles(cmd);
    Serial.println("OK");
  }
  else if (cmd == "HOME") {
    goHome();
    Serial.println("OK");
  }
  else if (cmd == "STOP") {
    // Emergency stop
    for (int i = 0; i < 6; i++) {
      targetAngles[i] = currentAngles[i];
    }
    Serial.println("OK");
  }
  else if (cmd == "STATUS") {
    sendStatus();
  }
  else {
    Serial.println("ERROR,Unknown command");
  }
}

void parseAngles(String cmd) {
  // Parse "MOVE,90,45,120,60,30,90"
  int idx = 5;  // Skip "MOVE,"
  for (int i = 0; i < 6; i++) {
    int commaIdx = cmd.indexOf(',', idx);
    if (commaIdx == -1) commaIdx = cmd.length();

    String angleStr = cmd.substring(idx, commaIdx);
    targetAngles[i] = angleStr.toInt();

    idx = commaIdx + 1;
  }
}

void updateServos() {
  // Smooth movement (increment by 1 degree per step)
  for (int i = 0; i < 6; i++) {
    if (currentAngles[i] < targetAngles[i]) {
      currentAngles[i]++;
      servos[i].write(currentAngles[i]);
    }
    else if (currentAngles[i] > targetAngles[i]) {
      currentAngles[i]--;
      servos[i].write(currentAngles[i]);
    }
  }
  delay(15);  // ~60Hz update rate
}
```

---

## Node Launch Commands

```bash
# Launch individual nodes for testing

# Camera
ros2 run pepper_vision camera_node

# Vision
ros2 run pepper_vision vision_node

# Task planner
ros2 run pepper_planning task_planner_node

# Left arm
ros2 run pepper_control arm_controller_node --ros-args -p arm_id:=left -p serial_port:=/dev/ttyUSB0

# Right arm
ros2 run pepper_control arm_controller_node --ros-args -p arm_id:=right -p serial_port:=/dev/ttyUSB1
```

---

**ROS2 Nodes Detail Complete ✓**
