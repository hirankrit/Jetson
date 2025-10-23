# 4. Custom ROS2 Messages

## Message Package Structure

```
pepper_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── DetectedPepper.msg
    ├── PepperArray.msg
    ├── ArmCommand.msg
    └── ArmStatus.msg
```

---

## 1. DetectedPepper.msg

**Purpose**: Represents a single detected pepper with 3D position and classification

```python
# Header with timestamp
std_msgs/Header header

# 3D Position in world frame (meters)
geometry_msgs/Point position
# position.x: left(-) to right(+)
# position.y: back(-) to front(+)
# position.z: height from workspace (usually small, ~0-0.05m)

# Classification results
string color                    # "red", "green", "yellow"
string quality                  # "fresh", "rotten"

# Measurements
float32 size_mm                 # Estimated size in millimeters
float32 confidence              # Detection confidence [0.0-1.0]

# Tracking
int32 id                        # Unique pepper ID for tracking
bool is_reachable               # Can arms reach this position?

# Optional: Bounding box in image space (for visualization)
int32 bbox_x
int32 bbox_y
int32 bbox_width
int32 bbox_height
```

### Example Usage

```python
from pepper_msgs.msg import DetectedPepper
from geometry_msgs.msg import Point

pepper = DetectedPepper()
pepper.header.stamp = node.get_clock().now().to_msg()
pepper.header.frame_id = "world"

pepper.position = Point(x=0.05, y=0.10, z=0.02)
pepper.color = "red"
pepper.quality = "fresh"
pepper.size_mm = 45.2
pepper.confidence = 0.92
pepper.id = 1001
pepper.is_reachable = True
```

---

## 2. PepperArray.msg

**Purpose**: Array of detected peppers in a single frame

```python
# Header with timestamp
std_msgs/Header header

# Array of detected peppers
DetectedPepper[] peppers

# Metadata
int32 total_count               # Total number of peppers detected
float32 average_confidence      # Average confidence across all detections

# Frame info (optional)
int32 frame_id                  # Sequential frame number
```

### Example Usage

```python
from pepper_msgs.msg import PepperArray

msg = PepperArray()
msg.header.stamp = node.get_clock().now().to_msg()
msg.header.frame_id = "world"

msg.peppers = [pepper1, pepper2, pepper3]
msg.total_count = len(msg.peppers)
msg.average_confidence = sum(p.confidence for p in msg.peppers) / len(msg.peppers)

publisher.publish(msg)
```

---

## 3. ArmCommand.msg

**Purpose**: Command message for robot arm pick-and-place operations

```python
# Header with timestamp
std_msgs/Header header

# Target positions (in world frame, meters)
geometry_msgs/Point pickup_position    # Where to pick the pepper
geometry_msgs/Point place_position     # Where to place it

# Pepper information (for logging/tracking)
int32 pepper_id                        # ID from DetectedPepper
string sorting_category                # e.g., "fresh_red", "rotten_green"

# Command type
string command_type                    # "pick_place", "home", "stop", "calibrate"

# Optional: Motion parameters
float32 speed_scale                    # Speed multiplier [0.1-1.0], default 1.0
bool wait_for_completion              # Block until done (default true)

# Priority
int32 priority                         # Higher = more urgent (default 0)
```

### Command Types

| Command Type | Description |
|--------------|-------------|
| `pick_place` | Execute full pick and place sequence |
| `home` | Return to home position |
| `stop` | Emergency stop |
| `calibrate` | Move to calibration position |

### Example Usage

```python
from pepper_msgs.msg import ArmCommand
from geometry_msgs.msg import Point

cmd = ArmCommand()
cmd.header.stamp = node.get_clock().now().to_msg()

cmd.pickup_position = Point(x=0.08, y=0.05, z=0.01)
cmd.place_position = Point(x=0.0, y=0.25, z=0.0)

cmd.pepper_id = 1001
cmd.sorting_category = "fresh_red"
cmd.command_type = "pick_place"
cmd.speed_scale = 0.8  # 80% speed for safety

publisher.publish(cmd)
```

---

## 4. ArmStatus.msg

**Purpose**: Current status of robot arm

```python
# Header with timestamp
std_msgs/Header header

# Current state
string state                           # "idle", "moving", "picking", "placing", "error"

# Current position (world frame)
geometry_msgs/Point current_position

# Joint states (angles in degrees)
float32[] joint_angles                 # Current servo angles
float32[] joint_targets                # Target servo angles

# Gripper state
bool gripper_closed                    # True if gripper is closed
float32 gripper_angle                  # Current gripper servo angle

# Task tracking
int32 current_pepper_id                # ID of pepper being handled (0 if none)
string current_task                    # Description of current task

# Error handling
string error_message                   # Empty if no error
int32 error_code                       # 0 = no error

# Performance metrics
float32 task_completion_time           # Time taken for last task (seconds)
int32 total_picks_completed            # Lifetime pick count
int32 total_failures                   # Lifetime failure count

# Hardware status
float32 battery_voltage                # If using battery (volts)
bool serial_connected                  # Arduino connection status
```

### State Machine Values

| State | Description |
|-------|-------------|
| `idle` | Ready for new command |
| `moving` | Moving to target position |
| `picking` | Executing pick sequence |
| `placing` | Executing place sequence |
| `homing` | Returning to home position |
| `error` | Error occurred, needs intervention |

### Example Usage

```python
from pepper_msgs.msg import ArmStatus

status = ArmStatus()
status.header.stamp = node.get_clock().now().to_msg()
status.header.frame_id = "arm_left_base"

status.state = "picking"
status.current_position = Point(x=0.08, y=0.05, z=0.01)
status.joint_angles = [90.0, 45.0, 120.0, 60.0, 30.0]
status.gripper_closed = False
status.current_pepper_id = 1001
status.serial_connected = True
status.error_code = 0

publisher.publish(status)
```

---

## 5. Optional: Statistics Message

**Purpose**: Performance metrics for monitoring

```python
# PlannerStatistics.msg (optional)
std_msgs/Header header

# Throughput
int32 peppers_sorted_total
int32 peppers_sorted_last_minute
float32 average_cycle_time             # seconds per pepper

# Success rates
float32 pick_success_rate              # [0.0-1.0]
float32 place_success_rate

# Arm utilization
float32 left_arm_utilization           # [0.0-1.0] (busy time / total time)
float32 right_arm_utilization

# Queue status
int32 pending_tasks
int32 completed_tasks
int32 failed_tasks

# Classification breakdown
int32 count_red
int32 count_green
int32 count_yellow
int32 count_fresh
int32 count_rotten
```

---

## Message Dependencies

### package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>pepper_msgs</name>
  <version>0.1.0</version>
  <description>Custom messages for pepper sorting robot</description>
  <maintainer email="jay@example.com">jay</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(pepper_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedPepper.msg"
  "msg/PepperArray.msg"
  "msg/ArmCommand.msg"
  "msg/ArmStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

---

## Building the Message Package

```bash
cd ~/pepper_sorting_ws

# Build message package first (other packages depend on it)
colcon build --packages-select pepper_msgs

# Source the workspace
source install/setup.bash

# Verify messages are available
ros2 interface list | grep pepper_msgs
```

Expected output:
```
pepper_msgs/msg/ArmCommand
pepper_msgs/msg/ArmStatus
pepper_msgs/msg/DetectedPepper
pepper_msgs/msg/PepperArray
```

---

## Using Messages in Python Nodes

```python
# Import custom messages
from pepper_msgs.msg import DetectedPepper, PepperArray, ArmCommand, ArmStatus

# In your node
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Publisher
        self.pub = self.create_publisher(PepperArray, '/detected_peppers', 10)

        # Subscriber
        self.sub = self.create_subscription(
            ArmStatus,
            '/arm/left/status',
            self.status_callback,
            10
        )

    def status_callback(self, msg: ArmStatus):
        self.get_logger().info(f'Arm state: {msg.state}')
```

---

## Message Testing

```bash
# Echo a topic with custom message
ros2 topic echo /detected_peppers

# Publish manually (for testing)
ros2 topic pub /arm/left/command pepper_msgs/ArmCommand "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
  pickup_position: {x: 0.1, y: 0.05, z: 0.01},
  place_position: {x: 0.0, y: 0.25, z: 0.0},
  pepper_id: 999,
  sorting_category: 'test',
  command_type: 'pick_place'
}"
```

---

**Custom Messages Complete ✓**
