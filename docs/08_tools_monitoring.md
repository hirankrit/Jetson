# 8. Tools & Monitoring

## ROS2 Command-Line Tools

### Essential Commands

```bash
# Node management
ros2 node list                    # List all running nodes
ros2 node info /vision_node       # Show node details

# Topic management
ros2 topic list                   # List all topics
ros2 topic info /detected_peppers # Show topic details
ros2 topic echo /detected_peppers # Print messages
ros2 topic hz /camera/left/image_raw  # Check publishing rate

# Parameter management
ros2 param list /vision_node      # List node parameters
ros2 param get /vision_node confidence_threshold
ros2 param set /vision_node confidence_threshold 0.7
ros2 param dump /vision_node > params.yaml  # Save current params

# Service management
ros2 service list                 # List available services
ros2 service call /emergency_stop std_srvs/srv/Trigger

# Launch
ros2 launch pepper_bringup system.launch.py
```

---

## RViz2 Visualization

### Setup RViz2

```bash
# Install if not already
sudo apt install ros-humble-rviz2

# Launch RViz2
ros2 run rviz2 rviz2

# Or with config
ros2 run rviz2 rviz2 -d config/rviz_config.rviz
```

### Recommended RViz2 Configuration

**Displays to add:**

1. **Camera Image**
   - Add → By display type → Image
   - Topic: `/camera/left/image_raw` or `/vision/debug_image`
   - Shows live camera feed with detection overlays

2. **TF (Transform Frames)**
   - Add → TF
   - Shows all coordinate frames
   - Verify camera, arm_left_base, arm_right_base frames

3. **Markers (Detected Peppers)**
   - Add → Marker
   - Topic: `/detected_peppers_viz` (custom visualization topic)
   - Shows detected peppers as colored spheres in 3D space

4. **Robot Model (if URDF available)**
   - Add → RobotModel
   - Shows arm kinematic chain

**Fixed Frame:** Set to `world`

**Save Config:**
- File → Save Config As → `pepper_bringup/config/rviz_config.rviz`

---

## RQT Tools

### 1. rqt_graph - Node Graph Visualization

```bash
ros2 run rqt_graph rqt_graph
```

**Purpose:** Visualize node connections and topic flow
- Shows all nodes as boxes
- Shows topics as arrows
- Helps debug communication issues

**Usage:**
- Refresh to see current graph
- Uncheck "Debug" to hide hidden topics

---

### 2. rqt_plot - Real-time Data Plotting

```bash
ros2 run rqt_plot rqt_plot
```

**Purpose:** Plot numeric data over time

**Example plots:**
- `/detected_peppers/total_count` - Number of peppers detected
- `/arm/left/status/joint_angles[0]` - Joint 1 angle over time
- `/planner/statistics/throughput` - Peppers per minute

**Usage:**
- Add topic by typing in field
- Click "+" to add to plot
- Adjust time window (default 60s)

---

### 3. rqt_console - Log Viewer

```bash
ros2 run rqt_console rqt_console
```

**Purpose:** View and filter ROS2 logs

**Features:**
- Filter by severity (DEBUG, INFO, WARN, ERROR, FATAL)
- Filter by node name
- Search log messages
- Highlight specific messages

**Log Levels:**
- **DEBUG**: Detailed information for debugging
- **INFO**: Normal operation messages
- **WARN**: Warning, not critical
- **ERROR**: Error occurred, but system continues
- **FATAL**: Critical error, system may stop

---

### 4. rqt_reconfigure - Dynamic Parameter Tuning

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

**Purpose:** Adjust parameters in real-time without restarting nodes

**Example use cases:**
- Tune `confidence_threshold` while watching detections
- Adjust `speed_scale` to slow down arm movement
- Change `priority_mode` to test different strategies

---

### 5. rqt_image_view - Image Viewer

```bash
ros2 run rqt_image_view rqt_image_view
```

**Purpose:** View camera images and debug visualizations

**Topics to view:**
- `/camera/left/image_raw` - Raw camera feed
- `/vision/debug_image` - Detections with bounding boxes
- `/vision/depth_map` - Depth visualization

---

## Rosbag - Data Recording & Playback

### Recording Data

```bash
# Record all topics
ros2 bag record -a -o pepper_session_001

# Record specific topics
ros2 bag record /camera/left/image_raw /camera/right/image_raw /detected_peppers -o vision_test

# Record with compression (saves space)
ros2 bag record -a --compression-mode file --compression-format zstd -o compressed_bag
```

### Playback Data

```bash
# Play bag file
ros2 bag play pepper_session_001

# Play at half speed (for debugging)
ros2 bag play pepper_session_001 --rate 0.5

# Play in loop
ros2 bag play pepper_session_001 --loop

# Play and remap topics (if needed)
ros2 bag play pepper_session_001 --remap /old_topic:=/new_topic
```

### Inspect Bag File

```bash
# Show bag info
ros2 bag info pepper_session_001

# Output example:
# Duration: 120.5s
# Topics: 5
#   /camera/left/image_raw (sensor_msgs/msg/Image): 3600 messages
#   /detected_peppers (pepper_msgs/msg/PepperArray): 1200 messages
```

**Use cases:**
- Debug issues without hardware
- Collect data for later analysis
- Test algorithm improvements offline

---

## Performance Monitoring

### Custom Statistics Node

Create a monitoring node that tracks system performance:

```python
# statistics_monitor_node.py
import rclpy
from rclpy.node import Node
from pepper_msgs.msg import ArmStatus, PepperArray
import time

class StatisticsMonitor(Node):
    def __init__(self):
        super().__init__('statistics_monitor')

        self.peppers_detected = 0
        self.picks_attempted = 0
        self.picks_succeeded = 0
        self.start_time = time.time()

        self.sub_peppers = self.create_subscription(
            PepperArray, '/detected_peppers', self.peppers_callback, 10
        )
        self.sub_left = self.create_subscription(
            ArmStatus, '/arm/left/status', self.left_status_callback, 10
        )
        self.sub_right = self.create_subscription(
            ArmStatus, '/arm/right/status', self.right_status_callback, 10
        )

        # Timer to print stats every 60 seconds
        self.timer = self.create_timer(60.0, self.print_statistics)

    def peppers_callback(self, msg):
        self.peppers_detected = msg.total_count

    def left_status_callback(self, msg):
        if msg.state == 'idle' and msg.current_pepper_id != 0:
            self.picks_attempted += 1
            if msg.error_code == 0:
                self.picks_succeeded += 1

    def print_statistics(self):
        elapsed = time.time() - self.start_time
        throughput = self.picks_succeeded / (elapsed / 60.0)  # per minute
        success_rate = self.picks_succeeded / max(self.picks_attempted, 1) * 100

        self.get_logger().info(f'''
        === Statistics ===
        Elapsed time: {elapsed:.1f}s
        Peppers detected: {self.peppers_detected}
        Picks attempted: {self.picks_attempted}
        Picks succeeded: {self.picks_succeeded}
        Success rate: {success_rate:.1f}%
        Throughput: {throughput:.2f} peppers/min
        ''')

def main(args=None):
    rclpy.init(args=args)
    node = StatisticsMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Run:**
```bash
ros2 run pepper_planning statistics_monitor_node
```

---

## System Health Checks

### Check Node Status

```bash
#!/bin/bash
# check_system.sh - Verify all nodes are running

echo "Checking system health..."

REQUIRED_NODES=(
  "/camera_node"
  "/vision_node"
  "/task_planner_node"
  "/arm_left_node"
  "/arm_right_node"
)

for node in "${REQUIRED_NODES[@]}"; do
  if ros2 node list | grep -q "$node"; then
    echo "✓ $node is running"
  else
    echo "✗ $node is NOT running"
  fi
done

echo ""
echo "Topic rates:"
ros2 topic hz /camera/left/image_raw --once
ros2 topic hz /detected_peppers --once
```

---

## Debugging Techniques

### 1. Check Topic Publishing

```bash
# Is topic publishing?
ros2 topic hz /detected_peppers

# Expected output: average rate: 10.0
# If no output → node not publishing
```

### 2. Inspect Message Content

```bash
# See actual message content
ros2 topic echo /detected_peppers --once

# Verify values are reasonable (not NaN, within expected range)
```

### 3. Check TF Transforms

```bash
# List all transforms
ros2 run tf2_ros tf2_echo world camera_link

# Verify transformation is correct
```

### 4. Enable Debug Logging

In your node:
```python
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

Or via command line:
```bash
ros2 run pepper_vision vision_node --ros-args --log-level DEBUG
```

### 5. Use Python Debugger (pdb)

Add breakpoint in code:
```python
import pdb; pdb.set_trace()
```

Run node normally, it will pause at breakpoint:
```bash
ros2 run pepper_vision vision_node
```

---

## Jetson-Specific Monitoring

### GPU Usage

```bash
# Install jtop
sudo pip3 install -U jetson-stats

# Monitor GPU/CPU/RAM usage
sudo jtop
```

**Key metrics:**
- GPU utilization (should be 40-60% during vision processing)
- RAM usage (8GB total on Orin Nano)
- Power consumption
- Temperature (keep < 80°C)

### Model Performance

```bash
# Benchmark YOLO inference speed
python3 benchmark_yolo.py

# Check FPS
# Target: >15 FPS on Jetson Orin Nano with YOLOv8n
```

---

## Troubleshooting Workflow

1. **System won't start**
   - Check `ros2 node list` → Are all nodes running?
   - Check logs: `ros2 run rqt_console rqt_console`
   - Verify hardware connections (camera, Arduino USB)

2. **Detection not working**
   - Check camera feed: `ros2 run rqt_image_view rqt_image_view`
   - Check model confidence: lower threshold temporarily
   - Verify lighting conditions

3. **Arm not moving**
   - Check serial connection: `ls /dev/ttyUSB*`
   - Test Arduino independently (Serial Monitor)
   - Check IK solver (add debug prints)

4. **Low throughput**
   - Profile vision node (check FPS)
   - Increase arm speed (speed_scale parameter)
   - Optimize task planner (reduce idle time)

5. **High failure rate**
   - Check calibration (hand-eye, stereo)
   - Verify depth accuracy (compare with ruler)
   - Tune gripper timing (approach height, close delay)

---

**Tools & Monitoring Complete ✓**
