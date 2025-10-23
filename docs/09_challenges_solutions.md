# 9. Challenges & Solutions

## Common Issues and How to Solve Them

---

## 1. Vision System Challenges

### Challenge: Poor Detection Accuracy

**Symptoms:**
- YOLO misses peppers (false negatives)
- Detects non-peppers (false positives)
- Confidence scores are low (<0.5)

**Solutions:**

1. **Improve Dataset Quality**
   - Collect more diverse images (500-1000+)
   - Include edge cases (overlapping peppers, shadows, reflections)
   - Balance classes (equal number of red/green/yellow)
   - Use data augmentation (rotation, brightness, blur)

2. **Adjust Model**
   - Try larger YOLO model (yolov8s instead of yolov8n)
   - Train longer (increase epochs from 100 to 200)
   - Tune hyperparameters (learning rate, batch size)

3. **Optimize Lighting**
   - Add consistent LED lighting (5000-6500K)
   - Diffuse light to reduce harsh shadows
   - Avoid reflections on peppers

4. **Lower Confidence Threshold**
   - Temporarily set `confidence_threshold: 0.5` (instead of 0.6)
   - Trade-off: more false positives, but fewer misses

**Prevention:**
- Test model on validation set before deployment
- Collect data under actual operating conditions
- Regularly retrain with new data

---

### Challenge: Inaccurate Depth Estimation

**Symptoms:**
- Z-coordinate (depth) is off by >2cm
- Disparity map is noisy or blank
- Arm reaches wrong height

**Solutions:**

1. **Re-do Stereo Calibration**
   - Use high-quality checkerboard (printed on flat surface)
   - Collect 30+ calibration images at various angles
   - Verify reprojection error <0.5 pixels
   - Ensure camera lenses are clean

2. **Tune Stereo Matching Parameters**
   ```yaml
   num_disparities: 128  # Increase if objects are close
   block_size: 11        # Decrease for finer detail (odd number)
   uniqueness_ratio: 10  # Increase to reduce noise
   ```

3. **Improve Image Quality**
   - Ensure good lighting (avoid dark spots)
   - Add texture to workspace background (avoid plain white/black)
   - Check for camera synchronization issues

4. **Filter Invalid Depth**
   ```python
   # In vision_node.py
   if depth < self.depth_min or depth > self.depth_max:
       continue  # Skip this detection
   ```

**Prevention:**
- Periodic calibration checks (monthly)
- Use fiducial markers to verify depth accuracy
- Log depth measurements for analysis

---

### Challenge: Color Misclassification

**Symptoms:**
- Red peppers classified as green (or vice versa)
- Accuracy <80%

**Solutions:**

1. **Improve Color Classification Logic**
   ```python
   def classify_color(self, image, bbox):
       # Extract ROI
       roi = image[bbox[1]:bbox[3], bbox[0]:bbox[2]]

       # Convert to HSV (better for color analysis)
       hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

       # Define color ranges
       red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
       green_mask = cv2.inRange(hsv, (40, 100, 100), (80, 255, 255))
       yellow_mask = cv2.inRange(hsv, (20, 100, 100), (40, 255, 255))

       # Count pixels
       red_count = cv2.countNonZero(red_mask)
       green_count = cv2.countNonZero(green_mask)
       yellow_count = cv2.countNonZero(yellow_mask)

       # Return color with max count
       counts = {'red': red_count, 'green': green_count, 'yellow': yellow_count}
       return max(counts, key=counts.get)
   ```

2. **Train Separate Classification Model**
   - Use YOLO for detection only
   - Train small CNN for color classification
   - More accurate than heuristic methods

3. **Control Lighting**
   - Use consistent color temperature (6500K daylight)
   - Avoid colored lighting (warm/yellow bulbs)

---

## 2. Calibration Challenges

### Challenge: Hand-Eye Calibration Drifts

**Symptoms:**
- Arm reaches 2-5cm away from target
- Error increases over time
- Calibration was accurate initially

**Solutions:**

1. **Re-calibrate Periodically**
   - Perform calibration weekly during development
   - Monthly in production

2. **Check for Mechanical Issues**
   - Verify camera mount is rigid (no wobbling)
   - Check for loose servo screws on arm
   - Ensure workspace doesn't shift

3. **Use More Calibration Points**
   - Increase from 10 to 20+ calibration positions
   - Cover entire workspace uniformly
   - Include positions at different heights

4. **Improve Calibration Accuracy**
   - Use precise calibration target (e.g., metal pin on checkerboard)
   - Measure target position with ruler for verification
   - Repeat calibration 3 times, use average

**Prevention:**
- Mark camera and arm base positions on workspace
- Use bolts/clamps instead of tape for mounting
- Add calibration check to daily startup routine

---

### Challenge: Stereo Calibration Reprojection Error Too High

**Symptoms:**
- Reprojection error >1.0 pixels
- Depth map is noisy
- 3D positions are inaccurate

**Solutions:**

1. **Improve Calibration Procedure**
   - Ensure checkerboard is flat (glue to rigid board)
   - Hold checkerboard still during capture (no motion blur)
   - Capture at various angles (tilt, rotate)
   - Cover entire field of view

2. **Check Camera Hardware**
   - Clean camera lenses
   - Ensure cameras are firmly attached (no wobbling)
   - Verify CSI cable connections

3. **Use Larger Checkerboard**
   - Bigger squares (30mm instead of 25mm)
   - More corners (10x7 instead of 9x6)
   - Better detection accuracy

4. **Increase Calibration Images**
   - Collect 40-50 images (instead of 20)
   - More data = better calibration

---

## 3. Arm Control Challenges

### Challenge: Inverse Kinematics Fails

**Symptoms:**
- IK returns None (no solution)
- Arm cannot reach target
- Console shows "IK failed" errors

**Solutions:**

1. **Check Workspace Limits**
   ```python
   # In arm_controller_node.py
   def is_reachable(self, position):
       # Check if position is within arm reach
       distance = np.linalg.norm(position)
       return distance < self.max_reach
   ```

2. **Implement Numerical IK Solver**
   ```python
   from scipy.optimize import minimize

   def ik_numerical(self, target_pos):
       # Use numerical optimization
       result = minimize(
           lambda angles: np.linalg.norm(self.forward_kinematics(angles) - target_pos),
           x0=self.current_angles,
           method='SLSQP',
           bounds=self.joint_limits
       )
       if result.success:
           return result.x
       return None
   ```

3. **Adjust Target Position**
   - Add offset to bring target closer: `target.z += 0.02`
   - Use "approach from above" strategy

4. **Increase Joint Limits**
   - If safe, expand servo range (check mechanically)

**Prevention:**
- Test IK solver with known positions before deployment
- Add reachability check in task planner
- Visualize workspace boundary in RViz

---

### Challenge: Gripper Fails to Pick Pepper

**Symptoms:**
- Gripper closes but pepper drops
- Success rate <60%

**Solutions:**

1. **Tune Pick Sequence**
   ```python
   def pick(self, position):
       # 1. Approach from higher up
       above = position.copy()
       above.z += 0.08  # 8cm above (increase if needed)
       self.move_to(above, gripper_open=True)

       # 2. Lower slowly
       self.move_to(position, gripper_open=True, speed=0.5)

       # 3. Wait for stabilization
       time.sleep(0.3)

       # 4. Close gripper
       self.gripper_close()

       # 5. Wait for gripper to fully close
       time.sleep(0.5)  # Increase if needed

       # 6. Lift up slowly
       self.move_to(above, gripper_open=False, speed=0.6)
   ```

2. **Improve Gripper Design**
   - Add rubber/foam padding to gripper fingers
   - Adjust gripper closing angle (not too wide, not too tight)
   - Test with different pepper sizes

3. **Verify Depth Accuracy**
   - Place pepper at known position, check detected Z
   - Adjust position by measured error: `position.z += error_offset`

4. **Add Grasp Detection**
   - Use servo current sensing (if available)
   - Detect if gripper closed on object or just closed empty

**Prevention:**
- Test gripper with variety of pepper sizes
- Periodic maintenance (check servo torque, gripper alignment)

---

## 4. Coordination Challenges

### Challenge: Both Arms Try to Pick Same Pepper

**Symptoms:**
- Collision or interference
- Double-picking same pepper

**Solutions:**

1. **Improve Task Tracking**
   ```python
   # In task_planner_node.py
   self.assigned_peppers = {}  # {pepper_id: arm_name}

   def assign_pepper(self, pepper, arm):
       if pepper.id in self.assigned_peppers:
           return False  # Already assigned
       self.assigned_peppers[pepper.id] = arm
       return True
   ```

2. **Add Mutex Lock**
   - Before assigning, check if pepper is already assigned
   - Use ROS2 services for atomic operations (optional)

3. **Workspace Division**
   - Strictly enforce left/right zones
   - Add "no-go" overlap zone

---

### Challenge: Arms Collide

**Symptoms:**
- Physical collision between arms
- System error/emergency stop

**Solutions:**

1. **Increase Safety Margin**
   ```yaml
   workspace_config.yaml:
     collision_margin: 0.10  # 10cm safety zone
   ```

2. **Collision Detection**
   ```python
   def check_collision(self, left_pos, right_pos):
       distance = np.linalg.norm(left_pos - right_pos)
       return distance < self.collision_margin
   ```

3. **Sequential Picking in Overlap Zone**
   - If both arms need to enter center zone, queue operations
   - Wait for first arm to finish before allowing second

4. **Emergency Stop**
   - Add hardware E-stop button
   - Monitor arm positions, auto-stop if too close

---

## 5. System Integration Challenges

### Challenge: ROS2 Topics Not Updating

**Symptoms:**
- `ros2 topic hz` shows 0 Hz
- Nodes running but no communication

**Solutions:**

1. **Check QoS Settings**
   ```python
   # Ensure publisher and subscriber use compatible QoS
   from rclpy.qos import QoSProfile, ReliabilityPolicy

   qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
   self.publisher = self.create_publisher(Image, '/camera/left/image_raw', qos)
   ```

2. **Verify Topic Names**
   - Typo in topic name? `/detected_pepper` vs `/detected_peppers`
   - Use `ros2 topic list` to see actual topic names

3. **Check Node Lifecycle**
   - Is node stuck in initialization?
   - Add debug prints: `self.get_logger().info('Publishing...')`

---

### Challenge: System Slow / Low FPS

**Symptoms:**
- Camera feed <15 FPS
- High latency (>500ms)

**Solutions:**

1. **Profile Nodes**
   ```python
   import time
   start = time.time()
   # ... processing ...
   elapsed = time.time() - start
   self.get_logger().info(f'Processing took {elapsed:.3f}s')
   ```

2. **Optimize Vision Node**
   - Use smaller YOLO model (yolov8n instead of yolov8s)
   - Reduce camera resolution (640x480 instead of 1280x720)
   - Lower processing rate (5Hz instead of 10Hz)
   - Use TensorRT for 2-3× speedup

3. **Check Jetson Performance**
   - Use `sudo jtop` to monitor GPU usage
   - Ensure Jetson is in MAX performance mode:
   ```bash
   sudo nvpmodel -m 0  # Max performance mode
   sudo jetson_clocks   # Max clock speeds
   ```

4. **Reduce Logging**
   - Too many `get_logger().info()` calls slow down system
   - Use DEBUG level for verbose logs

---

## 6. Hardware Challenges

### Challenge: Servo Jitter / Unstable Movement

**Symptoms:**
- Servos vibrate or jitter
- Erratic movement

**Solutions:**

1. **Add Capacitor to Power Supply**
   - 1000µF capacitor across servo power lines
   - Reduces voltage spikes

2. **Smooth Motion**
   ```cpp
   // In Arduino firmware
   void updateServos() {
       for (int i = 0; i < 6; i++) {
           if (currentAngles[i] < targetAngles[i]) {
               currentAngles[i]++;  // Increment by 1 degree
           } else if (currentAngles[i] > targetAngles[i]) {
               currentAngles[i]--;
           }
           servos[i].write(currentAngles[i]);
       }
       delay(15);  // Smooth motion
   }
   ```

3. **Check Power Supply**
   - Servos need 5V/3A+ supply
   - Use separate supply from Jetson

---

### Challenge: Arduino Serial Connection Drops

**Symptoms:**
- Arm stops responding
- Serial timeout errors

**Solutions:**

1. **Auto-Reconnect Logic**
   ```python
   def send_command(self, cmd):
       if not self.serial.is_open:
           self.reconnect()
       self.serial.write(cmd.encode())
   ```

2. **Use Quality USB Cable**
   - Short cable (<1m)
   - Avoid cheap cables

3. **Add Watchdog on Arduino**
   ```cpp
   unsigned long lastCommandTime = 0;
   void loop() {
       if (millis() - lastCommandTime > 5000) {
           // No command for 5s, go to safe state
           goHome();
       }
   }
   ```

---

## Summary: Troubleshooting Checklist

When things go wrong, follow this checklist:

1. **Check Hardware**
   - [ ] All cables connected?
   - [ ] Power supplies on?
   - [ ] Servos responding?
   - [ ] Camera feed visible?

2. **Check ROS2**
   - [ ] All nodes running? `ros2 node list`
   - [ ] Topics publishing? `ros2 topic hz <topic>`
   - [ ] Check logs: `ros2 run rqt_console rqt_console`

3. **Check Calibration**
   - [ ] Stereo calibration recent? (<1 month)
   - [ ] Hand-eye calibration accurate? (test with known position)

4. **Check Parameters**
   - [ ] Confidence threshold too high?
   - [ ] Speed too fast?
   - [ ] Workspace limits correct?

5. **Check Code**
   - [ ] Add debug prints
   - [ ] Use Python debugger (pdb)
   - [ ] Review logs for errors

---

**Challenges & Solutions Complete ✓**
