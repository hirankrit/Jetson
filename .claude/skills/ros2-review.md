# ROS2 Code Review Skill

เมื่อทำ code review สำหรับโปรเจค ROS2 ให้ตรวจสอบหัวข้อเหล่านี้:

## 1. ROS2 Best Practices
- ✅ ใช้ rclpy/rclcpp API ถูกต้อง
- ✅ Node naming convention (lowercase with underscores)
- ✅ Topic/Service naming ตามมาตรฐาน ROS2
- ✅ ใช้ QoS profiles ที่เหมาะสม
- ✅ Proper shutdown handling (destroy_node, shutdown)

## 2. Python Code Quality (ถ้าเป็น Python)

### 2.1 PEP 8 Style Guide
- ✅ **Indentation**: ใช้ 4 spaces (ไม่ใช่ tabs)
- ✅ **Line length**: ไม่เกิน 79 ตัวอักษร (docstrings ไม่เกิน 72)
- ✅ **Naming conventions**:
  - `snake_case` สำหรับ functions, variables, methods
  - `PascalCase` สำหรับ Classes
  - `UPPER_CASE` สำหรับ constants
  - `_leading_underscore` สำหรับ private methods/variables
- ✅ **Imports**:
  - อยู่ด้านบนสุดของไฟล์
  - เรียงตาม: standard library → third-party → local
  - แยกบรรทัดสำหรับแต่ละ import
  - ตัวอย่าง:
    ```python
    import os
    import sys

    import rclpy
    from rclpy.node import Node

    from my_package.msg import CustomMsg
    ```
- ✅ **Whitespace**:
  - เว้นวรรค 2 บรรทัดก่อน class definition
  - เว้นวรรค 1 บรรทัดระหว่าง methods
  - ไม่มีช่องว่างก่อน `:`, `,`, `;`

### 2.2 Type Hints
- ✅ **ใส่ type hints สำหรับ**:
  - Function parameters
  - Function return types
  - Class attributes
- ✅ **ตัวอย่างที่ถูกต้อง**:
  ```python
  from typing import List, Optional, Tuple

  def process_image(
      self,
      image: np.ndarray,
      threshold: float = 0.5
  ) -> Tuple[List[Detection], bool]:
      """Process image and return detections."""
      pass

  class VisionNode(Node):
      _camera_info: Optional[CameraInfo] = None
      _detections: List[Detection] = []
  ```
- ✅ **ใช้ Optional[] สำหรับค่าที่อาจเป็น None**
- ✅ **ใช้ List[], Dict[], Tuple[] จาก typing module**

### 2.3 Docstrings (Google Style)
- ✅ **ทุก class และ public method ต้องมี docstring**
- ✅ **รูปแบบ Google Style**:
  ```python
  def calculate_depth(
      self,
      disparity: np.ndarray,
      baseline: float,
      focal_length: float
  ) -> np.ndarray:
      """คำนวณ depth map จาก disparity.

      ฟังก์ชันนี้แปลง disparity map เป็น depth map โดยใช้สูตร
      depth = (baseline * focal_length) / disparity

      Args:
          disparity: Disparity map จากกล้อง stereo (numpy array)
          baseline: ระยะห่างระหว่าง 2 กล้อง (mm)
          focal_length: ค่า focal length ของกล้อง (pixels)

      Returns:
          Depth map เป็น numpy array หน่วยเป็น mm

      Raises:
          ValueError: ถ้า disparity มีค่าเป็น 0 หรือติดลบ

      Example:
          >>> depth = self.calculate_depth(disp, 60.0, 700.0)
      """
      if np.any(disparity <= 0):
          raise ValueError("Disparity must be positive")
      return (baseline * focal_length) / disparity
  ```
- ✅ **Class docstring**:
  ```python
  class CameraNode(Node):
      """Node สำหรับจัดการกล้อง stereo และ publish images.

      Node นี้รับภาพจากกล้อง stereo 2 ตัว ทำ rectification
      และ publish เป็น ROS2 messages

      Attributes:
          camera_info: ข้อมูล calibration ของกล้อง
          left_pub: Publisher สำหรับภาพซ้าย
          right_pub: Publisher สำหรับภาพขวา

      Topics:
          Publishers:
              - /camera/left/image_raw (sensor_msgs/Image)
              - /camera/right/image_raw (sensor_msgs/Image)
          Subscribers:
              - /camera/control (std_msgs/String)
      """
  ```

### 2.4 Error Handling
- ✅ **ใช้ try-except เฉพาะเจาะจง**:
  ```python
  # ❌ ไม่ดี - จับ exception ทั่วไป
  try:
      result = risky_operation()
  except:
      pass

  # ✅ ดี - ระบุ exception ชัดเจน
  try:
      result = self.load_calibration(path)
  except FileNotFoundError:
      self.get_logger().error(f'Calibration file not found: {path}')
      return None
  except yaml.YAMLError as e:
      self.get_logger().error(f'Invalid YAML format: {e}')
      return None
  ```
- ✅ **Log errors ด้วย ROS2 logger**:
  ```python
  self.get_logger().error('Error message')
  self.get_logger().warn('Warning message')
  self.get_logger().info('Info message')
  self.get_logger().debug('Debug message')
  ```
- ✅ **ไม่ silence errors** (ไม่ใช้ `except: pass`)
- ✅ **Cleanup resources** ใน finally block:
  ```python
  try:
      file = open('data.txt', 'r')
      data = file.read()
  except IOError as e:
      self.get_logger().error(f'Cannot read file: {e}')
  finally:
      file.close()
  ```

### 2.5 Code Structure
- ✅ **Function length**: ไม่ควรเกิน 50 บรรทัด (ยิ่งสั้นยิ่งดี)
- ✅ **Single Responsibility**: 1 function ทำ 1 งาน
- ✅ **Avoid magic numbers**: ใช้ constants แทน
  ```python
  # ❌ ไม่ดี
  if depth > 500:
      return True

  # ✅ ดี
  MAX_DEPTH_MM = 500
  if depth > MAX_DEPTH_MM:
      return True
  ```
- ✅ **Use list comprehensions** (เมื่อเหมาะสม):
  ```python
  # ✅ ดี - อ่านง่าย
  valid_depths = [d for d in depths if d > 0]

  # แต่ไม่ซับซ้อนเกินไป
  # ❌ ไม่ดี - อ่านยาก
  result = [[func(x) for x in row if x > 0] for row in matrix if len(row) > 5]
  ```

### 2.6 ROS2 Python Specific
- ✅ **Proper node initialization**:
  ```python
  class MyNode(Node):
      def __init__(self):
          super().__init__('my_node_name')
          self.get_logger().info('Node initialized')
  ```
- ✅ **Declare parameters**:
  ```python
  self.declare_parameter('update_rate', 30.0)
  rate = self.get_parameter('update_rate').value
  ```
- ✅ **Proper callback signatures**:
  ```python
  def image_callback(self, msg: Image) -> None:
      """Process incoming image message."""
      pass
  ```
- ✅ **Cleanup in destroy**:
  ```python
  def destroy_node(self):
      self.get_logger().info('Shutting down node...')
      # Cleanup resources here
      super().destroy_node()
  ```

## 3. C++ Code Quality (ถ้าเป็น C++)
- ✅ ใช้ smart pointers (shared_ptr, unique_ptr)
- ✅ RAII principles
- ✅ const correctness

## 4. Performance
- ✅ ไม่มี busy waiting loops
- ✅ Timer/Callback frequency เหมาะสม
- ✅ Message size ไม่ใหญ่เกินไป

## 5. Safety
- ✅ Null pointer checks
- ✅ Array bounds checking
- ✅ Thread safety (ถ้ามี multi-threading)

## รูปแบบการรีวิว:
ให้คำติชมเป็นภาษาไทย แบ่งเป็น:
- ✅ **ดีแล้ว**: สิ่งที่ทำได้ดี
- ⚠️ **ควรปรับปรุง**: ข้อเสนอแนะ
- ❌ **ต้องแก้ไข**: ปัญหาที่ต้องแก้ก่อน merge
