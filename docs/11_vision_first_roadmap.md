# 11. Vision-First Development Roadmap

## แนวทางใหม่: เน้น Vision System ก่อน

เนื่องจากคุณมีพื้นฐานความรู้อยู่แล้ว เราจะเริ่มจาก **Vision System** เลย เพื่อให้มี **output รายงาน** ได้เร็วที่สุด โดยยังไม่ต้องใช้ ROS2 และ Robot Arms ในระยะแรก

---

## ภาพรวมแผนใหม่

```
Phase 1: Vision System (Week 1-4)
    → Output: Detection model + 3D positioning
    → Report: รายงานความแม่นยำ

Phase 2: ROS2 Integration (Week 5-6)
    → Output: ROS2 vision nodes
    → Report: Performance metrics

Phase 3: Single Arm (Week 7-8)
    → Output: Pick & place working

Phase 4: Dual Arms (Week 9-10)
    → Output: Complete system

Phase 5: Optimization (Week 11-12)
    → Output: Final report
```

---

## Phase 1: Vision System (Week 1-4)

### Week 1: Camera Setup & Stereo Calibration

**Goal**: ได้ stereo depth map ที่ดี
**Output**: รายงานผล calibration + ตัวอย่าง depth map

#### Day 1: Hardware Setup ✅ เสร็จแล้ว!
- [x] เชื่อม IMX219 Stereo Camera กับ Jetson
- [x] Enable IMX219 ใน device tree (merge_imx219_dtb.sh)
- [x] ติดตั้ง GStreamer และ ROS2 Humble
- [x] สร้าง gstreamer_camera_node.py (ROS2 node)
- [x] สร้าง view_camera.py (Python viewer)
- [x] ทดสอบ capture ภาพจาก 2 กล้อง @ 30 fps ✓
- [ ] ติดตั้ง camera mount (ความสูง 50-80cm)
- [ ] Setup lighting (LED panels หรือ natural light)

**Camera Testing:**
```bash
# Quick test with viewer (ใช้แล้ว ✓)
python3 view_camera.py

# Test with ROS2 node (ใช้แล้ว ✓)
source /opt/ros/humble/setup.bash
python3 gstreamer_camera_node.py

# ผลการทดสอบ:
# ✓ กล้องทั้ง 2 ตัวทำงานปกติ @ 30 fps
# ✓ Resolution: 1280x720 (default), รองรับถึง 3280x2464
# ✓ ROS2 topics: /stereo/left/image_raw, /stereo/right/image_raw
```

#### Day 2-3: Stereo Calibration

**Checklist**:
- [ ] พิมพ์ checkerboard pattern (9×6 corners, 25mm squares)
- [ ] ติด checkerboard บนแผ่นแข็ง (foam board/acrylic)
- [ ] Collect 30-40 calibration images (หลายมุม, หลายระยะ)

```python
# stereo_calibration.py
import cv2
import numpy as np
import glob
import yaml

# Parameters
CHECKERBOARD = (9, 6)
SQUARE_SIZE = 25  # mm

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []
imgpoints_left = []
imgpoints_right = []

# Load calibration images
images_left = sorted(glob.glob('calib_left/*.jpg'))
images_right = sorted(glob.glob('calib_right/*.jpg'))

print(f"Found {len(images_left)} calibration image pairs")

for img_left_path, img_right_path in zip(images_left, images_right):
    img_left = cv2.imread(img_left_path)
    img_right = cv2.imread(img_right_path)

    gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

    ret_left, corners_left = cv2.findChessboardCorners(gray_left, CHECKERBOARD)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, CHECKERBOARD)

    if ret_left and ret_right:
        objpoints.append(objp)

        # Refine corners
        corners_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1),
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        corners_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1),
                                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        imgpoints_left.append(corners_left)
        imgpoints_right.append(corners_right)

        print(f"✓ {img_left_path}")

print(f"\nUsing {len(objpoints)} image pairs for calibration")

# Calibrate left camera
print("Calibrating left camera...")
ret_left, K_left, D_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
    objpoints, imgpoints_left, gray_left.shape[::-1], None, None
)
print(f"Left camera RMS error: {ret_left:.4f}")

# Calibrate right camera
print("Calibrating right camera...")
ret_right, K_right, D_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
    objpoints, imgpoints_right, gray_right.shape[::-1], None, None
)
print(f"Right camera RMS error: {ret_right:.4f}")

# Stereo calibration
print("Performing stereo calibration...")
ret, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    K_left, D_left, K_right, D_right,
    gray_left.shape[::-1],
    flags=cv2.CALIB_FIX_INTRINSIC
)
print(f"Stereo calibration RMS error: {ret:.4f}")

# Stereo rectification
print("Computing rectification...")
R_left, R_right, P_left, P_right, Q, roi_left, roi_right = cv2.stereoRectify(
    K_left, D_left, K_right, D_right,
    gray_left.shape[::-1], R, T,
    alpha=0
)

# Save calibration
calib_data = {
    'K_left': K_left.tolist(),
    'D_left': D_left.tolist(),
    'K_right': K_right.tolist(),
    'D_right': D_right.tolist(),
    'R': R.tolist(),
    'T': T.tolist(),
    'R_left': R_left.tolist(),
    'R_right': R_right.tolist(),
    'P_left': P_left.tolist(),
    'P_right': P_right.tolist(),
    'Q': Q.tolist(),
    'roi_left': roi_left,
    'roi_right': roi_right,
    'image_size': list(gray_left.shape[::-1]),
    'rms_error': float(ret)
}

with open('stereo_calibration.yaml', 'w') as f:
    yaml.dump(calib_data, f)

print("\n✓ Calibration saved to stereo_calibration.yaml")
print(f"Baseline: {abs(T[0][0]):.1f} mm")
```

**Expected Output**:
- RMS error < 0.5 pixels ✓
- Baseline ≈ 60mm ✓

#### Day 4-5: Depth Map Testing

```python
# test_depth_map.py
import cv2
import numpy as np
import yaml

# Load calibration
with open('stereo_calibration.yaml', 'r') as f:
    calib = yaml.safe_load(f)

K_left = np.array(calib['K_left'])
D_left = np.array(calib['D_left'])
K_right = np.array(calib['K_right'])
D_right = np.array(calib['D_right'])
R_left = np.array(calib['R_left'])
R_right = np.array(calib['R_right'])
P_left = np.array(calib['P_left'])
P_right = np.array(calib['P_right'])
Q = np.array(calib['Q'])

# Create rectification maps
img_size = tuple(calib['image_size'])
map_left_x, map_left_y = cv2.initUndistortRectifyMap(
    K_left, D_left, R_left, P_left, img_size, cv2.CV_32FC1
)
map_right_x, map_right_y = cv2.initUndistortRectifyMap(
    K_right, D_right, R_right, P_right, img_size, cv2.CV_32FC1
)

# Stereo matcher
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=128,
    blockSize=11,
    P1=8 * 3 * 11 ** 2,
    P2=32 * 3 * 11 ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

# Open cameras
cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(1)

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not (ret_left and ret_right):
        continue

    # Rectify images
    rect_left = cv2.remap(frame_left, map_left_x, map_left_y, cv2.INTER_LINEAR)
    rect_right = cv2.remap(frame_right, map_right_x, map_right_y, cv2.INTER_LINEAR)

    # Convert to grayscale
    gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

    # Compute disparity
    disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    # Convert to depth
    depth_map = cv2.reprojectImageTo3D(disparity, Q)

    # Visualize
    disparity_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    disparity_color = cv2.applyColorMap(disparity_vis, cv2.COLORMAP_JET)

    # Show results
    cv2.imshow('Left Rectified', rect_left)
    cv2.imshow('Disparity Map', disparity_color)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
```

**✅ Milestone 1: Week 1 Report**
- รายงานผล stereo calibration
- ตัวอย่าง depth map
- วิเคราะห์ความแม่นยำ (วัดกับ object ที่รู้ระยะ)

---

### Week 2: Dataset Collection & Annotation

**Goal**: ได้ dataset พริก 500-1000 ภาพ พร้อม labels
**Output**: Dataset ready for training

#### Day 6-7: Setup Data Collection

```python
# collect_dataset.py
import cv2
import os
from datetime import datetime

output_dir = "pepper_dataset/raw"
os.makedirs(output_dir, exist_ok=True)

cap = cv2.VideoCapture(0)
count = 0

print("Press 'c' to capture, 'q' to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    cv2.imshow('Dataset Collection', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{output_dir}/pepper_{count:04d}_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        count += 1
        print(f"✓ Saved {filename} (Total: {count})")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"\n✓ Collected {count} images")
```

**Data Collection Strategy**:
- วางพริกหลายแบบ:
  - เดี่ยว (1 ผล)
  - กลุ่มเล็ก (3-5 ผล)
  - กองใหญ่ (10+ ผล)
- หลากหลายสี: แดง, เขียว, เหลือง
- หลากหลายคุณภาพ: สด, เน่า, รอยดำ
- หลากหลายท่าวาง: หงาย, คว่ำ, ข้าง
- หลากหลายแสง: เช้า, เที่ยง, เย็น (ถ้าทำได้)

**เป้าหมาย**: 500-1000 ภาพ

#### Day 8-10: Data Annotation

**Option 1: Roboflow (แนะนำ - ง่ายที่สุด)**
1. สร้าง account: https://roboflow.com/
2. Upload images
3. Label แต่ละ pepper (bounding box + class)
4. Classes:
   - `pepper_red_fresh`
   - `pepper_red_rotten`
   - `pepper_green_fresh`
   - `pepper_green_rotten`
   - `pepper_yellow_fresh`
   - `pepper_yellow_rotten`
5. Export → YOLOv8 format

**Option 2: LabelImg (offline)**
```bash
pip install labelImg
labelImg
```

**Dataset Structure**:
```
pepper_dataset/
├── images/
│   ├── train/  (80%)
│   └── val/    (20%)
├── labels/
│   ├── train/
│   └── val/
└── data.yaml
```

**data.yaml**:
```yaml
path: /home/jay/pepper_dataset
train: images/train
val: images/val

nc: 6
names: ['pepper_red_fresh', 'pepper_red_rotten',
        'pepper_green_fresh', 'pepper_green_rotten',
        'pepper_yellow_fresh', 'pepper_yellow_rotten']
```

**✅ Milestone 2: Week 2 Checkpoint**
- Dataset summary report (จำนวนภาพ, distribution)
- ตัวอย่าง annotated images

---

### Week 3: Model Training

**Goal**: Train YOLO model ที่แม่นยำ >80%
**Output**: Trained model + performance report

#### Day 11-12: Setup Training Environment

```bash
# Install Ultralytics
pip install ultralytics

# Verify CUDA
python3 -c "import torch; print(torch.cuda.is_available())"
# Should output: True
```

#### Day 13-14: Train Model

```python
# train_yolo.py
from ultralytics import YOLO

# Load pretrained model
model = YOLO('yolov8n-seg.pt')  # nano segmentation model

# Train
results = model.train(
    data='pepper_dataset/data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,  # Adjust based on Jetson memory
    device=0,  # GPU
    patience=20,  # Early stopping
    save=True,
    plots=True
)

# Evaluate
metrics = model.val()

print(f"\nmAP50: {metrics.box.map50:.3f}")
print(f"mAP50-95: {metrics.box.map:.3f}")
```

**Training time**: 2-4 hours บน Jetson Orin Nano

**Monitor training**:
```bash
# Watch GPU usage
sudo jtop

# View tensorboard (optional)
tensorboard --logdir runs/segment/train
```

#### Day 15: Model Evaluation

```python
# evaluate_model.py
from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO('runs/segment/train/weights/best.pt')

# Test on validation set
results = model.val()

print("=== Model Performance ===")
print(f"mAP50: {results.box.map50:.3f}")
print(f"mAP50-95: {results.box.map:.3f}")
print(f"Precision: {results.box.mp:.3f}")
print(f"Recall: {results.box.mr:.3f}")

# Per-class metrics
print("\nPer-class mAP50:")
for i, name in enumerate(model.names.values()):
    print(f"  {name}: {results.box.maps50[i]:.3f}")

# Test on single image
img = cv2.imread('test_image.jpg')
results = model(img)

# Visualize
annotated = results[0].plot()
cv2.imshow('Detection', annotated)
cv2.waitKey(0)
```

**Success Criteria**:
- mAP50 > 0.80 (80% accuracy)
- All classes > 0.70
- Visual inspection looks good

**✅ Milestone 3: Week 3 Report**
- Training metrics (loss curves, mAP)
- Confusion matrix
- ตัวอย่าง predictions (good & bad)
- วิเคราะห์ error cases

---

### Week 4: Integration (Detection + Depth)

**Goal**: รวม object detection กับ 3D positioning
**Output**: System ที่บอกตำแหน่ง XYZ ของพริก

#### Day 16-18: Implement Vision Pipeline

```python
# vision_pipeline.py
import cv2
import numpy as np
import yaml
from ultralytics import YOLO

class VisionPipeline:
    def __init__(self, calib_file, model_path):
        # Load calibration
        with open(calib_file, 'r') as f:
            calib = yaml.safe_load(f)

        self.K_left = np.array(calib['K_left'])
        self.D_left = np.array(calib['D_left'])
        self.K_right = np.array(calib['K_right'])
        self.D_right = np.array(calib['D_right'])
        self.R_left = np.array(calib['R_left'])
        self.R_right = np.array(calib['R_right'])
        self.P_left = np.array(calib['P_left'])
        self.P_right = np.array(calib['P_right'])
        self.Q = np.array(calib['Q'])

        # Create rectification maps
        img_size = tuple(calib['image_size'])
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            self.K_left, self.D_left, self.R_left, self.P_left, img_size, cv2.CV_32FC1
        )
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            self.K_right, self.D_right, self.R_right, self.P_right, img_size, cv2.CV_32FC1
        )

        # Stereo matcher
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=11,
            P1=8 * 3 * 11 ** 2,
            P2=32 * 3 * 11 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32
        )

        # Load YOLO model
        self.model = YOLO(model_path)

    def process(self, img_left, img_right):
        # Rectify images
        rect_left = cv2.remap(img_left, self.map_left_x, self.map_left_y, cv2.INTER_LINEAR)
        rect_right = cv2.remap(img_right, self.map_right_x, self.map_right_y, cv2.INTER_LINEAR)

        # Compute disparity
        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
        disparity = self.stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

        # Detect peppers
        results = self.model(rect_left, verbose=False)[0]

        # Extract detections with 3D positions
        detections = []
        for box in results.boxes:
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

            # Get depth
            if 0 <= cy < disparity.shape[0] and 0 <= cx < disparity.shape[1]:
                d = disparity[cy, cx]

                # Convert to 3D
                if d > 0:
                    Z = self.Q[2, 3] / (d + self.Q[3, 3])
                    X = (cx - self.Q[0, 3]) * Z / self.Q[0, 0]
                    Y = (cy - self.Q[1, 3]) * Z / self.Q[1, 1]

                    # Convert from mm to m
                    X, Y, Z = X / 1000.0, Y / 1000.0, Z / 1000.0

                    detection = {
                        'class': int(box.cls[0]),
                        'class_name': self.model.names[int(box.cls[0])],
                        'confidence': float(box.conf[0]),
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'position_3d': (X, Y, Z)
                    }
                    detections.append(detection)

        return detections, rect_left, disparity

# Main loop
pipeline = VisionPipeline('stereo_calibration.yaml', 'runs/segment/train/weights/best.pt')

cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture(1)

while True:
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not (ret_left and ret_right):
        continue

    # Process
    detections, img_vis, disparity = pipeline.process(frame_left, frame_right)

    # Visualize
    for det in detections:
        x1, y1, x2, y2 = det['bbox']
        X, Y, Z = det['position_3d']

        # Draw bbox
        cv2.rectangle(img_vis, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw label
        label = f"{det['class_name']} {det['confidence']:.2f}"
        label += f"\nXYZ: ({X:.3f}, {Y:.3f}, {Z:.3f})m"

        y_text = y1 - 10
        for i, line in enumerate(label.split('\n')):
            cv2.putText(img_vis, line, (x1, y_text + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('Vision Pipeline', img_vis)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap_left.release()
cap_right.release()
cv2.destroyAllWindows()
```

#### Day 19-20: Testing & Validation

**Accuracy Test**:
```python
# test_3d_accuracy.py
# วางพริกที่ตำแหน่งที่รู้ (วัดด้วยไม้บรรทัด)
# เปรียบเทียบกับที่ vision system detect

ground_truth = [
    {'position': (0.05, 0.10, 0.02), 'label': 'Position 1'},  # meters
    {'position': (0.10, 0.15, 0.03), 'label': 'Position 2'},
    # ... อีก 5-10 positions
]

detected = [
    # ... จาก vision pipeline
]

# Calculate error
errors = []
for gt, det in zip(ground_truth, detected):
    gt_pos = np.array(gt['position'])
    det_pos = np.array(det['position_3d'])
    error = np.linalg.norm(gt_pos - det_pos)
    errors.append(error)
    print(f"{gt['label']}: Error = {error * 1000:.1f} mm")

print(f"\nMean error: {np.mean(errors) * 1000:.1f} mm")
print(f"Std error: {np.std(errors) * 1000:.1f} mm")
```

**Target**: Mean error < 10mm (1cm)

**✅ Milestone 4: Week 4 Final Report**

**รายงานสรุป Phase 1** (Vision System):

1. **Stereo Calibration**
   - RMS error
   - Baseline measurement
   - ตัวอย่าง depth map

2. **Object Detection**
   - mAP50, mAP50-95
   - Per-class performance
   - Confusion matrix
   - ตัวอย่าง predictions

3. **3D Positioning**
   - Mean error (mm)
   - Error distribution
   - ทดสอบกับ objects ที่รู้ตำแหน่ง

4. **System Performance**
   - FPS (frames per second)
   - GPU/CPU usage
   - Latency

5. **Challenges & Solutions**
   - ปัญหาที่เจอ
   - วิธีแก้

6. **Next Steps**
   - ROS2 integration plan

---

## Phase 2: ROS2 Integration (Week 5-6)

*(จะทำหลังจาก Phase 1 เสร็จ)*

### Week 5: ROS2 Basics + Camera Node

**Goal**: เรียน ROS2 พื้นฐาน และสร้าง camera node
**Output**: Camera node publishing images

#### Day 21-23: ROS2 Learning
- ติดตั้ง ROS2 Humble
- Tutorial: Publisher/Subscriber
- Tutorial: Custom messages
- Tutorial: Launch files

#### Day 24-25: Camera Node
- สร้าง `pepper_vision` package
- Implement `camera_node.py`
- Publish stereo images

### Week 6: Vision Node

**Goal**: Convert vision pipeline เป็น ROS2 node
**Output**: Vision node publishing detected peppers

#### Day 26-28: Vision Node Implementation
- Implement `vision_node.py`
- Subscribe camera images
- Publish `DetectedPepper` messages

#### Day 29-30: Testing & Visualization
- ทดสอบ camera + vision nodes
- Setup RViz2
- Visualize detections

**✅ Milestone 5: ROS2 Vision System Report**

---

## Phase 3-5: Robot Arms (Week 7-12)

*(ทำต่อหลัง Vision เสร็จสมบูรณ์)*

ดูรายละเอียดใน `docs/07_development_roadmap.md`

---

## สรุป Output ที่จะได้แต่ละ Week

| Week | Output | Report |
|------|--------|--------|
| 1 | Stereo calibration ✓ | รายงาน calibration quality |
| 2 | Dataset 500-1000 images ✓ | Dataset summary |
| 3 | Trained YOLO model ✓ | Training metrics + evaluation |
| 4 | Complete vision pipeline ✓ | **Final Phase 1 Report** |
| 5 | ROS2 camera node | ROS2 setup report |
| 6 | ROS2 vision node | **Phase 2 Report** |

---

## Tips สำหรับการทำ Phase 1

### 1. Calibration Tips
- ถ่ายรูปหลายมุม: ซ้าย-ขวา-บน-ล่าง-ใกล้-ไกล
- ให้ checkerboard เต็มเฟรม ~60-80%
- ไม่ motion blur (ถือให้นิ่ง)
- แสงสม่ำเสมอ

### 2. Data Collection Tips
- เริ่มจากง่าย (พริก 1-3 ผล) → ยาก (10+ ผล)
- Label อย่างละเอียด (แม้พริกที่บดบัง)
- Balance classes (จำนวนใกล้เคียงกัน)
- Quality > Quantity (500 ภาพดี > 1000 ภาพแย่)

### 3. Training Tips
- เริ่มจาก yolov8n (nano) ก่อน
- Monitor training: ดู loss ลดลง, mAP เพิ่มขึ้น
- Early stopping: ถ้า 20 epochs ไม่ดีขึ้น → หยุด
- Overfitting: ถ้า train good แต่ val แย่ → เพิ่ม augmentation

### 4. Testing Tips
- ทดสอบหลายแสง (เช้า/เที่ยง/เย็น)
- ทดสอบหลาย layout
- วัดความแม่นยำด้วยวิธีวิทยาศาสตร์ (ruler!)

---

**Vision-First Roadmap Complete ✓**

**พร้อมเริ่ม Phase 1 เลยไหมครับ?** หรือมีคำถามเพิ่มเติม? 🚀
