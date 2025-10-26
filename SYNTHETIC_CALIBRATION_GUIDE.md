# 🔬 Synthetic Stereo Calibration Testing Guide

**วัตถุประสงค์:** ทดสอบว่า `stereo_calibration.py` ทำงานถูกต้องหรือไม่ โดยใช้ข้อมูลจำลอง (synthetic data) ที่สมบูรณ์แบบ

---

## 🎯 Why Synthetic Data?

### ปัญหาที่พบ:
- Baseline calibration ออกมา 436mm (ผิดปกติมาก!)
- ไม่แน่ใจว่าปัญหาอยู่ที่:
  - ❓ Code มีบั๊ก (`stereo_calibration.py`)
  - ❓ ข้อมูลจริงไม่ดี (focus, pattern spacing, lens distortion)

### วิธีแก้:
✅ **ทดสอบด้วย Perfect Synthetic Data**

**ข้อดี:**
1. **รู้ Ground Truth ทุกค่า** - baseline, focal length, distortion
2. **แยกปัญหาได้ชัดเจน:**
   - Synthetic data ถูก → Code ถูก → ปัญหาอยู่ที่ real data
   - Synthetic data ผิด → Code ผิด → ต้องแก้ algorithm
3. **Reproducible 100%** - ไม่พึ่ง hardware, ทำซ้ำได้เสมอ
4. **Debug ง่าย** - ไม่ต้องถ่ายรูปใหม่, ไม่ต้องปรับ focus

---

## 📋 Files Overview

### สคริปต์หลัก:

**1. `generate_synthetic_calibration.py`**
- สร้างภาพ stereo calibration จำลอง (perfect synthetic data)
- กำหนด camera parameters เอง (known ground truth)
- Project asymmetric circles pattern ลงบนภาพ
- สร้าง 30 poses (มุมมอง/ระยะต่างๆ)

**2. `stereo_calibration_synthetic.py`**
- รัน `stereo_calibration.py` กับ synthetic data
- เปรียบเทียบผลลัพธ์กับ ground truth
- สร้าง comparison report

**3. `stereo_calibration.py`** (existing)
- Algorithm ที่ต้องการทดสอบ

---

## 🚀 Usage Instructions

### Step 1: Generate Synthetic Data

```bash
# บน Jetson Nano (หรือ machine ที่มี Python + OpenCV + NumPy)
cd ~/Jetson
python3 generate_synthetic_calibration.py
```

**Output:**
```
calib_images/synthetic/left/
  img_000_*.jpg
  img_001_*.jpg
  ...
  img_029_*.jpg

calib_images/synthetic/right/
  img_000_*.jpg
  img_001_*.jpg
  ...
  img_029_*.jpg

ground_truth.yaml  ← Known camera parameters
```

**Ground Truth Parameters:**
- **Baseline:** 60.0 mm (realistic)
- **Focal length:** 750.0 px (realistic for IMX219 1280×720)
- **Principal point:** (640.0, 360.0) - image center
- **Distortion:** [0, 0, 0, 0, 0] - perfect lens
- **Pattern:** Asymmetric Circles Grid 5×6, spacing 18mm

---

### Step 2: Run Calibration on Synthetic Data

```bash
python3 stereo_calibration_synthetic.py
```

**Output:**
```
stereo_calib_synthetic.yaml          ← Calibration results
calibration_comparison_report.yaml   ← Comparison report
rectification_maps.npz
```

**หรือรัน manually:**
```bash
# แก้ stereo_calibration.py ใน main() ให้ใช้ synthetic data:
# images_left_path='calib_images/synthetic/left/*.jpg',
# images_right_path='calib_images/synthetic/right/*.jpg',

python3 stereo_calibration.py
```

---

### Step 3: Analyze Results

ดูที่ **comparison report:**

```bash
cat calibration_comparison_report.yaml
```

**Expected Results (Perfect Case):**

| Parameter | Ground Truth | Calibrated | Error | Status |
|-----------|--------------|------------|-------|--------|
| Baseline | 60.0 mm | ~60.0 mm | < 1% | ✓ EXCELLENT |
| Focal Length | 750.0 px | ~750.0 px | < 1% | ✓ EXCELLENT |
| RMS Error | N/A | < 0.5 px | N/A | ✓ EXCELLENT |

---

## 📊 Interpreting Results

### ✅ **PASS Scenario:**

```yaml
comparison:
  baseline_error_pct: 0.5  # < 5%
  focal_error_pct: 0.3     # < 5%
  rms_error: 0.2           # < 0.5
  pass: true
```

**สรุป:**
- ✅ `stereo_calibration.py` **ทำงานถูกต้อง!**
- ❌ ปัญหาอยู่ที่ **real captured images:**
  - Pattern spacing ไม่ตรง (16mm vs 18mm)
  - Focus ไม่เหมาะสม
  - Lens distortion ซับซ้อน
  - Pattern print quality ต่ำ

**แนวทางแก้:**
1. วัด pattern spacing จริงๆ ด้วยเวอร์เนียร์
2. ตรวจสอบ focus values จาก capture logs
3. ถ่ายรูป calibration ใหม่ให้ดีกว่า

---

### ❌ **FAIL Scenario:**

```yaml
comparison:
  baseline_error_pct: 25.0  # ≥ 10%
  focal_error_pct: 12.0     # ≥ 5%
  rms_error: 5.3            # > 2.0
  pass: false
```

**สรุป:**
- ❌ `stereo_calibration.py` **มีปัญหา!**
- 🔍 ต้องตรวจสอบ:
  - Object points generation (line 51-62)
  - Pattern spacing formula
  - `stereoCalibrate()` flags/parameters

**แนวทางแก้:**
1. ตรวจสอบ asymmetric grid formula:
   ```python
   objp[i * pattern_cols + j] = [
       (2 * j + i % 2) * spacing_mm,  # ← ตรงนี้ถูกหรือไม่?
       i * spacing_mm,
       0
   ]
   ```

2. ลอง spacing อื่น (16mm แทน 18mm)

3. ทดสอบด้วย flags ต่างกัน:
   ```python
   # Current: CALIB_FIX_INTRINSIC
   # Try: 0 (optimize everything)
   ```

---

## 🔧 Advanced Testing

### Test 1: Different Baselines

แก้ `generate_synthetic_calibration.py`:

```python
# Test baseline = 50mm
generator = SyntheticStereoCalibrationGenerator(
    baseline_mm=50.0,  # Changed
    ...
)
```

Run และดูว่า calibration ได้ baseline ตรงหรือไม่

---

### Test 2: Add Lens Distortion

```python
# Test with realistic distortion
generator = SyntheticStereoCalibrationGenerator(
    distortion_coeffs=[-0.1, 0.01, 0, 0, 0],  # Slight barrel distortion
    ...
)
```

Run และดูว่า calibration handle distortion ได้หรือไม่

---

### Test 3: Different Spacing

```python
# Test if spacing = 16mm works
generator = SyntheticStereoCalibrationGenerator(
    spacing_mm=16.0,  # Changed from 18mm
    ...
)
```

จากนั้นรัน calibration:
```python
calibrate_stereo_asymmetric_circles(
    ...
    spacing_mm=16.0,  # Must match!
)
```

---

## 📝 Ground Truth File Format

**`ground_truth.yaml`:**

```yaml
description: Synthetic Stereo Calibration Ground Truth
generated_date: 2025-10-26T...
image_width: 1280
image_height: 720
pattern_type: asymmetric_circles
pattern_rows: 5
pattern_cols: 6
pattern_spacing_mm: 18.0
circle_diameter_mm: 14.0

# KNOWN GROUND TRUTH:
baseline_mm: 60.0

camera_matrix:
  - [750.0, 0.0, 640.0]
  - [0.0, 750.0, 360.0]
  - [0.0, 0.0, 1.0]

distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]

rotation_matrix:
  - [1.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0]
  - [0.0, 0.0, 1.0]

translation_vector:
  - [60.0]  # Baseline
  - [0.0]
  - [0.0]

poses:
  - index: 0
    distance_mm: 300.0
    tilt_x_deg: -15.0
    tilt_y_deg: -15.0
    tilt_z_deg: -10.0
  ...
```

---

## 🎨 How Synthetic Images Are Generated

### 1. Virtual Camera Setup

```
Left Camera                Right Camera
    |                          |
    |<------- 60mm ----------->|
    |                          |
    ↓                          ↓
  (0,0,0)                  (60,0,0)
```

### 2. Pattern 3D Points

Asymmetric circles grid (5×6, spacing 18mm):

```
Y ↑
  |
  • - • - • - • - • - •     Row 0: offset 0
  - • - • - • - • - • -     Row 1: offset +9mm (i%2)
  • - • - • - • - • - •     Row 2: offset 0
  - • - • - • - • - • -     Row 3: offset +9mm
  • - • - • - • - • - •     Row 4: offset 0

  --------------------------------> X
```

### 3. Projection

For each pose (distance, tilt angles):
```python
# Pattern position in world
rvec, tvec = pose_to_transform(distance, tilt_x, tilt_y)

# Project to left camera
points_2d_left = cv2.projectPoints(pattern_3d, rvec, tvec, K, dist)

# Project to right camera (offset by baseline)
tvec_right = tvec - [baseline, 0, 0]
points_2d_right = cv2.projectPoints(pattern_3d, rvec, tvec_right, K, dist)
```

### 4. Rendering

```python
# White background
img = np.ones((720, 1280), dtype=uint8) * 255

# Draw black circles at projected points
for pt in points_2d:
    cv2.circle(img, center, radius, 0, -1)
```

---

## ✅ Success Criteria

### For stereo_calibration.py to PASS:

1. **Baseline Error < 5%**
   - Ground truth: 60.0 mm
   - Calibrated: 57-63 mm

2. **Focal Length Error < 5%**
   - Ground truth: 750.0 px
   - Calibrated: 712-788 px

3. **RMS Error < 1.0 pixels**
   - Perfect data: < 0.5 pixels
   - With noise: < 1.0 pixels

4. **Principal Point Close to Center**
   - Ground truth: (640, 360)
   - Calibrated: within ±10 px

---

## 🐛 Troubleshooting

### Problem: "Pattern not detected in synthetic images"

**Cause:** Circle size too small/large

**Fix:**
```python
# In generate_synthetic_calibration.py, adjust:
radius_px = int((self.circle_diameter_mm / avg_distance) * self.focal_length * 0.5)
radius_px = max(8, min(radius_px, 25))  # Adjust range
```

---

### Problem: "RMS error > 2.0 pixels on perfect data"

**Cause:** Bug in stereo_calibration.py

**Fix:** Check:
1. Pattern points generation (asymmetric formula)
2. spacing_mm value
3. `stereoCalibrate()` parameters

---

### Problem: "Baseline error > 10%"

**Possible causes:**
1. ❌ Object points formula wrong
2. ❌ Spacing value incorrect
3. ❌ Pattern type mismatch

**Debug steps:**
```bash
# Visualize detected points
python3 test_pattern_detection.py
# Check if all 30 points detected in each image
```

---

## 📚 References

- **stereo_calibration.py:** Main calibration algorithm
- **CALIBRATION_ANALYSIS.md:** Analysis of capture_calibration.py
- **ground_truth.yaml:** Known camera parameters
- **OpenCV docs:** https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html

---

## 🎯 Next Steps After Testing

### If PASS (Synthetic works, Real fails):

1. ✅ Measure real pattern spacing with caliper
2. ✅ Check focus values from capture logs
3. ✅ Re-capture calibration images with better quality
4. ✅ Try spacing_mm = 16.0 if pattern was printed at different scale

### If FAIL (Synthetic fails):

1. ❌ Fix object points generation
2. ❌ Verify pattern type (asymmetric vs symmetric)
3. ❌ Adjust calibration flags
4. ❌ Check for OpenCV version differences

---

**Generated:** 2025-10-26
**Author:** Claude (10X Developer Mode)
