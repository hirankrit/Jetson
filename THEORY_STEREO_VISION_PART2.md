# ทฤษฎีและหลักการ Stereo Vision (Part 2)
## สำหรับระบบ Pepper Sorting Robot

**ต่อจาก Part 1**: บทที่ 1-5 (Camera Model, Calibration, Epipolar Geometry, Rectification)
**Part 2**: บทที่ 6-8 (Disparity, Stereo Matching, Applications) + ภาคผนวก

---

# บทที่ 6: Disparity และ Depth Estimation

## 6.1 Disparity คืออะไร?

### คำนิยาม

**Disparity (d)** คือความแตกต่างของตำแหน่ง x ระหว่างจุดที่สอดคล้องกันบนภาพซ้ายและภาพขวา (หลัง rectification)

**สูตร**:
```
d = x_left - x_right

โดยที่:
  x_left = ตำแหน่ง x ของจุดบนภาพซ้าย
  x_right = ตำแหน่ง x ของจุดบนภาพขวา
  (y_left = y_right เพราะ rectified แล้ว)
```

### การตีความ Disparity

```
      Camera L        Camera R
          👁️             👁️
           ╲            ╱
            ╲          ╱
  Near       ╲   P1  ╱        → Large disparity (d₁)
              ╲     ╱
               ╲   ╱
  Middle        ╲P2╱          → Medium disparity (d₂)
                 ╲╱
  Far            P3           → Small disparity (d₃)

ความสัมพันธ์:
  d₁ > d₂ > d₃
  Z₁ < Z₂ < Z₃
```

**ข้อสังเกต**:
- วัตถุใกล้ → disparity สูง → เห็นการเลื่อนมาก
- วัตถุไกล → disparity ต่ำ → เห็นการเลื่อนน้อย
- วัตถุที่อินฟินิตี้ → disparity ≈ 0

## 6.2 ความสัมพันธ์ระหว่าง Disparity และ Depth

### สูตรพื้นฐาน

**จาก Similar Triangles**:

```
        CL          CR
        ●────b─────●
         ╲         ╱
          ╲       ╱  Optical axes
           ╲     ╱
            ╲   ╱
             ╲ ╱
              ● P (depth Z)
```

**Derivation**:
```
จากเรขาคณิต:

Left camera:  xL / f = X / Z
Right camera: xR / f = (X - b) / Z

ลบกัน:
  xL - xR = (X / Z - (X - b) / Z) × f
          = (b / Z) × f

ดังนั้น:
  d = xL - xR = (f × b) / Z

จัดรูป:
  Z = (f × b) / d  ← **สูตรสำคัญที่สุด!**
```

### ตัวอย่างการคำนวณ

**กำหนด (จากโปรเจคจริง)**:
```
Focal length (f) = 688.31 pixels
Baseline (b) = 60.57 mm
```

**คำนวณ depth สำหรับ disparity ต่างๆ**:

| Disparity (d) | Depth (Z) | หน่วย |
|---------------|-----------|-------|
| 512 px | (688×60.57)/512 = **81.5 mm** | ~8 cm (ใกล้มาก) |
| 256 px | (688×60.57)/256 = **163 mm** | ~16 cm |
| 128 px | (688×60.57)/128 = **326 mm** | ~33 cm ✅ |
| 64 px | (688×60.57)/64 = **651 mm** | ~65 cm |
| 32 px | (688×60.57)/32 = **1303 mm** | ~130 cm (ไกล) |
| 16 px | (688×60.57)/16 = **2606 mm** | ~260 cm (ไกลมาก) |

**สังเกต**:
- Disparity ลดครึ่ง → Depth เพิ่ม 2 เท่า
- ความแม่นยำลดลงเมื่อระยะไกลขึ้น (ดูหัวข้อถัดไป)

## 6.3 ความแม่นยำของ Depth Measurement

### Depth Error Analysis

**จาก**:
```
Z = (f × b) / d
```

**หา error ของ Z เมื่อ d มี error δd**:
```
dZ/dd = -f × b / d²

δZ = |dZ/dd| × δd
   = (f × b / d²) × δd
   = (Z² / (f × b)) × δd

หรือ:
  δZ / Z = (Z / (f × b)) × δd
```

**ตีความ**:
```
Error ของ depth:
  - แปรผันตาม Z² (ยิ่งไกล error ยิ่งสูง!)
  - แปรผันตาม δd (disparity error 1 px → depth error เท่าไหร่?)
  - แปรผกผันกับ f × b (focal length, baseline ยิ่งใหญ่ → error ยิ่งน้อย)
```

### ตัวอย่าง Error Calculation

**สมมุติ**:
- Disparity error δd = 1 pixel
- f × b = 688.31 × 60.57 = 41,699 mm·px

**คำนวณ depth error สำหรับระยะต่างๆ**:

| Depth (Z) | Disparity (d) | δZ (1 px error) | % Error |
|-----------|---------------|-----------------|---------|
| 33 cm | 128 px | 2.6 mm | 0.8% ✅ |
| 65 cm | 64 px | 10.2 mm | 1.6% ✅ |
| 130 cm | 32 px | 40.7 mm | 3.1% ⚠️ |
| 260 cm | 16 px | 162.7 mm | 6.3% ❌ |

**บทเรียน**:
- ระยะใกล้ (25-50 cm) → แม่นยำมาก ✅
- ระยะกลาง (50-100 cm) → แม่นยำพอใช้ ⚠️
- ระยะไกล (>100 cm) → error สูง ❌

**สำหรับ Pepper Sorting (30-40 cm)**:
```
ทำงานที่ระยะใกล้ → error ต่ำ → เหมาะสม! ✅
```

## 6.4 Disparity Map

### ความหมาย

**Disparity Map** คือภาพที่แต่ละ pixel บอกค่า disparity

```
Rectified Left:        Rectified Right:       Disparity Map:
┌─────────┐           ┌─────────┐           ┌─────────┐
│    🌶️    │           │   🌶️     │           │  ███▓▓  │
│         │  -----→   │         │  -----→   │ ████▓▒  │
│  🌶️  🌶️  │           │ 🌶️  🌶️   │           │ ███▓▒░  │
└─────────┘           └─────────┘           └─────────┘
                                            █ = High disparity (ใกล้)
                                            ░ = Low disparity (ไกล)
```

**Format**:
- แต่ละ pixel = disparity value (หน่วย: pixels)
- Grayscale image (ค่าสูง = สว่าง, ค่าต่ำ = มืด)
- 0 = no valid disparity (ไม่มีข้อมูล)

### การแสดงผล Disparity Map

**Normalization สำหรับแสดงผล**:
```python
import cv2
import numpy as np

# Disparity map จาก stereo matching (ค่า 0-512)
disparity = ...  # shape (H, W), dtype=float32

# Normalize เป็น 0-255
disparity_vis = cv2.normalize(
    disparity,
    None,
    alpha=0,
    beta=255,
    norm_type=cv2.NORM_MINMAX,
    dtype=cv2.CV_8U
)

# Apply colormap (ง่ายต่อการมอง)
disparity_color = cv2.applyColorMap(disparity_vis, cv2.COLORMAP_JET)

# Red = ใกล้, Blue = ไกล
cv2.imshow('Disparity Map', disparity_color)
```

### จาก Disparity Map → Depth Map

```python
# Load calibration Q matrix
Q = ...  # จาก stereo rectification

# Convert disparity → 3D points
points_3D = cv2.reprojectImageTo3D(disparity, Q)

# Extract depth (Z coordinate)
depth_map = points_3D[:, :, 2]  # shape (H, W)

# Depth map ใน mm หรือ cm (ขึ้นกับหน่วยของ baseline)

# ถ้าต้องการ depth map อย่างเดียว (ไม่ใช่ 3D):
depth_map = (f * baseline) / disparity
depth_map[disparity == 0] = 0  # Invalid pixels
```

## 6.5 Depth Map Quality Metrics

### 1. Coverage (ความครอบคลุม)

**คำนิยาม**: เปอร์เซ็นต์ของ pixels ที่มี valid depth

```python
def calculate_coverage(depth_map):
    """คำนวณ coverage ของ depth map"""
    total_pixels = depth_map.size
    valid_pixels = np.count_nonzero(depth_map > 0)
    coverage = (valid_pixels / total_pixels) * 100
    return coverage

# ตัวอย่าง
coverage = calculate_coverage(depth_map)
print(f"Coverage: {coverage:.1f}%")
```

**เกณฑ์**:
```
Coverage > 80%  → Excellent ✅ (pattern board, textured objects)
Coverage 50-80% → Good ✅ (most real objects)
Coverage 30-50% → Acceptable ⚠️ (smooth objects)
Coverage < 30%  → Poor ❌ (very smooth / no texture)
```

**จากโปรเจค**:
```
Pattern board: 80-90% ✅
Peppers: 40-70% ✅
Smooth background: 8-27% (ปกติ)
```

### 2. Accuracy (ความแม่นยำ)

**วิธีวัด**: เปรียบเทียบกับ ground truth

```python
def calculate_accuracy(depth_map, ground_truth_depth, roi):
    """
    คำนวณ accuracy ใน ROI

    Args:
        depth_map: depth map ที่คำนวณได้ (cm)
        ground_truth_depth: ระยะที่วัดจริง (cm)
        roi: (x, y, w, h) บริเวณที่ต้องการวัด

    Returns:
        mean_error, std_error, percent_error
    """
    x, y, w, h = roi
    roi_depth = depth_map[y:y+h, x:x+w]

    valid_depth = roi_depth[roi_depth > 0]

    if len(valid_depth) == 0:
        return None, None, None

    mean_depth = np.mean(valid_depth)
    std_depth = np.std(valid_depth)

    mean_error = mean_depth - ground_truth_depth
    percent_error = (mean_error / ground_truth_depth) * 100

    return mean_error, std_depth, percent_error

# ตัวอย่าง
roi = (100, 100, 50, 50)  # ROI บน pepper
error_cm, std_cm, error_pct = calculate_accuracy(
    depth_map, ground_truth=32.0, roi=roi
)

print(f"Mean error: {error_cm:.2f} cm ({error_pct:.1f}%)")
print(f"Std dev: {std_cm:.2f} cm")
```

**เกณฑ์**:
```
Error < 1% depth   → Excellent ✅
Error < 3% depth   → Good ✅
Error < 5% depth   → Acceptable ⚠️
Error ≥ 5% depth   → Poor ❌
```

### 3. Repeatability (ความเสถียร)

**วิธีวัด**: วัดหลายครั้งที่ตำแหน่งเดียวกัน

```python
def test_repeatability(n_tests=15):
    """ทดสอบ repeatability"""
    depths = []

    for i in range(n_tests):
        # Capture + compute depth
        depth_map = capture_and_compute_depth()
        roi_depth = depth_map[y:y+h, x:x+w]
        mean_depth = np.mean(roi_depth[roi_depth > 0])
        depths.append(mean_depth)

    mean = np.mean(depths)
    std = np.std(depths)

    print(f"Repeatability: {mean:.2f} ± {std:.2f} cm")
    print(f"Coefficient of variation: {(std/mean)*100:.2f}%")

    return mean, std
```

**เกณฑ์**:
```
Std < 1 mm     → Excellent ✅ (โปรเจคนี้: 0.4mm!)
Std < 5 mm     → Good ✅
Std < 10 mm    → Acceptable ⚠️
Std ≥ 10 mm    → Poor ❌
```

## 6.6 Depth Estimation Challenges

### Challenge 1: Smooth Surfaces (พื้นผิวเรียบ)

**ปัญหา**:
- ไม่มี texture → matching ยาก
- Disparity map มีรู (holes) เยอะ
- Coverage ต่ำ

**ตัวอย่าง**:
```
Textured (พริก):        Smooth (ผนัง):
█████████               ░░░▓░░░░
████████░               ░░░░░░░░
██████▓▒░               ░░░░░▓░░

Coverage: 70% ✅        Coverage: 10% ❌
```

**Solution**:
- เพิ่ม lighting → create artificial texture (shadows)
- ใช้ pattern projection (structured light)
- ยอมรับ → design system around it

### Challenge 2: Occlusion (การบัง)

**ปัญหา**: พื้นที่ที่กล้องซ้ายเห็น แต่กล้องขวาไม่เห็น (หรือตรงกันข้าม)

```
Left:              Right:
  ●───────────────●
   ╲             ╱
    ╲           ╱
     ╲  ┌───┐ ╱
      ╲ │ A │╱ ← A visible to both
       ╲└───┘
        ╲│ B│  ← B visible to left only (occluded from right)
         └──┘
```

**ผลกระทบ**:
- พื้นที่ B → ไม่มี correspondence ที่ถูกต้อง
- Disparity = 0 หรือ incorrect
- มักเกิดที่ขอบวัตถุ (edges)

**จากโปรเจค Week 1 Discovery**:
```
พริกกอง:
  Center (top) → มี occlusion → no depth ❌
  Edges        → visible to both → good depth ✅

นี่คือ physics limitation ของ stereo!
```

### Challenge 3: Specular Reflection (แสงสะท้อน)

**ปัญหา**: พื้นผิวมันเงา → สะท้อนแสงต่างกันระหว่าง 2 กล้อง

```
Left sees:         Right sees:
   ╱│╲                ╱│╲
  ╱ │ ╲              ╱ │ ╲
 ╱  ●  ╲            ╱ ●'  ╲   ← Different reflection
    │                  │
  Shiny object
```

**Solution**:
- ใช้ polarizing filter
- ปรับมุมแสง (ไม่ส่องตรง)
- ใช้ diffuse lighting

### Challenge 4: Repetitive Patterns (ลวดลายซ้ำ)

**ปัญหา**: Pattern ซ้ำกัน → matching ผิด (ambiguity)

```
Left:  ▓▒▓▒▓▒▓▒
Right: ▒▓▒▓▒▓▒▓

ตรง ▓ ใดๆ ใน left สามารถ match กับ ▓ หลายตัวใน right ได้!
```

**Solution**:
- ใช้ global matching algorithms (ไม่ใช่ local เพียงอย่างเดียว)
- Uniqueness check (ratio test)

## 6.7 สรุปบทที่ 6

### สูตรสำคัญ

| สูตร | ความหมาย | หมายเหตุ |
|------|----------|----------|
| `Z = (f×b)/d` | Depth from disparity | สำคัญที่สุด! |
| `δZ = (Z²/(f×b))×δd` | Depth error | Error ∝ Z² |
| `d = xL - xR` | Disparity | หลัง rectification |

### Quality Metrics

| Metric | เกณฑ์ดี | เกณฑ์พอใช้ |
|--------|---------|-----------|
| **Coverage** | >80% | 40-80% |
| **Accuracy** | <1% | <5% |
| **Repeatability** | <1mm std | <5mm std |

### Challenges & Solutions

| Challenge | Cause | Solution |
|-----------|-------|----------|
| **Smooth surfaces** | No texture | Add lighting, accept limitation |
| **Occlusion** | Physics | Adaptive percentile, YOLO+ROI |
| **Specular** | Shiny surface | Polarizer, diffuse light |
| **Repetitive** | Ambiguity | Global matching, uniqueness |

---

# บทที่ 7: Stereo Matching Algorithms

## 7.1 Stereo Matching Problem

### Problem Definition

**Input**:
- Rectified left image (IL)
- Rectified right image (IR)

**Output**:
- Disparity map (D)

**Objective**:
สำหรับแต่ละ pixel (x, y) ใน IL, หา pixel (x-d, y) ใน IR ที่ **match กันมากที่สุด**

```
Rectified Left (IL):          Rectified Right (IR):
┌─────────────────┐          ┌─────────────────┐
│        ●─────────┼─────────→│      ●          │
│       (x,y)      │   d px   │   (x-d,y)       │
└─────────────────┘          └─────────────────┘
                      ↓
               Disparity = d
```

### Matching Cost

**วิธีวัดความ "เหมือนกัน"**:

#### 1. Sum of Absolute Differences (SAD)
```
C_SAD(x, y, d) = Σ |IL(x+i, y+j) - IR(x-d+i, y+j)|
                i,j∈W

W = window (เช่น 11×11)
```

**ข้อดี**: เร็ว
**ข้อเสีย**: ไวต่อความสว่างต่างกัน

#### 2. Sum of Squared Differences (SSD)
```
C_SSD(x, y, d) = Σ [IL(x+i, y+j) - IR(x-d+i, y+j)]²
                i,j∈W
```

**ข้อดี**: ลงโทษ outlier มากกว่า SAD
**ข้อเสีย**: ช้ากว่า SAD เล็กน้อย

#### 3. Normalized Cross Correlation (NCC)
```
           Σ (IL - μL)(IR - μR)
C_NCC = ─────────────────────────
        √[Σ(IL-μL)²] × √[Σ(IR-μR)²]

μL, μR = mean ใน window
```

**ข้อดี**: ทนต่อความสว่างต่างกัน
**ข้อเสีย**: ช้า (ต้องคำนวณ mean, std)

## 7.2 StereoBM (Block Matching)

### หลักการ

**Block Matching**: หา correspondence โดยเปรียบเทียบ **block** (window) รอบๆ pixel

**Algorithm**:
```
สำหรับแต่ละ pixel (x, y) ใน left image:
  1. เอา block รอบ (x, y) ขนาด blockSize × blockSize
  2. Slide block นี้ไปทาง left บน right image
  3. คำนวณ cost สำหรับแต่ละ shift (0 ถึง numDisparities)
  4. หา disparity ที่ cost ต่ำที่สุด
```

**Pseudocode**:
```python
def stereo_bm(left, right, blockSize=11, numDisparities=128):
    H, W = left.shape
    disparity = np.zeros((H, W), dtype=np.float32)

    for y in range(blockSize//2, H - blockSize//2):
        for x in range(blockSize//2 + numDisparities, W - blockSize//2):
            # Extract left block
            blockL = left[y-r:y+r+1, x-r:x+r+1]

            min_cost = inf
            best_d = 0

            # Search right image
            for d in range(0, numDisparities):
                if x - d < blockSize//2:
                    continue

                blockR = right[y-r:y+r+1, x-d-r:x-d+r+1]

                # Compute cost (SAD)
                cost = np.sum(np.abs(blockL - blockR))

                if cost < min_cost:
                    min_cost = cost
                    best_d = d

            disparity[y, x] = best_d

    return disparity
```

### OpenCV Implementation

```python
import cv2

# Create StereoBM object
stereo = cv2.StereoBM_create(
    numDisparities=128,  # Max disparity (must be divisible by 16)
    blockSize=11         # Block size (odd number, 5-21)
)

# Optional: Set parameters
stereo.setPreFilterCap(31)
stereo.setUniquenessRatio(10)
stereo.setSpeckleWindowSize(100)
stereo.setSpeckleRange(32)

# Compute disparity
disparity = stereo.compute(left_gray, right_gray)

# Normalize (StereoBM returns 16-bit fixed-point)
disparity = disparity.astype(np.float32) / 16.0
```

### StereoBM Parameters

| Parameter | ความหมาย | แนะนำ | หมายเหตุ |
|-----------|----------|-------|----------|
| `numDisparities` | จำนวน disparity ที่ search | 128-256 | Must be ×16 |
| `blockSize` | ขนาด window | 11-21 | Odd number |
| `preFilterCap` | Clip pre-filtered pixels | 31 | ลด noise |
| `uniquenessRatio` | Margin for best match | 10-15 | สูง=strict |
| `speckleWindowSize` | Size of speckle to filter | 50-200 | ลด noise spots |
| `speckleRange` | Max disparity variation | 16-32 | ใน speckle |

**ข้อดี**:
- ✅ เร็ว (simple algorithm)
- ✅ เหมาะสำหรับ real-time

**ข้อเสีย**:
- ❌ ไม่ดีใน textureless regions
- ❌ Noisy disparity map
- ❌ Edges ไม่ sharp

## 7.3 StereoSGBM (Semi-Global Block Matching)

### หลักการ

**Semi-Global Matching (SGM)**: ไม่ใช่ local เพียงอย่างเดียว แต่พิจารณา **global smoothness**

**Key Idea**:
- Local matching: หา cost สำหรับแต่ละ pixel
- Global optimization: เพิ่ม penalty ถ้า neighbor มี disparity ต่างกันมาก
- Semi-global: ประนีประนอมระหว่าง local และ global

**Energy Function**:
```
E(D) = Σ C(p, Dp) + Σ P1 × [|Dp - Dq| = 1]
       p           q∈Np

     + Σ P2 × [|Dp - Dq| > 1]
       q∈Np

โดยที่:
  C(p, Dp) = matching cost
  P1 = penalty สำหรับ disparity ต่าง 1 pixel
  P2 = penalty สำหรับ disparity ต่างมาก (>1 pixel)
  Np = neighbors ของ p
```

**ตีความ**:
- ถ้า neighbor มี disparity ต่างกันเล็กน้อย → โทษน้อย (P1)
- ถ้า neighbor มี disparity ต่างกันมาก → โทษมาก (P2)
- → Encourage smooth disparity (แต่ยอมให้มี edges)

### Dynamic Programming Approach

**SGM ใช้ Dynamic Programming หาเส้นทาง (paths) จาก 8 ทิศทาง**:

```
      ↖  ↑  ↗
        ╲│╱
      ← ─●─ →   8 directions
        ╱│╲
      ↙  ↓  ↘
```

**สำหรับแต่ละ direction r**:
```
Lr(p, d) = C(p, d) + min {
    Lr(p-r, d),           # Same disparity
    Lr(p-r, d-1) + P1,    # Disparity +1
    Lr(p-r, d+1) + P1,    # Disparity -1
    min_k Lr(p-r, k) + P2 # Large change
}
```

**รวมจาก 8 ทิศทาง**:
```
S(p, d) = Σ Lr(p, d)
          r

Best disparity:
  D(p) = argmin_d S(p, d)
```

### OpenCV Implementation

```python
import cv2

# Create StereoSGBM object
stereo = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=512,      # Max disparity (must be ×16)
    blockSize=11,            # Window size
    P1=8 * 3 * blockSize**2, # P1 = 8 * channels * blockSize²
    P2=32 * 3 * blockSize**2,# P2 = 32 * channels * blockSize²
    disp12MaxDiff=1,         # Left-right consistency check
    uniquenessRatio=10,      # Uniqueness margin (%)
    speckleWindowSize=100,   # Speckle filter window
    speckleRange=32,         # Speckle disparity range
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # 3-way matching
)

# Compute disparity
disparity = stereo.compute(left_gray, right_gray)

# Normalize
disparity = disparity.astype(np.float32) / 16.0
```

### StereoSGBM Parameters

**สำคัญที่สุด**:

1. **numDisparities**: จำนวน disparity search
   ```
   คำนวณจาก:
     max_d = (f × b) / min_Z

   โปรเจคนี้:
     f × b = 41699 mm·px
     min_Z = 250 mm (ใกล้สุด)
     max_d = 41699 / 250 = 167 px
     → ใช้ 512 px (ปลอดภัย, 32×16)
   ```

2. **P1, P2**: Smoothness penalties
   ```
   Standard formula:
     P1 = 8 × channels × blockSize²
     P2 = 32 × channels × blockSize²

   channels = 1 (grayscale) หรือ 3 (color)

   ตัวอย่าง blockSize=11:
     P1 = 8 × 3 × 121 = 2904
     P2 = 32 × 3 × 121 = 11616

   P2 > P1 → โทษการเปลี่ยนแปลงใหญ่มากกว่า
   ```

3. **uniquenessRatio**: Uniqueness margin
   ```
   ยิ่งสูง → ยิ่ง strict (ปฏิเสธ ambiguous matches)

   10-15: Standard (recommended)
   5-10: Permissive (more matches, may be noisy)
   15-20: Strict (fewer matches, high confidence)
   ```

**ข้อดี**:
- ✅ Disparity map เนียนกว่า StereoBM
- ✅ Sharp edges
- ✅ ดีกับ textureless regions มากกว่า

**ข้อเสีย**:
- ⚠️ ช้ากว่า StereoBM
- ⚠️ Memory usage สูง

## 7.4 Post-Processing

### 1. Speckle Filtering

**ปัญหา**: Disparity map มี "speckles" (จุดแยกเดี่ยวที่ผิดปกติ)

**วิธีแก้**:
```python
stereo.setSpeckleWindowSize(100)  # ขนาด connected component ที่ถือว่า speckle
stereo.setSpeckleRange(32)        # Disparity variation ใน speckle

# Manual filtering (ถ้า built-in ไม่พอ):
def remove_speckles(disparity, max_size=100, max_diff=32):
    """Remove small isolated regions (speckles)"""
    # Label connected components
    num_labels, labels = cv2.connectedComponents(
        (disparity > 0).astype(np.uint8)
    )

    for label in range(1, num_labels):
        mask = (labels == label)
        size = np.sum(mask)

        if size < max_size:
            # Check disparity variation
            region_disp = disparity[mask]
            if np.ptp(region_disp) < max_diff:
                disparity[mask] = 0  # Remove speckle

    return disparity
```

### 2. WLS Filter (Weighted Least Squares)

**วัตถุประสงค์**: ทำให้ disparity map เนียน แต่คงขอบ (edges) ไว้

**หลักการ**: ใช้ left image เป็น guide → edges ใน left image = edges ใน disparity map

```python
import cv2

# สร้าง WLS filter
left_matcher = cv2.StereoSGBM_create(...)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
wls_filter.setLambda(8000)    # Smoothness (higher = smoother)
wls_filter.setSigmaColor(1.5) # Edge sensitivity

# Compute left และ right disparities
disp_left = left_matcher.compute(left_gray, right_gray)
disp_right = right_matcher.compute(right_gray, left_gray)

# Apply WLS filter
disp_filtered = wls_filter.filter(
    disparity_map_left=disp_left,
    left_view=left_gray,
    disparity_map_right=disp_right
)

# Normalize
disp_filtered = disp_filtered.astype(np.float32) / 16.0
```

**พารามิเตอร์**:
- `lambda`: สูง → smooth มาก (แต่อาจเบลอ edges)
- `sigmaColor`: ต่ำ → sensitive to edges (preserve ดีกว่า)

**ข้อดี**:
- ✅ Smooth แต่คง edges
- ✅ ลด noise มาก

**ข้อเสีย**:
- ❌ ช้า (ต้องคำนวณ 2 ครั้ง: left+right)
- ❌ Memory usage สูง (เก็บ 2 disparity maps)

### 3. Left-Right Consistency Check

**หลักการ**: หา disparity จาก 2 ทิศทาง (left→right และ right→left) แล้วตรวจสอบว่าสอดคล้องกันหรือไม่

```python
def lr_consistency_check(disp_left, disp_right, threshold=1):
    """
    Left-Right consistency check

    Args:
        disp_left: Disparity จาก left → right
        disp_right: Disparity จาก right → left
        threshold: Max allowed difference (pixels)

    Returns:
        disp_consistent: Disparity ที่ pass check
    """
    H, W = disp_left.shape
    disp_consistent = np.copy(disp_left)

    for y in range(H):
        for x in range(W):
            d_left = disp_left[y, x]

            if d_left <= 0:
                continue

            # ตำแหน่งสอดคล้องใน right image
            x_right = int(x - d_left)

            if x_right < 0 or x_right >= W:
                disp_consistent[y, x] = 0
                continue

            d_right = disp_right[y, x_right]

            # ตรวจสอบ consistency
            if abs(d_left - d_right) > threshold:
                disp_consistent[y, x] = 0  # Inconsistent!

    return disp_consistent
```

**ข้อดี**: กรอง outliers ได้ดี
**ข้อเสีย**: ช้า (compute 2 ครั้ง)

## 7.5 สรุปบทที่ 7

### Algorithm Comparison

| Feature | StereoBM | StereoSGBM | SGM+WLS |
|---------|----------|------------|---------|
| **Speed** | Fast ✅ | Moderate ⚠️ | Slow ❌ |
| **Quality** | Basic ⚠️ | Good ✅ | Excellent ✅ |
| **Edge preservation** | Poor ❌ | Good ✅ | Excellent ✅ |
| **Textureless regions** | Poor ❌ | Moderate ⚠️ | Good ✅ |
| **Memory** | Low ✅ | Moderate ⚠️ | High ❌ |

### Recommended Pipeline

**สำหรับ real-time applications (โปรเจคนี้)**:
```python
# Resolution: 640×480 (lower = faster)
# Algorithm: StereoSGBM (ไม่ใช้ WLS → stable, fast)
# Post-process: Speckle filter only

stereo = cv2.StereoSGBM_create(
    numDisparities=512,      # จาก analysis
    blockSize=11,
    P1=2904,
    P2=11616,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32
)

disparity = stereo.compute(left_640, right_640) / 16.0

# ไม่ใช้ WLS → ประมวลผลเร็ว (~500ms)
# Accuracy: ±0.5cm (เพียงพอสำหรับ sorting)
```

---

# บทที่ 8: การประยุกต์ใช้งาน

## 8.1 Pepper Sorting Robot (โปรเจคนี้)

### System Architecture

**Pipeline**:
```
Camera Pair
    ↓
[Stereo Matching]
    ↓
Disparity Map
    ↓
[Depth Computation]
    ↓
Depth Map
    ↓
[YOLO Detection] → Bounding Boxes
    ↓
[ROI Depth Extraction]
    ↓
[Adaptive Percentile]
    ↓
3D Position (X, Y, Z)
    ↓
[Robot Control]
    ↓
Pick & Place
```

### Key Techniques Applied

#### 1. YOLO + ROI Depth
```python
# Step 1: YOLO detection
bboxes = yolo_model.predict(image)  # [(x,y,w,h, class, conf), ...]

for bbox in bboxes:
    x, y, w, h, cls, conf = bbox

    # Step 2: Extract ROI depth
    roi_depth = depth_map[y:y+h, x:x+w]
    valid = roi_depth[roi_depth > 0]

    # Step 3: Coverage
    coverage = len(valid) / (w * h)

    # Step 4: Adaptive percentile
    if coverage < 0.25:
        percentile = 5  # Low coverage → closest point
    else:
        percentile = 10

    pepper_depth = np.percentile(valid, percentile)

    # Step 5: 3D position
    x_center = x + w/2
    y_center = y + h/2

    position_3d = (x_center, y_center, pepper_depth)

    # Step 6: Send to robot
    robot_arm.pick(position_3d)
```

#### 2. Foreground Detection
```python
def extract_foreground(depth_map, min_depth=100, max_depth=500):
    """แยก foreground peppers ออกจาก background"""
    # Threshold
    fg_mask = (depth_map > min_depth) & (depth_map < max_depth)

    # Morphological operations
    kernel = np.ones((5,5), np.uint8)
    fg_mask = cv2.morphologyEx(fg_mask.astype(np.uint8),
                                cv2.MORPH_OPEN, kernel)
    fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)

    # Extract foreground depth
    foreground_depth = depth_map.copy()
    foreground_depth[~fg_mask] = 0

    return foreground_depth, fg_mask
```

### Lessons from Week 1

**Edge Bias Discovery**:
- Stereo vision measures **edges** well, **centers** poorly
- Physics limitation (occlusion, surface orientation)
- **Not a bug!**

**Solutions Developed**:
1. Adaptive percentile (5% or 10% based on coverage)
2. YOLO for X,Y + ROI depth for Z
3. Accept limitation → design around it

## 8.2 Autonomous Vehicles

### Obstacle Detection

**Application**: ตรวจจับรถคัน, คนเดิน, สิ่งกีดขวาง

**Pipeline**:
```
Stereo Camera (forward-facing)
    ↓
Depth Map (0-50 meters)
    ↓
[Segmentation] → Road / Obstacle
    ↓
[Clustering] → Individual obstacles
    ↓
[Tracking] → Persistent IDs
    ↓
[Decision] → Brake / Steer / Continue
```

**Key Challenges**:
- Wide depth range (1m - 50m) → large numDisparities needed
- Real-time requirement (<100ms per frame)
- Must work in various lighting (day/night/rain)

**Solutions**:
- GPU acceleration (CUDA)
- Hierarchical approach (coarse→fine)
- Sensor fusion (stereo + LiDAR + radar)

## 8.3 Augmented Reality (AR)

### Depth-aware AR

**Application**: วางวัตถุ virtual ให้สมจริง (ถูกบังโดยวัตถุจริง)

```
Real world:          With AR:
   👤                  👤
  ╱│╲                ╱│╲
   │                  │ 🐉 ← Virtual dragon
  ╱ ╲                ╱ ╲
 ┬───┬             ┬───┬

Person occludes dragon → ต้องรู้ depth!
```

**Pipeline**:
```
Stereo Camera
    ↓
Depth Map (real world)
    ↓
[Place virtual object at Z_virtual]
    ↓
[Depth test]: if Z_real < Z_virtual → hide virtual
    ↓
Render (realistic occlusion)
```

**Example**:
```python
def render_with_occlusion(rgb_image, depth_map, virtual_object, z_virtual):
    """Render virtual object with occlusion"""
    # Render virtual object
    virtual_render = render_object(virtual_object, z_virtual)

    # Depth test
    for y, x in virtual_render.pixels:
        z_real = depth_map[y, x]

        if z_real > 0 and z_real < z_virtual:
            # Real object closer → occlude virtual
            continue
        else:
            # Virtual object visible
            rgb_image[y, x] = virtual_render[y, x]

    return rgb_image
```

## 8.4 3D Reconstruction

### Point Cloud Generation

**Application**: สร้างโมเดล 3D จากภาพ

```python
def generate_point_cloud(left_image, disparity, Q, color=True):
    """
    สร้าง point cloud จาก disparity map

    Args:
        left_image: RGB image (H×W×3)
        disparity: Disparity map (H×W)
        Q: Disparity-to-3D matrix (4×4)
        color: ใส่สีหรือไม่

    Returns:
        points_3D: (N, 3) array of [X, Y, Z]
        colors: (N, 3) array of [R, G, B] (ถ้า color=True)
    """
    # Reproject to 3D
    points_3D = cv2.reprojectImageTo3D(disparity, Q)

    # Valid mask (disparity > 0)
    mask = disparity > 0

    # Extract 3D points
    points = points_3D[mask]  # (N, 3)

    if color:
        # Extract colors
        colors = left_image[mask]  # (N, 3) in BGR
        colors = colors[:, ::-1]   # Convert to RGB
        return points, colors
    else:
        return points

# Save as PLY file
def save_ply(filename, points, colors=None):
    """Save point cloud as PLY file"""
    import open3d as o3d

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)

    o3d.io.write_point_cloud(filename, pcd)
    print(f"Saved {len(points)} points to {filename}")

# Usage
points, colors = generate_point_cloud(left_rgb, disparity, Q, color=True)
save_ply("pepper.ply", points, colors)
```

### Mesh Generation

**จาก Point Cloud → Mesh**:
```python
import open3d as o3d

# Load point cloud
pcd = o3d.io.read_point_cloud("pepper.ply")

# Estimate normals
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
)

# Poisson surface reconstruction
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    pcd, depth=9
)

# Remove low-density vertices
vertices_to_remove = densities < np.quantile(densities, 0.01)
mesh.remove_vertices_by_mask(vertices_to_remove)

# Save mesh
o3d.io.write_triangle_mesh("pepper_mesh.obj", mesh)
```

## 8.5 Industrial Inspection

### Defect Detection on Curved Surfaces

**Challenge**: ตรวจหารอยบุบบนพื้นผิวโค้ง (เช่น กระป๋อง, ขวด)

```python
def detect_dents(depth_map, expected_radius, tolerance=2):
    """
    ตรวจหารอยบุบจาก depth map

    Args:
        depth_map: Depth map ของพื้นผิวโค้ง (mm)
        expected_radius: รัศมีที่คาดหวัง (mm)
        tolerance: Tolerance (mm)

    Returns:
        dent_mask: Binary mask (True = dent)
    """
    # Fit ideal cylinder/sphere
    ideal_surface = fit_cylinder(depth_map, expected_radius)

    # Compute deviation
    deviation = depth_map - ideal_surface

    # Dents = depth ลึกกว่าที่คาดหวัง
    dent_mask = deviation < -tolerance

    return dent_mask

# Visualization
dent_mask = detect_dents(depth_map, expected_radius=50, tolerance=2)
defect_image = rgb_image.copy()
defect_image[dent_mask] = [255, 0, 0]  # Highlight in red
```

## 8.6 Best Practices

### 1. System Design

**Do's**:
- ✅ เข้าใจ limitations ของเซนเซอร์
- ✅ ออกแบบระบบรอบ limitations (เช่น YOLO+ROI แทนใช้ depth เพียงอย่างเดียว)
- ✅ เริ่มจาก simple → complex
- ✅ ทดสอบกับวัตถุจริงเร็วๆ (ไม่รอถึง Week 10!)

**Don'ts**:
- ❌ คาดหวัง perfect depth map (จะมี holes, noise)
- ❌ ใช้ depth ตรงๆ สำหรับวัตถุเล็ก (ใช้ YOLO+ROI แทน)
- ❌ ละเลย calibration quality

### 2. Parameter Tuning

**Systematic approach**:
1. เริ่มจาก default parameters
2. ทดสอบกับ pattern board (ground truth)
3. ปรับทีละพารามิเตอร์
4. Record results (before/after)
5. Validate กับวัตถุจริง

**Key parameters** (priority order):
1. **numDisparities** (มีผลมากที่สุด!)
2. blockSize
3. uniquenessRatio
4. P1, P2 (สำหรับ SGBM)

### 3. Troubleshooting

**Symptom**: Depth map มี holes เยอะ

**Possible causes**:
- Smooth surfaces (no texture)
- Lighting ไม่ดี
- uniquenessRatio สูงเกินไป
- numDisparities ต่ำเกินไป

**Solutions**:
1. เพิ่มแสง (diffuse, not direct)
2. ลด uniquenessRatio (10→5)
3. เพิ่ม numDisparities
4. ใช้ WLS filter
5. ยอมรับ (design around it)

---

**Symptom**: Depth inaccurate (error >5%)

**Possible causes**:
- Calibration spacing ผิด ⭐ (พบบ่อย!)
- numDisparities ต่ำเกินไป
- Lens distortion ไม่ได้แก้
- Pattern ไม่แบน

**Solutions**:
1. **วัด spacing อีกครั้ง** (จากกระดาษจริง!)
2. เพิ่ม numDisparities
3. Verify undistortion
4. ใช้ foam board/acrylic (not paper alone)

---

**Symptom**: Processing ช้า (>1s per frame)

**Solutions**:
1. ลด resolution (1280×720 → 640×480)
2. ใช้ StereoBM แทน SGBM
3. ไม่ใช้ WLS filter
4. Optimize code (vectorization)
5. ใช้ GPU (CUDA)

## 8.7 สรุปบทที่ 8

### Applications Summary

| Application | Depth Range | Key Challenges | Solutions |
|-------------|-------------|----------------|-----------|
| **Pepper Sorting** | 25-50 cm | Edge bias | Adaptive percentile |
| **Autonomous Vehicles** | 1-50 m | Real-time | GPU, hierarchical |
| **AR** | 0.5-5 m | Occlusion | Depth test |
| **3D Reconstruction** | Varied | Coverage | Multiple views |
| **Industrial Inspection** | 10-100 cm | Curved surfaces | Surface fitting |

### Design Principles

1. **Understand your sensor**: รู้ limitations → ออกแบบรอบมัน
2. **Test early**: ทดสอบกับวัตถุจริงเร็วๆ
3. **Iterate**: ปรับปรุงแบบทีละน้อย
4. **Document**: บันทึกทุกอย่าง (parameters, results, lessons)

---

# ภาคผนวก ก: ตัวอย่างโค้ด Python

## A.1 Complete Stereo Calibration

```python
#!/usr/bin/env python3
"""
Stereo Camera Calibration
สำหรับ Asymmetric Circles Grid pattern
"""

import cv2
import numpy as np
import glob
import yaml

# ==================== Configuration ====================
PATTERN_TYPE = 'asymmetric_circles'
ROWS = 5
COLS = 6
SPACING_MM = 18.0  # ⚠️ สำคัญ! วัดจากกระดาษจริง

CALIB_LEFT = 'calib_images/left/*.jpg'
CALIB_RIGHT = 'calib_images/right/*.jpg'
OUTPUT_FILE = 'stereo_calib.yaml'

# ==================== Functions ====================

def create_object_points(rows, cols, spacing_mm):
    """สร้าง object points สำหรับ asymmetric circles grid"""
    objp = np.zeros((rows * cols, 3), np.float32)

    for i in range(rows):
        for j in range(cols):
            objp[i * cols + j, 0] = (2 * j + i % 2) * spacing_mm
            objp[i * cols + j, 1] = i * spacing_mm
            objp[i * cols + j, 2] = 0

    return objp

def find_circles(image_path, pattern_size):
    """หา circles ใน image"""
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, centers = cv2.findCirclesGrid(
        gray,
        pattern_size,
        flags=cv2.CALIB_CB_ASYMMETRIC_GRID
    )

    if ret:
        # Draw circles (for debugging)
        cv2.drawChessboardCorners(img, pattern_size, centers, ret)

    return ret, centers, img

def calibrate_stereo():
    """Main calibration function"""

    # Pattern info
    pattern_size = (ROWS, COLS)
    objp = create_object_points(ROWS, COLS, SPACING_MM)

    # Storage
    objpoints = []
    imgpoints_left = []
    imgpoints_right = []

    # Load images
    images_left = sorted(glob.glob(CALIB_LEFT))
    images_right = sorted(glob.glob(CALIB_RIGHT))

    print(f"Found {len(images_left)} left images")
    print(f"Found {len(images_right)} right images")

    assert len(images_left) == len(images_right), "Mismatch image count!"

    # Process each pair
    for i, (img_L, img_R) in enumerate(zip(images_left, images_right)):
        print(f"\nProcessing pair {i+1}/{len(images_left)}...")

        ret_L, centers_L, _ = find_circles(img_L, pattern_size)
        ret_R, centers_R, _ = find_circles(img_R, pattern_size)

        if ret_L and ret_R:
            objpoints.append(objp)
            imgpoints_left.append(centers_L)
            imgpoints_right.append(centers_R)
            print(f"  ✓ Detected pattern successfully")
        else:
            print(f"  ✗ Pattern detection failed")
            if not ret_L:
                print(f"    - Left image failed")
            if not ret_R:
                print(f"    - Right image failed")

    print(f"\n{'='*60}")
    print(f"Valid image pairs: {len(objpoints)}/{len(images_left)}")

    if len(objpoints) < 10:
        print("ERROR: Not enough valid images (minimum 10)")
        return None

    # Get image size
    img = cv2.imread(images_left[0])
    image_size = (img.shape[1], img.shape[0])  # (width, height)

    print(f"Image size: {image_size}")

    # Calibrate left camera
    print(f"\n{'='*60}")
    print("Calibrating left camera...")
    ret_L, K_L, D_L, rvecs_L, tvecs_L = cv2.calibrateCamera(
        objpoints, imgpoints_left, image_size, None, None
    )
    print(f"  RMS Error: {ret_L:.4f} pixels")

    # Calibrate right camera
    print("\nCalibrating right camera...")
    ret_R, K_R, D_R, rvecs_R, tvecs_R = cv2.calibrateCamera(
        objpoints, imgpoints_right, image_size, None, None
    )
    print(f"  RMS Error: {ret_R:.4f} pixels")

    # Stereo calibration
    print(f"\n{'='*60}")
    print("Performing stereo calibration...")
    ret, K_L, D_L, K_R, D_R, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpoints_left,
        imgpoints_right,
        K_L, D_L,
        K_R, D_R,
        image_size,
        flags=cv2.CALIB_FIX_INTRINSIC
    )
    print(f"  Stereo RMS Error: {ret:.4f} pixels")
    print(f"  Baseline: {np.linalg.norm(T):.2f} mm")

    # Stereo rectification
    print("\nComputing rectification...")
    R_L, R_R, P_L, P_R, Q, roi_L, roi_R = cv2.stereoRectify(
        K_L, D_L,
        K_R, D_R,
        image_size,
        R, T,
        alpha=0  # Crop invalid pixels
    )

    # Save results
    print(f"\n{'='*60}")
    print(f"Saving calibration to {OUTPUT_FILE}...")

    calib_data = {
        'image_size': list(image_size),
        'K_left': K_L.tolist(),
        'D_left': D_L.tolist(),
        'K_right': K_R.tolist(),
        'D_right': D_R.tolist(),
        'R': R.tolist(),
        'T': T.tolist(),
        'E': E.tolist(),
        'F': F.tolist(),
        'R_left': R_L.tolist(),
        'R_right': R_R.tolist(),
        'P_left': P_L.tolist(),
        'P_right': P_R.tolist(),
        'Q': Q.tolist(),
        'roi_left': list(roi_L),
        'roi_right': list(roi_R),
        'rms_left': float(ret_L),
        'rms_right': float(ret_R),
        'rms_stereo': float(ret),
        'baseline_mm': float(np.linalg.norm(T)),
        'pattern': {
            'type': PATTERN_TYPE,
            'rows': ROWS,
            'cols': COLS,
            'spacing_mm': SPACING_MM
        }
    }

    with open(OUTPUT_FILE, 'w') as f:
        yaml.dump(calib_data, f, default_flow_style=False)

    print("  ✓ Saved successfully!")

    # Summary
    print(f"\n{'='*60}")
    print("CALIBRATION SUMMARY:")
    print(f"  Left Camera RMS:  {ret_L:.4f} pixels")
    print(f"  Right Camera RMS: {ret_R:.4f} pixels")
    print(f"  Stereo RMS:       {ret:.4f} pixels")
    print(f"  Baseline:         {np.linalg.norm(T):.2f} mm")
    print(f"  Images used:      {len(objpoints)}")
    print(f"{'='*60}")

    return calib_data

# ==================== Main ====================

if __name__ == '__main__':
    print("Stereo Camera Calibration")
    print("="*60)

    calib_data = calibrate_stereo()

    if calib_data:
        print("\n✓ Calibration completed successfully!")
    else:
        print("\n✗ Calibration failed!")
```

## A.2 Complete Depth Estimation

```python
#!/usr/bin/env python3
"""
Real-time Depth Estimation from Stereo Camera
"""

import cv2
import numpy as np
import yaml

# ==================== Configuration ====================
CALIB_FILE = 'stereo_calib.yaml'
CAMERA_LEFT = 0   # Camera ID or video file
CAMERA_RIGHT = 1

# Stereo matching parameters
STEREO_PARAMS = {
    'numDisparities': 512,
    'blockSize': 11,
    'P1': 8 * 3 * 11**2,
    'P2': 32 * 3 * 11**2,
    'disp12MaxDiff': 1,
    'uniquenessRatio': 10,
    'speckleWindowSize': 100,
    'speckleRange': 32
}

# ==================== Load Calibration ====================

def load_calibration(filename):
    """Load calibration data"""
    with open(filename, 'r') as f:
        calib = yaml.safe_load(f)

    # Convert lists to numpy arrays
    K_L = np.array(calib['K_left'])
    D_L = np.array(calib['D_left'])
    K_R = np.array(calib['K_right'])
    D_R = np.array(calib['D_right'])
    R_L = np.array(calib['R_left'])
    R_R = np.array(calib['R_right'])
    P_L = np.array(calib['P_left'])
    P_R = np.array(calib['P_right'])
    Q = np.array(calib['Q'])

    return K_L, D_L, K_R, D_R, R_L, R_R, P_L, P_R, Q

# ==================== Create Rectification Maps ====================

def create_rectification_maps(K_L, D_L, K_R, D_R, R_L, R_R, P_L, P_R, image_size):
    """Create rectification lookup tables"""
    map_L_x, map_L_y = cv2.initUndistortRectifyMap(
        K_L, D_L, R_L, P_L, image_size, cv2.CV_32FC1
    )
    map_R_x, map_R_y = cv2.initUndistortRectifyMap(
        K_R, D_R, R_R, P_R, image_size, cv2.CV_32FC1
    )
    return map_L_x, map_L_y, map_R_x, map_R_y

# ==================== Main ====================

def main():
    print("Loading calibration...")
    K_L, D_L, K_R, D_R, R_L, R_R, P_L, P_R, Q = load_calibration(CALIB_FILE)

    print("Opening cameras...")
    cap_L = cv2.VideoCapture(CAMERA_LEFT)
    cap_R = cv2.VideoCapture(CAMERA_RIGHT)

    # Get image size
    ret, frame = cap_L.read()
    if not ret:
        print("ERROR: Cannot read from left camera")
        return
    image_size = (frame.shape[1], frame.shape[0])

    print(f"Image size: {image_size}")

    # Create rectification maps
    print("Creating rectification maps...")
    map_L_x, map_L_y, map_R_x, map_R_y = create_rectification_maps(
        K_L, D_L, K_R, D_R, R_L, R_R, P_L, P_R, image_size
    )

    # Create stereo matcher
    print("Creating stereo matcher...")
    stereo = cv2.StereoSGBM_create(**STEREO_PARAMS)

    print("Starting real-time depth estimation...")
    print("  Press 'q' to quit")
    print("  Press 's' to save")

    while True:
        # Capture frames
        ret_L, frame_L = cap_L.read()
        ret_R, frame_R = cap_R.read()

        if not (ret_L and ret_R):
            break

        # Rectify
        rect_L = cv2.remap(frame_L, map_L_x, map_L_y, cv2.INTER_LINEAR)
        rect_R = cv2.remap(frame_R, map_R_x, map_R_y, cv2.INTER_LINEAR)

        # Convert to grayscale
        gray_L = cv2.cvtColor(rect_L, cv2.COLOR_BGR2GRAY)
        gray_R = cv2.cvtColor(rect_R, cv2.COLOR_BGR2GRAY)

        # Compute disparity
        disparity = stereo.compute(gray_L, gray_R).astype(np.float32) / 16.0

        # Compute depth
        points_3D = cv2.reprojectImageTo3D(disparity, Q)
        depth_map = points_3D[:, :, 2]  # Z coordinate (mm)
        depth_map[depth_map < 0] = 0    # Remove negative depths
        depth_map[depth_map > 2000] = 0 # Remove far points (>2m)

        # Depth map in cm
        depth_cm = depth_map / 10.0

        # Visualize disparity
        disp_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

        # Visualize depth
        depth_vis = cv2.normalize(depth_cm, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # Stack images
        top = np.hstack([rect_L, rect_R])
        bottom = np.hstack([disp_color, depth_color])
        combined = np.vstack([top, bottom])

        # Add text
        cv2.putText(combined, "Left", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "Right", (rect_L.shape[1] + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "Disparity", (10, rect_L.shape[0] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(combined, "Depth (cm)", (rect_L.shape[1] + 10, rect_L.shape[0] + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Show
        cv2.imshow('Stereo Vision', combined)

        # Keyboard
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('disparity.png', disp_color)
            cv2.imwrite('depth.png', depth_color)
            np.save('depth_map.npy', depth_cm)
            print("Saved images and depth map")

    cap_L.release()
    cap_R.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

---

# ภาคผนวก ข: พารามิเตอร์ที่ใช้งานจริง

## B.1 โปรเจค Pepper Sorting

### Camera Specifications
```yaml
camera_model: IMX219-83 Stereo Camera
sensor: Sony IMX219 (8MP)
resolution_max: 3280×2464
resolution_used: 1280×720  # หรือ 640×480 สำหรับ real-time
fps: 30
baseline_spec: 60 mm
baseline_measured: 60.57 mm
fov: 160° (diagonal, wide-angle)
focus: Manual (Left: 176.5, Right: 171.0)
```

### Calibration Results
```yaml
# Single camera (Left)
left_camera:
  rms: 0.22 pixels
  K:
    fx: 688.31
    fy: 688.31
    cx: 640.00
    cy: 360.00
  D: [-0.34218365, 0.11644646, -0.00079223, -0.00016717, 0.0]

# Single camera (Right)
right_camera:
  rms: 0.20 pixels
  K:
    fx: 688.28
    fy: 688.28
    cx: 641.12
    cy: 359.85
  D: [-0.34102873, 0.11598237, -0.00081045, -0.00015932, 0.0]

# Stereo
stereo:
  rms: 50.79 pixels  # High due to wide-angle (160° FOV)
  baseline: 60.57 mm
  working_range: 25-50 cm
  depth_accuracy: ±0.5 cm @ 32 cm
```

### Stereo Matching Parameters
```yaml
algorithm: StereoSGBM
resolution: 640×480  # For stability
numDisparities: 512  # 32×16
blockSize: 11
P1: 2904   # 8 × 3 × 11²
P2: 11616  # 32 × 3 × 11²
disp12MaxDiff: 1
uniquenessRatio: 10
speckleWindowSize: 100
speckleRange: 32
mode: SGBM_3WAY
wls_filter: false  # Not used (for speed/stability)
```

### Performance
```yaml
processing_time: ~500 ms/frame
depth_accuracy: ±0.5 cm @ 30-40 cm
repeatability: ±0.4 mm (15 tests)
coverage_peppers: 40-70%
coverage_pattern: 80-90%
```

---

# ภาคผนวก ค: คำถามและแบบฝึกหัด

## ค.1 คำถามทบทวน

### บทที่ 1-2: Camera Model

1. อธิบายความแตกต่างระหว่าง Pinhole Camera Model กับกล้องจริง
2. Focal length มีผลต่อ FOV (Field of View) อย่างไร?
3. ทำไม wide-angle lens มักมี barrel distortion?
4. Principal point สามารถเลื่อนจากจุดกึ่งกลางได้หรือไม่? เพราะเหตุใด?

### บทที่ 3: Calibration

5. ทำไมต้อง calibrate กล้อง? ยกตัวอย่างปัญหาที่เกิดถ้าไม่ calibrate
6. เปรียบเทียบ Checkerboard vs. Asymmetric Circles pattern
7. RMS reprojection error 0.5 px ถือว่าดีหรือไม่? อธิบาย
8. ถ้า spacing ผิด 20% จะส่งผลต่อ baseline และ depth อย่างไร?

### บทที่ 4-5: Stereo Geometry

9. อธิบาย Epipolar Constraint และประโยชน์ในการ matching
10. Rectification ทำให้ matching ง่ายขึ้นอย่างไร?
11. Q matrix ใช้ทำอะไร? สูตรหลักคืออะไร?

### บทที่ 6-7: Disparity & Matching

12. จาก `Z = (f × b) / d`, ถ้า d ลดครึ่ง, Z เปลี่ยนแปลงอย่างไร?
13. ทำไม depth error แปรผันตาม Z²?
14. เปรียบเทียบ StereoBM vs. StereoSGBM (ข้อดี/ข้อเสีย)
15. WLS filter ช่วยอะไร? Trade-off คืออะไร?

### บทที่ 8: Applications

16. ใน Pepper Sorting, ทำไมใช้ YOLO+ROI แทนการใช้ center depth ตรงๆ?
17. อธิบาย Edge Bias limitation และ solution ที่ใช้
18. Adaptive Percentile คืออะไร? ทำไมใช้ 5% หรือ 10%?

## ค.2 แบบฝึกหัดคำนวณ

### แบบฝึกหัดที่ 1: Depth Calculation

**กำหนด**:
- Focal length (f) = 700 pixels
- Baseline (b) = 65 mm
- Disparity (d) = 150 pixels

**คำนวณ**:
a) Depth (Z) ในหน่วย mm และ cm
b) ถ้า disparity มี error ±1 pixel, depth error เท่าไหร่?
c) ถ้า baseline เพิ่มเป็น 100 mm, depth เปลี่ยนแปลงอย่างไร?

**เฉลย**:
```
a) Z = (700 × 65) / 150
     = 45500 / 150
     = 303.33 mm
     = 30.33 cm

b) δZ = (Z² / (f × b)) × δd
     = (303.33² / (700 × 65)) × 1
     = (91969 / 45500) × 1
     = 2.02 mm

c) Z_new = (700 × 100) / 150
         = 70000 / 150
         = 466.67 mm
         = 46.67 cm

   เพิ่มขึ้น: 46.67 - 30.33 = 16.34 cm
   เปอร์เซ็นต์: 53.9% increase
```

### แบบฝึกหัดที่ 2: numDisparities

**กำหนด**:
- f × b = 45000 mm·px
- ระยะใกล้ที่สุด (Z_min) = 200 mm
- ระยะไกลสุด (Z_max) = 2000 mm

**คำนวณ**:
a) Disparity สูงสุด (d_max) ที่ Z_min
b) Disparity ต่ำสุด (d_min) ที่ Z_max
c) numDisparities ที่เหมาะสม (must be ×16)

**เฉลย**:
```
a) d_max = (f × b) / Z_min
         = 45000 / 200
         = 225 pixels

b) d_min = (f × b) / Z_max
         = 45000 / 2000
         = 22.5 pixels

c) Range = d_max - d_min
         = 225 - 22.5
         = 202.5 pixels

   Round up to multiple of 16:
   numDisparities = 208 (13×16)

   แต่แนะนำใช้: 256 (16×16) เพื่อความปลอดภัย
```

### แบบฝึกหัดที่ 3: Calibration Error

**กำหนด**:
- Pattern spacing (actual) = 20 mm
- Pattern spacing (used in code) = 25 mm
- Baseline (measured) = 75 mm

**คำนวณ**:
a) Scale error (%)
b) Baseline (corrected)
c) Depth error ถ้า measured depth = 400 mm

**เฉลย**:
```
a) Scale error = 25 / 20 = 1.25 (25% overestimate)

b) Baseline_correct = 75 / 1.25
                    = 60 mm

c) ถ้า code คิดว่า spacing = 25mm:
   → คำนวณ baseline = 75mm (ผิด)
   → คำนวณ depth = 400mm

   แต่ความจริง spacing = 20mm:
   → baseline_actual = 60mm
   → depth_actual = 400 / 1.25 = 320mm

   Error = 400 - 320 = 80mm (25% overestimate)
```

## ค.3 โปรเจคแนะนำ

### โปรเจคที่ 1: Basic Stereo Calibration
**ระดับ**: พื้นฐาน
**ระยะเวลา**: 1-2 สัปดาห์

**Tasks**:
1. พิมพ์ calibration pattern (checkerboard หรือ asymmetric circles)
2. Capture 20-30 calibration image pairs
3. เขียนโค้ด calibrate (ใช้ OpenCV)
4. ตรวจสอบ RMS error และ baseline
5. Visualize undistortion (เส้นตรง → ตรง?)

### โปรเจคที่ 2: Real-time Depth Estimation
**ระดับ**: ปานกลาง
**ระยะเวลา**: 2-3 สัปดาห์

**Tasks**:
1. ใช้ calibration จากโปรเจคที่ 1
2. Implement real-time stereo matching
3. แสดงผล disparity และ depth maps
4. ทดสอบความแม่นยำ (วัดระยะจริง vs. measured)
5. Optimize parameters สำหรับ scene ของคุณ

### โปรเจคที่ 3: Pepper Sorting (Advanced)
**ระดับ**: สูง
**ระยะเวลา**: 4-6 สัปดาห์

**Tasks**:
1. Collect pepper dataset (500+ images)
2. Train YOLO model (detection + classification)
3. Integrate YOLO + Depth estimation
4. Implement adaptive percentile method
5. Test กับพริกจริง → 3D positions

### โปรเจคที่ 4: 3D Reconstruction
**ระดับ**: ปานกลาง-สูง
**ระยะเวลา**: 3-4 สัปดาห์

**Tasks**:
1. Capture stereo pairs ของวัตถุ
2. Generate point cloud จาก depth map
3. Implement multi-view fusion (ถ้ามีหลายมุม)
4. Create mesh จาก point cloud
5. Visualize 3D model (Open3D / MeshLab)

---

**จบ Part 2**

**เอกสารทั้งหมด**:
- Part 1 (THEORY_STEREO_VISION.md): บทที่ 1-5
- Part 2 (THEORY_STEREO_VISION_PART2.md): บทที่ 6-8 + ภาคผนวก

**สรุปทฤษฎี**: ✅ Complete
**ตัวอย่างโค้ด**: ✅ Complete
**แบบฝึกหัด**: ✅ Complete

พร้อมใช้สอนแล้วครับ! 📚🎓
