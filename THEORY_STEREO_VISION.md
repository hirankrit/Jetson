# ทฤษฎีและหลักการ Stereo Vision
## สำหรับระบบ Pepper Sorting Robot

**ผู้เรียบเรียง**: ระบบ Pepper Sorting Robot Project
**จุดประสงค์**: เอกสารประกอบการสอนภาษาไทย
**ระดับ**: ปริญญาตรี-โท สาขา Computer Vision / Robotics

---

# สารบัญ

## ภาคทฤษฎี
- [บทที่ 1: แนะนำ Stereo Vision](#บทที่-1-แนะนำ-stereo-vision)
- [บทที่ 2: Camera Model และ Pinhole Camera](#บทที่-2-camera-model-และ-pinhole-camera)
- [บทที่ 3: Camera Calibration](#บทที่-3-camera-calibration)
- [บทที่ 4: Stereo Geometry และ Epipolar Geometry](#บทที่-4-stereo-geometry-และ-epipolar-geometry)
- [บทที่ 5: Stereo Rectification](#บทที่-5-stereo-rectification)
- [บทที่ 6: Disparity และ Depth Estimation](#บทที่-6-disparity-และ-depth-estimation)
- [บทที่ 7: Stereo Matching Algorithms](#บทที่-7-stereo-matching-algorithms)
- [บทที่ 8: การประยุกต์ใช้งาน](#บทที่-8-การประยุกต์ใช้งาน)

## ภาคปฏิบัติ
- [ภาคผนวก ก: ตัวอย่างโค้ด Python](#ภาคผนวก-ก-ตัวอย่างโค้ด-python)
- [ภาคผนวก ข: พารามิเตอร์ที่ใช้งานจริง](#ภาคผนวก-ข-พารามิเตอร์ที่ใช้งานจริง)
- [ภาคผนวก ค: คำถามและแบบฝึกหัด](#ภาคผนวก-ค-คำถามและแบบฝึกหัด)

---

# บทที่ 1: แนะนำ Stereo Vision

## 1.1 ความเป็นมา

### ระบบการมองเห็นของมนุษย์

มนุษย์มีความสามารถในการรับรู้ระยะทางและความลึก (depth perception) ของวัตถุโดยใช้ตาสองข้าง การทำงานร่วมกันของตาทั้งสองข้างทำให้สมองสามารถคำนวณตำแหน่ง 3 มิติของวัตถุได้อย่างแม่นยำ

**หลักการพื้นฐาน**:
- ตาซ้ายและตาขวาอยู่ห่างกันประมาณ 6-7 เซนติเมตร (baseline)
- แต่ละตาเห็นภาพจากมุมมองที่แตกต่างกันเล็กน้อย (binocular disparity)
- สมองประมวลผลความแตกต่างนี้เพื่อคำนวณระยะทาง

**ตัวอย่างการทดลอง**:
```
ลองชูนิ้วชี้ข้างหนึ่งขึ้นข้างหน้า จากนั้น:
1. ปิดตาซ้าย → เห็นนิ้วอยู่ตำแหน่งหนึ่ง
2. ปิดตาขวา → เห็นนิ้วเลื่อนตำแหน่ง (disparity)
3. เปิดสองตา → รู้สึกความลึก (depth)

ยิ่งนิ้วใกล้ → disparity มาก
ยิ่งนิ้วไกล → disparity น้อย
```

### Stereo Vision ในระบบคอมพิวเตอร์

**Stereo Vision** คือการนำหลักการการมองเห็นสามมิติของมนุษย์มาประยุกต์ใช้กับระบบคอมพิวเตอร์ โดยใช้กล้องสองตัวแทนตาสองข้าง

**องค์ประกอบพื้นฐาน**:
1. **กล้องซ้าย (Left Camera)**: ถ่ายภาพจากด้านซ้าย
2. **กล้องขวา (Right Camera)**: ถ่ายภาพจากด้านขวา
3. **Baseline (b)**: ระยะห่างระหว่างกล้องทั้งสอง
4. **ระบบประมวลผล**: คำนวณความแตกต่างของภาพเพื่อหาระยะทาง

```
      Camera L          Camera R
          👁️                👁️
          │                │
          ↓                ↓
      Image L          Image R
          ╲              ╱
           ╲            ╱
            ╲          ╱
             ↓        ↓
          Stereo Matching
                 ↓
            Disparity Map
                 ↓
             Depth Map (3D)
```

## 1.2 ข้อดีและข้อจำกัด

### ข้อดี ✅

1. **Passive Sensing**
   - ไม่ต้องส่งสัญญาณออกไป (เช่น laser, infrared)
   - ใช้แสงธรรมชาติหรือแสงสว่างทั่วไป
   - ไม่รบกวนระบบอื่น

2. **ต้นทุนต่ำ**
   - ใช้กล้องธรรมดา (ถูกกว่า LiDAR, ToF)
   - เหมาะสำหรับการผลิตจำนวนมาก

3. **ข้อมูลหนาแน่น (Dense)**
   - ได้ depth ทุก pixel (dense depth map)
   - เห็นรายละเอียดพื้นผิววัตถุ

4. **ความละเอียดปรับได้**
   - ขึ้นกับความละเอียดกล้อง
   - Scale ได้ตามต้องการ (เปลี่ยน baseline, focal length)

### ข้อจำกัด ⚠️

1. **ต้องการ Texture**
   - พื้นผิวเรียบ ไม่มีลาย → matching ยาก
   - วัตถุโปร่งใส, สะท้อนแสง → ผิดพลาดได้

2. **Occlusion (การบัง)**
   - พื้นที่ที่กล้องหนึ่งเห็น แต่อีกตัวไม่เห็น → ไม่มี depth
   - Edge และ center มี occlusion ต่างกัน (ดังที่พบใน Week 1!)

3. **Computational Cost**
   - Stereo matching ใช้เวลาประมวลผลมาก
   - Trade-off ระหว่างความเร็วและความแม่นยำ

4. **Baseline vs. Range Trade-off**
   - Baseline เล็ก → วัดระยะไกลไม่ได้แม่นยำ
   - Baseline ใหญ่ → occlusion เยอะ, matching ยาก

5. **ผลกระทบจากแสง**
   - แสงไม่สม่ำเสมอ → ภาพแตกต่างกัน → matching ผิดพลาด
   - ต้องควบคุมแสงให้ดี

## 1.3 การประยุกต์ใช้งาน

### 1. Robotics
- **Navigation**: หุ่นยนต์เดินหลบหลีกสิ่งกีดขวาง
- **Manipulation**: คีบจับวัตถุด้วยแขนกล
- **Sorting**: คัดแยกวัตถุตามขนาด/รูปร่าง (อย่างในโปรเจคนี้!)

### 2. Autonomous Vehicles
- **Obstacle Detection**: ตรวจจับรถคัน, คนเดิน, สิ่งกีดขวาง
- **Lane Detection**: หาเส้นแบ่งช่องทาง
- **Parking Assistance**: ช่วยจอดรถอัตโนมัติ

### 3. Augmented Reality (AR)
- **Depth Sensing**: รู้ระยะทางสำหรับวางวัตถุ virtual
- **Occlusion**: วัตถุจริงบังวัตถุ virtual ได้ถูกต้อง

### 4. 3D Reconstruction
- **3D Scanning**: สแกนวัตถุเพื่อสร้างโมเดล 3D
- **Cultural Heritage**: เก็บรักษามรดกทางวัฒนธรรม
- **Medical Imaging**: ภาพ 3D ของส่วนต่างๆ ของร่างกาย

### 5. Industrial Inspection
- **Quality Control**: ตรวจสอบขนาด, รูปร่าง
- **Defect Detection**: หาตำหนิบนพื้นผิว
- **Measurement**: วัดขนาดวัตถุแม่นยำ

## 1.4 ภาพรวม Pipeline

```
Input: กล้องซ้าย + กล้องขวา
  │
  ├─→ [1. Camera Calibration]
  │     └─→ Camera parameters (K, D, R, T)
  │
  ├─→ [2. Image Rectification]
  │     └─→ Aligned images (epipolar lines horizontal)
  │
  ├─→ [3. Stereo Matching]
  │     └─→ Disparity map (pixel shift)
  │
  ├─→ [4. Depth Computation]
  │     └─→ Depth map (metric distance)
  │
  └─→ [5. 3D Reconstruction]
        └─→ Point cloud (X, Y, Z)
```

**แต่ละขั้นตอนจะอธิบายโดยละเอียดในบทถัดไป**

---

# บทที่ 2: Camera Model และ Pinhole Camera

## 2.1 Pinhole Camera Model

### แนวคิดพื้นฐาน

**Pinhole Camera** (กล้องรูเข็ม) เป็นโมเดลทางคณิตศาสตร์ที่ใช้อธิบายการทำงานของกล้องอย่างง่าย

**หลักการ**:
- แสงจากวัตถุผ่านรูเล็กๆ (pinhole) เดียว
- ฉายภาพลงบนแผ่นรับภาพ (image plane)
- ภาพที่ได้จะกลับหัว (inverted)

```
World Coordinate (3D)         Image Coordinate (2D)

       P (X, Y, Z)
         │
         │  ╱ Ray of light
         │╱
        ╱│
       ╱ │
      ╱  │
     ┌───┼───┐
     │   O   │  ← Pinhole (optical center)
     └───┼───┘
         │╲
         │ ╲
         │  ╲
        ╱    ╲
       │      ╲
    Image      p (x, y)
    Plane       ↑
             Image point
```

### พิกัดและความสัมพันธ์

**World Coordinate System (พิกัดโลก)**:
- (X, Y, Z): ตำแหน่งจริงของวัตถุในโลก 3 มิติ
- หน่วย: เมตร, เซนติเมตร, มิลลิเมตร

**Camera Coordinate System (พิกัดกล้อง)**:
- (Xc, Yc, Zc): ตำแหน่งเมื่อแปลงมาอยู่ในระบบพิกัดของกล้อง
- Origin (0, 0, 0) = Optical center
- Z-axis = กล้องชี้ไปข้างหน้า

**Image Coordinate System (พิกัดภาพ)**:
- (x, y): ตำแหน่งบนภาพ (หน่วย: pixel)
- Origin (0, 0) = มุมบนซ้ายของภาพ
- x → ไปทางขวา, y → ลงล่าง

## 2.2 Camera Projection (การฉายภาพ)

### Perspective Projection

จุด P(X, Y, Z) ในโลก 3D ฉายเป็นจุด p(x, y) บนภาพ 2D ตามสูตร:

**สูตรพื้นฐาน**:
```
x = f × (X / Z)
y = f × (Y / Z)
```

**โดยที่**:
- `f` = Focal length (ความยาวโฟกัส) หน่วยเดียวกับ X, Y, Z
- `Z` = ระยะห่างจากกล้องถึงวัตถุ (depth)

**ตัวอย่างที่ 1**: คำนวณตำแหน่งบนภาพ

```
กำหนด:
  วัตถุอยู่ที่ P = (10 cm, 5 cm, 50 cm) ในพิกัดกล้อง
  Focal length f = 4 mm = 0.4 cm

คำนวณ:
  x = 0.4 × (10 / 50) = 0.4 × 0.2 = 0.08 cm
  y = 0.4 × (5 / 50)  = 0.4 × 0.1 = 0.04 cm

ตำแหน่งบนภาพ = (0.08 cm, 0.04 cm)
```

### Homogeneous Coordinates (พิกัดเนื้อเดียวกัน)

เพื่อให้เขียนในรูปเมทริกซ์ได้สะดวก เราใช้ **Homogeneous Coordinates**:

**จาก Cartesian**:
```
(X, Y, Z) → [X, Y, Z, 1]ᵀ  (เพิ่ม 1 ต่อท้าย)
(x, y)    → [x, y, 1]ᵀ
```

**Projection Matrix**:
```
[x']   [fx  0  cx  0] [X]
[y'] = [0  fy  cy  0] [Y]
[z']   [0   0   1  0] [Z]
                      [1]

จากนั้น:
  x = x' / z'
  y = y' / z'
```

**พารามิเตอร์**:
- `fx, fy`: Focal length ในหน่วย pixel (แกน x, y)
- `cx, cy`: Principal point (จุดกึ่งกลางภาพ)

## 2.3 Camera Intrinsic Matrix (เมทริกซ์ภายใน)

### Intrinsic Parameters (พารามิเตอร์ภายใน)

**Camera Intrinsic Matrix K** เป็นเมทริกซ์ที่บรรจุพารามิเตอร์ภายในของกล้อง:

```
    [fx  s  cx]
K = [0  fy  cy]
    [0   0   1]
```

**พารามิเตอร์แต่ละตัว**:

1. **fx, fy**: Focal length (pixel)
   - `fx = F × mx` (F = focal length (mm), mx = pixel size x (mm/pixel))
   - `fy = F × my`
   - กล้องที่ดีมักมี `fx ≈ fy` (pixel สี่เหลี่ยมจัตุรัส)

2. **cx, cy**: Principal point (pixel)
   - จุดที่แกนกล้อง (optical axis) ตัดกับ image plane
   - ในอุดมคติ: `cx ≈ width/2, cy ≈ height/2`
   - ในความเป็นจริง: เลื่อนเล็กน้อย (misalignment)

3. **s**: Skew coefficient
   - แสดงความไม่ตั้งฉากของแกน x, y บนเซนเซอร์
   - กล้องยุคใหม่: `s ≈ 0` (negligible)

### ตัวอย่างค่า K จริง

**IMX219-83 Stereo Camera (โปรเจคนี้)**:
```python
K_left = [
  [688.31,   0.00, 640.00],  # fx=688.31, cx=640
  [  0.00, 688.31, 360.00],  # fy=688.31, cy=360
  [  0.00,   0.00,   1.00]
]

# Resolution: 1280×720
# cx ≈ 1280/2 = 640 ✅
# cy ≈ 720/2 = 360 ✅
# fx ≈ fy ✅ (pixel สี่เหลี่ยมจัตุรัส)
```

## 2.4 Lens Distortion (ความผิดเพี้ยนจากเลนส์)

### ประเภทของ Distortion

กล้องจริงมีเลนส์ → เกิดความผิดเพี้ยน (distortion) → ภาพไม่ตรงตามโมเดล pinhole

**ประเภทหลัก**:

#### 1. Radial Distortion (ความผิดเพี้ยนรัศมี)

**สาเหตุ**: รูปร่างของเลนส์โค้ง

**ชนิด**:
- **Barrel Distortion** (ถังบวม): เส้นตรงดูโค้งออก
  - เกิดกับ Wide-angle lens (FOV กว้าง)
- **Pincushion Distortion** (หมอนยุบ): เส้นตรงดูโค้งเข้า
  - เกิดกับ Telephoto lens (zoom เยอะ)

```
Barrel           Ideal          Pincushion
┌─────┐         ┌─────┐         ┌─────┐
│╭───╮│         │┌───┐│         │ ┌─┐ │
││   ││   vs.   ││   ││   vs.   │ │ │ │
│╰───╯│         │└───┘│         │ └─┘ │
└─────┘         └─────┘         └─────┘
  wide           ideal          telephoto
```

**สูตรแก้**:
```
r² = x² + y²  (ระยะจากจุดกึ่งกลาง)

x_corrected = x × (1 + k₁r² + k₂r⁴ + k₃r⁶)
y_corrected = y × (1 + k₁r² + k₂r⁴ + k₃r⁶)
```

**พารามิเตอร์**:
- `k₁, k₂, k₃`: Radial distortion coefficients

#### 2. Tangential Distortion (ความผิดเพี้ยนสัมผัส)

**สาเหตุ**: เลนส์และเซนเซอร์ไม่ขนานกัน

**สูตรแก้**:
```
x_corrected = x + [2p₁xy + p₂(r² + 2x²)]
y_corrected = y + [p₁(r² + 2y²) + 2p₂xy]
```

**พารามิเตอร์**:
- `p₁, p₂`: Tangential distortion coefficients

### Distortion Coefficients Vector

**OpenCV Format**:
```
D = [k₁, k₂, p₁, p₂, k₃]
```

**ตัวอย่างจากโปรเจค**:
```python
D_left = [-0.34218365,  # k₁ (negative → barrel distortion)
           0.11644646,  # k₂
          -0.00079223,  # p₁ (small → negligible)
          -0.00016717,  # p₂ (small → negligible)
           0.0]         # k₃ (often zero)

# k₁ < 0 → กล้องมี barrel distortion
# สอดคล้องกับ wide-angle lens (160° FOV)
```

### การแก้ Distortion

**Undistortion Process**:
```python
import cv2

# Input
distorted_image = ...  # ภาพที่ผิดเพี้ยน
K = ...               # Intrinsic matrix
D = ...               # Distortion coefficients

# Undistort
undistorted_image = cv2.undistort(distorted_image, K, D)

# Result: เส้นตรงในโลกจริง → เส้นตรงในภาพ ✅
```

## 2.5 Camera Extrinsic Parameters (พารามิเตอร์ภายนอก)

### Rotation และ Translation

**Extrinsic Parameters** บอกตำแหน่งและการหมุนของกล้องในโลก

**องค์ประกอบ**:
1. **Rotation Matrix R** (3×3):
   - การหมุนจากพิกัดโลก → พิกัดกล้อง
   - Orthogonal matrix (R⁻¹ = Rᵀ)

2. **Translation Vector T** (3×1):
   - การเลื่อนจากพิกัดโลก → พิกัดกล้อง
   - T = [tx, ty, tz]ᵀ

### Transformation (การแปลงพิกัด)

**จากพิกัดโลก → พิกัดกล้อง**:
```
[Xc]   [R | T] [Xw]
[Yc] = [---|--] [Yw]
[Zc]   [0 | 1] [Zw]
[1 ]            [1 ]

หรือ:
Pc = R × Pw + T
```

**จากพิกัดกล้อง → พิกัดภาพ**:
```
[x]       [Xc]
[y] = K × [Yc]  (แล้วหาร z)
[z]       [Zc]
```

**Full Pipeline**:
```
[x]         [Xw]
[y] = K[R|T][Yw]
[z]         [Zw]
            [1 ]
```

## 2.6 สรุปบทที่ 2

### สูตรสำคัญ

| สูตร | ความหมาย |
|------|----------|
| `p = K[R\|T]P` | World → Image (full) |
| `x = fx(X/Z) + cx` | 3D → 2D (x coordinate) |
| `y = fy(Y/Z) + cy` | 3D → 2D (y coordinate) |
| `Z = (fx × b) / d` | Disparity → Depth |

### พารามิเตอร์ที่ต้อง Calibrate

**Intrinsic** (ภายใน):
- K: Camera matrix (fx, fy, cx, cy)
- D: Distortion coefficients (k₁, k₂, p₁, p₂, k₃)

**Extrinsic** (ภายนอก):
- R: Rotation matrix
- T: Translation vector

**Stereo-Specific**:
- Baseline (b): ระยะห่างระหว่างกล้อง

### คำถามทบทวน

1. Focal length มีผลต่อภาพอย่างไร?
2. ทำไม wide-angle lens มัก barrel distortion?
3. ถ้า Z เพิ่มขึ้น 2 เท่า, x บนภาพเปลี่ยนแปลงอย่างไร?
4. Principal point เลื่อนจากกึ่งกลางได้หรือไม่? ทำไม?

---

# บทที่ 3: Camera Calibration

## 3.1 ความหมายและความสำคัญ

### Camera Calibration คืออะไร?

**Camera Calibration** คือกระบวนการหาพารามิเตอร์ภายใน (intrinsic) และความผิดเพี้ยน (distortion) ของกล้อง เพื่อให้สามารถแปลงระหว่างพิกัดโลก (3D) และพิกัดภาพ (2D) ได้อย่างแม่นยำ

**วัตถุประสงค์**:
1. หาค่า K (intrinsic matrix)
2. หาค่า D (distortion coefficients)
3. สำหรับ stereo: หาความสัมพันธ์ระหว่าง 2 กล้อง (R, T)

### ทำไมต้อง Calibrate?

**ปัญหาถ้าไม่ Calibrate**:
```
❌ ภาพผิดเพี้ยน (distortion) → เส้นตรงดูโค้ง
❌ ไม่รู้ focal length → วัดขนาดผิด
❌ ไม่รู้ principal point → ตำแหน่งผิด
❌ สำหรับ stereo: ไม่รู้ baseline → depth ผิด
```

**ประโยชน์ของ Calibration**:
```
✅ แก้ distortion → ภาพตรง
✅ รู้ K → แปลง pixel ↔ metric ได้
✅ รู้ D → undistort ภาพได้
✅ รู้ R, T → คำนวณ depth ได้แม่นยำ
```

## 3.2 Calibration Patterns

### ประเภท Pattern

#### 1. Checkerboard Pattern (หมากรุกดำ-ขาว)

**คุณสมบัติ**:
- Grid สี่เหลี่ยมสลับสี (ดำ-ขาว)
- หา corners ระหว่างสี่เหลี่ยม
- ง่าย, ใช้ทั่วไป

```
  0   1   2   3   4   5
  ┌───┬───┬───┬───┬───┐
0 │███│   │███│   │███│
  ├───┼───┼───┼───┼───┤
1 │   │███│   │███│   │
  ├───┼───┼───┼───┼───┤
2 │███│   │███│   │███│
  └───┴───┴───┴───┴───┘

○ = Corners to detect
```

**ข้อดี**:
- ✅ ง่าย, พิมพ์ง่าย
- ✅ Algorithm มีใน OpenCV

**ข้อเสีย**:
- ⚠️ ไวต่อแสงไม่สม่ำเสมอ
- ⚠️ ไม่มี unique orientation

#### 2. Asymmetric Circles Grid (วงกลมแบบ asymmetric)

**คุณสมบัติ**:
- วงกลมเรียงแบบ offset (row เลื่อนครึ่งช่อง)
- หา centroids ของวงกลม
- Sub-pixel accuracy สูง

```
Row 0:  ○     ○     ○     ○
Row 1:    ○     ○     ○
Row 2:  ○     ○     ○     ○
Row 3:    ○     ○     ○
Row 4:  ○     ○     ○     ○

Pattern: 5 rows × 6 cols (แต่ละ row ต่าง)
```

**ข้อดี**:
- ✅ ทนต่อแสงไม่สม่ำเสมอดีกว่า
- ✅ Sub-pixel accuracy สูง (circle centroid)
- ✅ Unique orientation (ไม่สับสน)
- ✅ เหมาะกับ wide-angle lens

**ข้อเสีย**:
- ⚠️ พิมพ์ยากกว่า checkerboard เล็กน้อย
- ⚠️ ต้องเข้าใจ spacing (ดังที่กล่าวใน Week 1!)

**ทำไมเลือก Asymmetric Circles?** (โปรเจคนี้)
```
เหตุผล:
1. งานเกษตร → แสงไม่สม่ำเสมอ → ต้องการ pattern ทนทาน ✅
2. Wide-angle lens (160° FOV) → barrel distortion สูง → circles แก้ดีกว่า squares ✅
3. วัตถุเล็ก (พริก) → ต้องการ sub-pixel accuracy ✅
```

### การเลือก Pattern Size

**พารามิเตอร์**:
- **Rows × Columns**: จำนวนจุดที่ detect
- **Spacing**: ระยะห่างระหว่างจุด (mm)
- **Total size**: ขนาดรวมของ pattern (mm)

**คำแนะนำ**:
```
จำนวนจุด:
  - Minimum: 6×8 = 48 จุด
  - Recommended: 7×9 = 63 จุด หรือมากกว่า
  - โปรเจคนี้: 5×6 = 33 วงกลม (พอเพียง)

Spacing:
  - ขึ้นกับขนาดพื้นที่ทำงาน
  - กล้องความละเอียดสูง → spacing เล็กได้
  - โปรเจคนี้: 18mm (พื้นที่ 320mm height)

Total size:
  - ไม่เกินพื้นที่ที่กล้องมองเห็นได้ในครั้งเดียว
  - ควรเต็ม 50-80% ของภาพ
```

## 3.3 Single Camera Calibration

### ขั้นตอนการ Calibrate

#### Step 1: Collect Calibration Images

**วิธีการ**:
1. พิมพ์ pattern บนกระดาษ A4 หรือใหญ่กว่า
2. ติดบนแผ่นแข็ง (foam board / acrylic) เพื่อให้แบน
3. ถ่ายภาพ pattern จาก **หลายมุมมอง, หลายระยะ**

**จำนวนภาพ**:
- Minimum: 10-15 ภาพ
- Recommended: 20-30 ภาพ
- โปรเจคนี้: 40 ภาพ (เกินเป้า!)

**หลักการถ่าย**:
```
Orientation:
  - ตรง (0°)
  - เอียงซ้าย-ขวา (±30°)
  - เอียงบน-ล่าง (±30°)
  - หมุน (±45°)

Distance:
  - ใกล้ (เต็มภาพ)
  - กลาง (70-80% ภาพ)
  - ไกล (50-60% ภาพ)

Position:
  - กึ่งกลาง
  - มุมต่างๆ (top-left, top-right, ฯลฯ)

เป้าหมาย: Coverage ทั่วภาพ!
```

#### Step 2: Detect Pattern Points

**สำหรับ Checkerboard**:
```python
import cv2
import numpy as np

# Pattern size (จำนวน corners, ไม่ใช่จำนวนสี่เหลี่ยม!)
pattern_size = (8, 5)  # 9×6 squares = 8×5 corners

# Find corners
ret, corners = cv2.findChessboardCorners(
    image_gray,
    pattern_size,
    flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
)

if ret:
    # Refine corners ให้แม่นยำ (sub-pixel)
    corners = cv2.cornerSubPix(
        image_gray,
        corners,
        (11, 11),  # Window size
        (-1, -1),  # Zero zone (no dead region)
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    )
```

**สำหรับ Asymmetric Circles**:
```python
# Pattern size (rows, columns)
pattern_size = (5, 6)  # 5 rows, 6 columns = 33 circles

# Find circles
ret, centers = cv2.findCirclesGrid(
    image_gray,
    pattern_size,
    flags=cv2.CALIB_CB_ASYMMETRIC_GRID
)

# Centers ได้จาก centroid → sub-pixel accuracy อยู่แล้ว!
```

#### Step 3: Prepare Object Points

**Object Points** คือตำแหน่ง 3D จริงของแต่ละจุดบน pattern

**สำหรับ Asymmetric Circles (spacing = 18mm)**:
```python
def create_object_points(rows, cols, spacing_mm):
    """
    สร้าง object points สำหรับ asymmetric circles grid

    Pattern layout:
    Row 0: (0,0)   (36,0)   (72,0)   ...
    Row 1:   (18,18)  (54,18)  (90,18) ...
    Row 2: (0,36)  (36,36)  (72,36)  ...
    """
    objp = np.zeros((rows * cols, 3), np.float32)

    for i in range(rows):
        for j in range(cols):
            # x = (2j + i%2) × spacing
            # y = i × spacing
            # z = 0 (pattern บนพื้นราบ)
            objp[i * cols + j, 0] = (2 * j + i % 2) * spacing_mm
            objp[i * cols + j, 1] = i * spacing_mm
            objp[i * cols + j, 2] = 0

    return objp

# ตัวอย่าง: 5×6, spacing=18mm
objp = create_object_points(5, 6, 18)

# objp[0] = [0, 0, 0]       # Row 0, Col 0
# objp[1] = [36, 0, 0]      # Row 0, Col 1
# objp[6] = [18, 18, 0]     # Row 1, Col 0
# objp[7] = [54, 18, 0]     # Row 1, Col 1
# ...
```

**🚨 ข้อสำคัญ**: `spacing_mm` ต้องเป็นค่าที่ถูกต้อง!
- ถ้า spacing ผิด → scale ผิด → ระยะทางทุกอย่างผิด!
- **ต้องวัดจากกระดาษที่พิมพ์จริง** ไม่ใช่ใช้ค่าจากโปรแกรมเลย!

#### Step 4: Calibrate Camera

**OpenCV Function**:
```python
import cv2

# รวบรวมข้อมูลจากทุกภาพ
object_points = []  # 3D points in real world
image_points = []   # 2D points in images

for each image:
    objp = create_object_points(...)
    ret, corners = cv2.findCirclesGrid(...)

    if ret:
        object_points.append(objp)
        image_points.append(corners)

# Calibrate!
ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
    object_points,
    image_points,
    image_size,
    None,  # K initial guess (None = auto)
    None,  # D initial guess (None = auto)
    flags=0
)

print(f"RMS Reprojection Error: {ret:.4f} pixels")
print(f"Camera Matrix K:\n{K}")
print(f"Distortion Coefficients D:\n{D}")
```

**Output**:
- `ret`: RMS reprojection error (ควรต่ำกว่า 1 pixel)
- `K`: Camera intrinsic matrix (3×3)
- `D`: Distortion coefficients (5×1 หรือ 8×1)
- `rvecs`: Rotation vectors สำหรับแต่ละภาพ
- `tvecs`: Translation vectors สำหรับแต่ละภาพ

### การประเมินคุณภาพ Calibration

#### 1. RMS Reprojection Error

**ความหมาย**: ความผิดพลาดเฉลี่ยของการฉายจุด 3D กลับมาเป็น 2D

**สูตร**:
```
RMS = sqrt(Σ||p_detected - p_projected||² / N)

โดยที่:
  p_detected = จุดที่ detect ได้จากภาพ
  p_projected = จุดที่คำนวณจาก K, D, R, T
  N = จำนวนจุดทั้งหมด
```

**เกณฑ์**:
```
RMS < 0.5 px  → Excellent ✅
RMS < 1.0 px  → Good ✅
RMS < 2.0 px  → Acceptable ⚠️
RMS ≥ 2.0 px  → Poor, ควร re-calibrate ❌
```

**โปรเจคนี้**:
```
Left Camera:  RMS = 0.22 px (Excellent!)
Right Camera: RMS = 0.20 px (Excellent!)
```

#### 2. Visual Inspection

**วิธีตรวจสอบ**:
```python
# Undistort ภาพ
undistorted = cv2.undistort(image, K, D)

# ตรวจสอบ:
# 1. เส้นตรงในโลกจริง → ตรงในภาพหรือไม่?
# 2. มุมห้อง (90°) → 90° ในภาพหรือไม่?
# 3. Pattern board → เป็นสี่เหลี่ยมผืนผ้าหรือไม่?
```

#### 3. Reprojection Error per Image

**ตรวจสอบว่าภาพไหนมีปัญหา**:
```python
for i in range(len(object_points)):
    imgpoints2, _ = cv2.projectPoints(
        object_points[i], rvecs[i], tvecs[i], K, D
    )
    error = cv2.norm(image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    print(f"Image {i}: Error = {error:.4f} px")
```

**ถ้าภาพไหน error สูง (>2px)**:
- ลบภาพนั้นออก
- Re-calibrate ใหม่

## 3.4 Stereo Camera Calibration

### ความแตกต่างจาก Single Camera

**Single Camera Calibration**:
- หา K, D ของกล้องตัวเดียว

**Stereo Calibration**:
- หา K_left, D_left
- หา K_right, D_right
- หา R, T (ความสัมพันธ์ระหว่าง 2 กล้อง)

### ขั้นตอน Stereo Calibration

#### Step 1: Calibrate แต่ละกล้อง

```python
# กล้องซ้าย
ret_L, K_L, D_L, rvecs_L, tvecs_L = cv2.calibrateCamera(
    objpoints, imgpoints_left, image_size, None, None
)

# กล้องขวา
ret_R, K_R, D_R, rvecs_R, tvecs_R = cv2.calibrateCamera(
    objpoints, imgpoints_right, image_size, None, None
)
```

#### Step 2: Stereo Calibration

```python
# Stereo calibrate (หา R, T)
ret, K_L, D_L, K_R, D_R, R, T, E, F = cv2.stereoCalibrate(
    objpoints,
    imgpoints_left,
    imgpoints_right,
    K_L, D_L,
    K_R, D_R,
    image_size,
    flags=cv2.CALIB_FIX_INTRINSIC  # Fix K, D (ใช้ค่าจาก single calib)
)

print(f"Stereo RMS Error: {ret:.4f} pixels")
print(f"Baseline: {np.linalg.norm(T):.2f} mm")
```

**Output เพิ่มเติม**:
- `R`: Rotation matrix (3×3) จาก left → right
- `T`: Translation vector (3×1) จาก left → right
- `E`: Essential matrix (เกี่ยวข้องกับ epipolar geometry)
- `F`: Fundamental matrix (เกี่ยวข้องกับ epipolar geometry)

**Baseline**:
```python
baseline = np.linalg.norm(T)  # ||T|| = ระยะห่างระหว่าง 2 กล้อง

# โปรเจคนี้: baseline = 60.57 mm
# Spec: 60 mm → Error 0.95% ✅
```

### Stereo Rectification

**วัตถุประสงค์**: ทำให้ epipolar lines เป็นแนวนอน (horizontal) → matching ง่ายขึ้น

```python
R_L, R_R, P_L, P_R, Q, roi_L, roi_R = cv2.stereoRectify(
    K_L, D_L,
    K_R, D_R,
    image_size,
    R, T,
    alpha=0  # 0 = crop เฉพาะ valid pixels, 1 = keep all pixels
)
```

**Output**:
- `R_L, R_R`: Rectification rotation matrices
- `P_L, P_R`: Projection matrices ใหม่ (3×4)
- `Q`: Disparity-to-depth mapping matrix (4×4)
- `roi_L, roi_R`: Region of interest (valid pixels)

**Q Matrix** (สำคัญมาก!):
```
    [1  0   0      -cx]
Q = [0  1   0      -cy]
    [0  0   0       f ]
    [0  0  -1/Tx  cy_diff/Tx]

โดยที่:
  f = focal length
  Tx = baseline (ระยะห่างระหว่างกล้อง)
  cx, cy = principal point

ใช้แปลง disparity → depth:
  Z = Q[2,3] / disparity = (f × Tx) / d
```

## 3.5 ปัญหาที่พบบ่อยและแนวทางแก้ไข

### ปัญหาที่ 1: RMS Error สูง (>2 pixels)

**สาเหตุ**:
- ภาพบางภาพมีปัญหา (motion blur, out of focus)
- Pattern detection ผิดพลาด
- Spacing ผิด

**แก้ไข**:
1. ตรวจสอบ error per image
2. ลบภาพที่ error สูง
3. เพิ่มจำนวนภาพที่ดี
4. ตรวจสอบ spacing อีกครั้ง

### ปัญหาที่ 2: Baseline ผิดมาก

**สาเหตุ**:
- ❌ **Spacing ผิด!** (พบบ่อยที่สุด!)
- Pattern ไม่แบน (โค้งไป)
- ภาพน้อยเกินไป

**ตัวอย่าง (จากโปรเจค)**:
```
Spacing = 25mm (ผิด) → Baseline = 436mm ❌
Spacing = 18mm (ถูก)  → Baseline = 60mm ✅

Scale error = 25/18 = 1.389 (39% ผิด!)
```

**แก้ไข**:
1. **วัด spacing จากกระดาษจริง**
2. วัดหลายๆ จุด แล้วเฉลี่ย
3. ใช้ไม้บรรทัดที่แม่นยำ

### ปัญหาที่ 3: Stereo RMS สูง แต่ Single RMS ต่ำ

**สาเหตุ**:
- Wide-angle lens → barrel distortion สูง
- กล้อง 2 ตัวไม่ sync กันพอดี

**ตัวอย่าง (จากโปรเจค)**:
```
Left RMS:   0.22 px ✅
Right RMS:  0.20 px ✅
Stereo RMS: 50.79 px ⚠️ (สูงแต่ยอมรับได้!)
```

**ทำไมยอมรับได้?**
- Wide-angle lens (160° FOV) → normal!
- Baseline ถูกต้อง (60mm) → metric scale OK
- Depth accuracy ทดสอบแล้ว (±0.5cm) → ใช้งานได้

### ปัญหาที่ 4: Pattern Detection ล้มเหลว

**สาเหตุ**:
- แสงไม่เพียงพอ / มากเกินไป
- Pattern out of focus
- Pattern ไม่เต็มใน FOV กล้อง
- Configuration ผิด (rows, cols)

**แก้ไข**:
1. ปรับแสง (brightness, contrast)
2. ปรับ focus กล้อง
3. ถ่ายให้ pattern เต็ม 50-80% ของภาพ
4. ตรวจสอบ (rows, cols) ให้ถูกต้อง

## 3.6 สรุปบทที่ 3

### Checklist การ Calibrate

**ก่อน Calibrate**:
- [ ] เลือก pattern เหมาะสม (asymmetric circles แนะนำ)
- [ ] พิมพ์และติดบนแผ่นแข็ง
- [ ] **วัด spacing จริง** (สำคัญมาก!)
- [ ] เตรียมแสงให้เหมาะสม

**ระหว่าง Calibrate**:
- [ ] ถ่ายภาพหลากหลาย (20-40 ภาพ)
- [ ] ครอบคลุมทุกพื้นที่ของภาพ
- [ ] เช็ค pattern detection real-time

**หลัง Calibrate**:
- [ ] RMS < 1 px ✅
- [ ] Baseline ตรงกับ spec (±5mm)
- [ ] Visual inspection (เส้นตรง → ตรง)
- [ ] ทดสอบ depth accuracy

### พารามิเตอร์ที่ได้

**จาก Single Calibration**:
```python
K_left = [[fx, 0, cx],
          [0, fy, cy],
          [0,  0,  1]]

D_left = [k1, k2, p1, p2, k3]
```

**จาก Stereo Calibration**:
```python
R = Rotation matrix (3×3)
T = Translation vector (3×1)
baseline = ||T||
Q = Disparity-to-depth matrix (4×4)
```

### บทต่อไป

- บทที่ 4: Stereo Geometry (เข้าใจ R, T, epipolar geometry)
- บทที่ 5: Rectification (ทำไมต้อง rectify?)
- บทที่ 6: Depth Estimation (จาก disparity → depth)

---

# บทที่ 4: Stereo Geometry และ Epipolar Geometry

## 4.1 Stereo Geometry Basics

### Coordinate Systems

**ระบบพิกัดใน Stereo Vision**:
```
World Coordinate (W):
  - จุดเริ่มต้น: ตำแหน่งใดก็ได้ในโลก
  - ใช้: (Xw, Yw, Zw)

Left Camera Coordinate (CL):
  - จุดเริ่มต้น: Optical center ของกล้องซ้าย
  - ใช้: (XL, YL, ZL)

Right Camera Coordinate (CR):
  - จุดเริ่มต้น: Optical center ของกล้องขวา
  - ใช้: (XR, YR, ZR)
```

**Transformation Chain**:
```
World → Left Camera:
  PL = R_L × Pw + T_L

Left Camera → Right Camera:
  PR = R × PL + T

โดยที่:
  R = Rotation matrix (3×3)
  T = Translation vector (3×1)
  ||T|| = Baseline
```

### Baseline and Depth Relationship

**Baseline (b)**: ระยะห่างระหว่าง optical centers ของ 2 กล้อง

```
        CL              CR
        👁️──────b──────👁️
         │╲          ╱│
         │ ╲        ╱ │
         │  ╲      ╱  │
         │   ╲    ╱   │
         │    ╲  ╱    │
         │     ╲╱     │
         │      P     │
              (X,Y,Z)
```

**ความสัมพันธ์**:
- Baseline ใหญ่ → วัดระยะไกลได้แม่นยำ
- Baseline เล็ก → วัดระยะใกล้ได้แม่นยำ

**โปรเจคนี้**:
- Baseline = 60mm
- ระยะทำงาน = 25-50cm
- → เหมาะสม ✅

## 4.2 Epipolar Geometry

### Epipolar Constraint

**Epipolar Geometry** อธิบายความสัมพันธ์ทางเรขาคณิตระหว่างภาพซ้ายและภาพขวา

**องค์ประกอบ**:

```
      CL              CR
      ○──────────────○   Baseline
      │╲            ╱│
      │ ╲          ╱ │
      │  ╲    P   ╱  │   ← จุดในโลก 3D
      │   ╲  ●   ╱   │
      │    ╲   ╱    │
     ╱│─────╲─╱─────│╲
    ╱ │  pL  ╳  pR  │ ╲
   ╱  │     ╱ ╲     │  ╲
  ╱───┼────╱───╲────┼───╲
 Image Left   Image Right

Epipolar line: เส้นตรงที่เชื่อม pL และ epipole
```

**Definitions**:

1. **Epipole (eL, eR)**:
   - จุดที่ baseline ตัดกับ image plane
   - eL = ตำแหน่งของ CR เมื่อฉายลงบน Image Left
   - eR = ตำแหน่งของ CL เมื่อฉายลงบน Image Right

2. **Epipolar Line**:
   - เส้นตรงที่เชื่อม epipole กับจุดบนภาพ
   - ถ้า pL อยู่ที่ไหน → pR ต้องอยู่บน epipolar line ที่สอดคล้อง

3. **Epipolar Plane**:
   - ระนาบที่ผ่าน CL, CR, และ P
   - ตัด image planes ได้ epipolar lines

### Epipolar Constraint Equation

**ถ้า pL ↔ pR สอดคล้องกัน (same 3D point P)**:
```
pR^T × F × pL = 0

โดยที่:
  F = Fundamental matrix (3×3)
  pL = [x_L, y_L, 1]^T (homogeneous coordinates)
  pR = [x_R, y_R, 1]^T
```

**Fundamental Matrix F**:
- เมทริกซ์ 3×3 ที่อธิบายความสัมพันธ์ epipolar
- หาได้จาก stereo calibration
- Rank = 2 (singular)

**Essential Matrix E**:
```
E = K_R^T × F × K_L

หรือ:
E = [T]× × R

โดยที่:
  [T]× = Skew-symmetric matrix ของ T
  R = Rotation matrix
```

### การใช้ Epipolar Constraint

**ประโยชน์**:
1. **ลด search space**:
   - ไม่ต้องหา correspondence ทั้งภาพ (2D search)
   - หาเฉพาะบน epipolar line (1D search)

2. **Reject outliers**:
   - ถ้า pR ไม่อยู่บน epipolar line → ไม่ใช่ correspondence ที่ถูก

3. **Rectification**:
   - แปลงภาพให้ epipolar lines เป็นแนวนอน → matching ง่ายขึ้น

## 4.3 Triangulation (การสร้างจุด 3D)

### หลักการ

**Triangulation** คือการหาตำแหน่ง 3D (P) จากจุด 2D บนภาพ 2 ภาพ (pL, pR)

```
        CL              CR
        ●───────────────●
         ╲             ╱
          ╲   Ray L   ╱ Ray R
           ╲         ╱
            ╲       ╱
             ╲     ╱
              ╲   ╱
               ╲ ╱
                ● P (X, Y, Z)
```

**สูตร (อย่างง่าย)**:

ถ้า epipolar lines เป็นแนวนอน (rectified):
```
Disparity (d) = x_L - x_R  (pixel shift)

Depth:
  Z = (f × b) / d

จากนั้นคำนวณ X, Y:
  X = (x_L - cx) × Z / f
  Y = (y_L - cy) × Z / f
```

### Linear Triangulation

**สำหรับ general case** (ไม่ rectified):

```python
def triangulate_point(pL, pR, P_L, P_R):
    """
    Linear triangulation (DLT method)

    Args:
        pL: [x, y, 1] in left image
        pR: [x, y, 1] in right image
        P_L: Projection matrix left (3×4)
        P_R: Projection matrix right (3×4)

    Returns:
        P: [X, Y, Z, 1] in world/camera coordinate
    """
    # สร้าง A matrix
    A = np.array([
        pL[1] * P_L[2,:] - P_L[1,:],
        P_L[0,:] - pL[0] * P_L[2,:],
        pR[1] * P_R[2,:] - P_R[1,:],
        P_R[0,:] - pR[0] * P_R[2,:]
    ])

    # Solve A × P = 0 using SVD
    _, _, Vt = np.linalg.svd(A)
    P = Vt[-1, :]
    P = P / P[3]  # Normalize

    return P[:3]  # [X, Y, Z]
```

**OpenCV Function**:
```python
import cv2

# Triangulate หลายจุด
points_4D = cv2.triangulatePoints(P_L, P_R, ptsL, ptsR)

# Convert จาก homogeneous coordinates
points_3D = points_4D[:3, :] / points_4D[3, :]
```

### Reprojection Error

**การตรวจสอบ**:

หลังจาก triangulate ได้ P แล้ว ฉาย P กลับมาเป็น pL', pR':
```
pL' = P_L × P
pR' = P_R × P
```

**Reprojection Error**:
```
error_L = ||pL - pL'||
error_R = ||pR - pR'||

Total error = sqrt(error_L² + error_R²)
```

**เกณฑ์**:
- Error < 1 px → Good ✅
- Error < 2 px → Acceptable ⚠️
- Error ≥ 2 px → Bad ❌

## 4.4 Rectification (การจัดภาพให้ตรง)

### ทำไมต้อง Rectify?

**ปัญหาภาพดิบ (unrectified)**:
- Epipolar lines ไม่ horizontal → search 2D
- Correspondence ยาก, ช้า

**หลัง Rectification**:
- Epipolar lines horizontal ✅
- Correspondence → search 1D (แนวนอน) ✅
- เร็วกว่ามาก!

```
Before Rectification:          After Rectification:

Left:         Right:          Left:         Right:
  ╱              ╲              ─              ─
 ╱                ╲             ─              ─
╱    Epipolar      ╲            ─  Horizontal ─
     lines                      ─    lines     ─
    ไม่ horizontal               ─              ─
```

### Rectification Process

**ขั้นตอน**:
1. คำนวณ Rectification rotation matrices: R_L, R_R
2. คำนวณ Projection matrices ใหม่: P_L, P_R
3. Remap images: ใช้ R_L, R_R แปลงภาพ

**ผลลัพธ์**:
- Epipolar lines → horizontal
- Corresponding points → same y coordinate
- Matching: search แนวนอนเท่านั้น (ที่ y เดียวกัน)

**OpenCV Implementation**:
```python
# Rectify (จาก stereo calibration)
R_L, R_R, P_L, P_R, Q, roi_L, roi_R = cv2.stereoRectify(
    K_L, D_L,
    K_R, D_R,
    image_size,
    R, T,
    alpha=0
)

# สร้าง rectification maps
map_L_x, map_L_y = cv2.initUndistortRectifyMap(
    K_L, D_L, R_L, P_L, image_size, cv2.CV_32FC1
)
map_R_x, map_R_y = cv2.initUndistortRectifyMap(
    K_R, D_R, R_R, P_R, image_size, cv2.CV_32FC1
)

# Remap (แปลงภาพ)
rect_L = cv2.remap(img_L, map_L_x, map_L_y, cv2.INTER_LINEAR)
rect_R = cv2.remap(img_R, map_R_x, map_R_y, cv2.INTER_LINEAR)

# ตรวจสอบ: วาดเส้นแนวนอน
for y in range(0, height, 30):
    cv2.line(rect_L, (0, y), (width, y), (0, 255, 0), 1)
    cv2.line(rect_R, (0, y), (width, y), (0, 255, 0), 1)

# จุดบน rect_L ที่ y=100 → จุดที่สอดคล้องบน rect_R ก็ต้อง y=100 ✅
```

## 4.5 สรุปบทที่ 4

### Epipolar Geometry - สรุปสั้น

| Concept | ความหมาย | ประโยชน์ |
|---------|----------|----------|
| **Epipolar Line** | เส้นที่ pR ต้องอยู่ถ้า pL คือ correspondence | ลด search space |
| **Fundamental Matrix F** | อธิบายความสัมพันธ์ epipolar | หา epipolar lines |
| **Essential Matrix E** | F แบบ calibrated | หา R, T |
| **Triangulation** | หา 3D จาก 2D correspondence | สร้าง point cloud |
| **Rectification** | ทำให้ epipolar lines เป็นแนวนอน | Matching เร็วขึ้น |

### สูตรสำคัญ

**Epipolar Constraint**:
```
pR^T × F × pL = 0
```

**Depth from Disparity**:
```
Z = (f × b) / d
```

**3D Point from Disparity**:
```
X = (x - cx) × Z / f
Y = (y - cy) × Z / f
Z = (f × b) / d
```

### คำถามทบทวน

1. ทำไม baseline ใหญ่เกินไปไม่ดี?
2. Rectification ช่วยเรื่องอะไร?
3. Triangulation คืออะไร?
4. Epipolar line เป็นแนวนอนหมายความว่าอย่างไร?

---

# บทที่ 5: Stereo Rectification

## 5.1 ทบทวนและวัตถุประสงค์

### ปัญหาของภาพที่ยัง Unrectified

**ลักษณะ**:
- กล้องซ้ายและขวาไม่ parallel กัน
- Epipolar lines เป็นเส้นเอียง, โค้ง
- Corresponding points อาจอยู่ที่ y ต่างกัน

**ผลกระทบ**:
```
สำหรับจุด pL ที่ (x=100, y=50):
  → pR อาจอยู่ที่ (x=?, y=?) ใดก็ได้บน epipolar line
  → ต้อง search 2D ❌ ช้า, ซับซ้อน
```

### วัตถุประสงค์ของ Rectification

**เป้าหมาย**:
1. ทำให้ image planes ของ 2 กล้อง **coplanar** (อยู่บนระนาบเดียวกัน)
2. ทำให้ epipolar lines เป็น **horizontal** (แนวนอน)
3. ทำให้ corresponding points อยู่ที่ **y เดียวกัน**

**ผลลัพธ์**:
```
สำหรับจุด pL ที่ (x=100, y=50):
  → pR ต้องอยู่ที่ (x=?, y=50) เท่านั้น
  → ต้อง search 1D (แนวนอน) ✅ เร็ว, ง่าย
```

## 5.2 หลักการ Rectification

### Rotation ของแต่ละกล้อง

**แนวคิด**:
- หมุนกล้องซ้ายด้วย R_L
- หมุนกล้องขวาด้วย R_R
- ทำให้ optical axes ขนานกัน

```
Before:                     After:
  CL       CR                 CL ──→  CR ──→
   ╲      ╱                   (parallel)
    ╲    ╱
     ╲  ╱
      ╲╱

Not parallel ❌            Parallel ✅
```

### Coordinate System ใหม่

**Rectified Coordinate System**:
```
x-axis: ทิศทาง baseline (CL → CR)
z-axis: ทิศทางกล้องชี้ (perpendicular to baseline)
y-axis: ตั้งฉากกับ x และ z (right-hand rule)
```

**Rotation Matrix**:
```
       [e1x  e1y  e1z]     [r1x  r1y  r1z]
R_new =[e2x  e2y  e2z]  =  [r2x  r2y  r2z]
       [e3x  e3y  e3z]     [r3x  r3y  r3z]

โดยที่:
  e1 = T / ||T||        (x-axis: baseline direction)
  e2 = e3 × e1          (y-axis: ตั้งฉาก)
  e3 = arbitrary        (z-axis: average of optical axes)
```

### Projection Matrix หลัง Rectification

**Left Camera**:
```
    [fx  0  cx  0  ]
PL =[0  fy  cy  0  ]
    [0   0   1  0  ]

(เหมือน K แต่ขยาย 3×4)
```

**Right Camera**:
```
    [fx  0  cx  -fx*Tx]
PR =[0  fy  cy   0    ]
    [0   0   1   0    ]

Tx = ||T|| = baseline
```

**สังเกต**:
- `PL[:3, :3] = PR[:3, :3]` (intrinsics เหมือนกัน)
- `PR[0, 3] = -fx * Tx` (ขวาเลื่อนจากซ้าย Tx)

## 5.3 การคำนวณ Rectification Maps

### Rectification Mapping

**วิธีการ**:
1. Rectify: หา R_L, R_R, P_L, P_R
2. Create Maps: สร้าง lookup tables (map_x, map_y)
3. Remap: ใช้ maps แปลงภาพ

**OpenCV Implementation**:
```python
import cv2
import numpy as np

# Step 1: Rectify
R_L, R_R, P_L, P_R, Q, roi_L, roi_R = cv2.stereoRectify(
    cameraMatrix1=K_L,
    distCoeffs1=D_L,
    cameraMatrix2=K_R,
    distCoeffs2=D_R,
    imageSize=image_size,  # (width, height)
    R=R,                    # จาก stereo calibration
    T=T,
    alpha=0,  # 0 = crop invalid pixels, 1 = keep all
    newImageSize=image_size
)

# Step 2: Create rectification maps
map_L_x, map_L_y = cv2.initUndistortRectifyMap(
    cameraMatrix=K_L,
    distCoeffs=D_L,
    R=R_L,           # Rectification rotation
    newCameraMatrix=P_L,
    size=image_size,
    m1type=cv2.CV_32FC1
)

map_R_x, map_R_y = cv2.initUndistortRectifyMap(
    cameraMatrix=K_R,
    distCoeffs=D_R,
    R=R_R,
    newCameraMatrix=P_R,
    size=image_size,
    m1type=cv2.CV_32FC1
)

# Step 3: Remap images
rectified_L = cv2.remap(
    src=image_L,
    map1=map_L_x,
    map2=map_L_y,
    interpolation=cv2.INTER_LINEAR
)

rectified_R = cv2.remap(
    src=image_R,
    map1=map_R_x,
    map2=map_R_y,
    interpolation=cv2.INTER_LINEAR
)
```

### การตรวจสอบ Rectification

**วิธีที่ 1: วาดเส้นแนวนอน**
```python
# วาดเส้นแนวนอน
for y in range(0, height, 30):
    cv2.line(rectified_L, (0, y), (width, y), (0, 255, 0), 1)
    cv2.line(rectified_R, (0, y), (width, y), (0, 255, 0), 1)

# Stack ภาพ 2 ภาพแนวนอน
combined = np.hstack([rectified_L, rectified_R])
cv2.imshow('Rectified Stereo', combined)

# ตรวจสอบ:
# วัตถุเดียวกันบนภาพซ้ายและขวาควรอยู่บนเส้นแนวนอนเดียวกัน ✅
```

**วิธีที่ 2: เช็ค y-coordinate ของ corresponding points**
```python
# หาจุดเด่น (features)
pts_L = cv2.goodFeaturesToTrack(rectified_L_gray, ...)
pts_R = cv2.goodFeaturesToTrack(rectified_R_gray, ...)

# Match features
matches = ...  # correspondence

# ตรวจสอบ y-coordinate
for match in matches:
    pt_L = pts_L[match.queryIdx]
    pt_R = pts_R[match.trainIdx]

    y_diff = abs(pt_L[1] - pt_R[1])

    if y_diff < 2:  # ต่างกันน้อยกว่า 2 pixels
        print(f"✅ Good: y_diff = {y_diff:.2f}")
    else:
        print(f"❌ Bad: y_diff = {y_diff:.2f}")
```

## 5.4 Q Matrix (Disparity-to-Depth Mapping)

### ความหมายและความสำคัญ

**Q Matrix** (4×4) ใช้แปลง disparity map → 3D point cloud

**รูปแบบ**:
```
    [1   0   0      -cx    ]
Q = [0   1   0      -cy    ]
    [0   0   0       f     ]
    [0   0  -1/Tx  (cx'-cx)/Tx]

โดยที่:
  f = focal length (fx หรือ fy)
  Tx = baseline
  cx, cy = principal point (left)
  cx' = principal point (right)
```

### การใช้ Q Matrix

**แปลง Disparity → 3D**:
```python
# มี disparity map (H×W)
disparity = ...  # คำนวณจาก stereo matching

# แปลงเป็น 3D point cloud
points_3D = cv2.reprojectImageTo3D(
    disparity=disparity,
    Q=Q,
    handleMissingValues=True
)

# points_3D shape = (H, W, 3) = (X, Y, Z) สำหรับแต่ละ pixel

# ตัวอย่าง: ดึง depth ที่ pixel (x=100, y=50)
X = points_3D[50, 100, 0]
Y = points_3D[50, 100, 1]
Z = points_3D[50, 100, 2]

print(f"3D position: ({X:.2f}, {Y:.2f}, {Z:.2f}) mm")
```

**สูตรโดยตรง**:
```
จาก disparity d ที่ pixel (x, y):

[X]       [x - cx          ]
[Y]       [y - cy          ]
[Z] = Q × [d              ]
[W]       [1              ]

แล้วหาร X, Y, Z ด้วย W

หรือง่ายกว่า (สมมุติ rectified):
  Z = f × Tx / d
  X = (x - cx) × Z / f
  Y = (y - cy) × Z / f
```

### ตัวอย่างการคำนวณ

**กำหนด (จากโปรเจคจริง)**:
```
fx = 688.31 px
cx = 640 px
cy = 360 px
Tx = 60.57 mm  (baseline)

pixel (x=400, y=300) มี disparity = 200 px
```

**คำนวณ 3D**:
```
Z = (688.31 × 60.57) / 200
  = 41699.85 / 200
  = 208.50 mm
  ≈ 20.85 cm

X = (400 - 640) × 208.50 / 688.31
  = -240 × 208.50 / 688.31
  = -72.70 mm
  = -7.27 cm  (ซ้ายจากจุดกึ่งกลาง)

Y = (300 - 360) × 208.50 / 688.31
  = -60 × 208.50 / 688.31
  = -18.17 mm
  = -1.82 cm  (เหนือจุดกึ่งกลาง)

ตำแหน่ง 3D = (-7.27 cm, -1.82 cm, 20.85 cm)
```

## 5.5 ROI (Region of Interest)

### ความหมาย

หลัง rectification บางพื้นที่ของภาพอาจไม่ valid (ไม่มีข้อมูล) เพราะการหมุนและการ crop

**ROI**: พื้นที่ที่ valid ใช้งานได้

```
Full Image:             Valid ROI:
┌──────────────┐       ┌──────────────┐
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓│       │▓▓▓┌────────┐▓│
│▓▓▓┌────────┐▓│  →    │▓▓▓│ Valid  │▓│
│▓▓▓│ Valid  │▓│       │▓▓▓│  Area  │▓│
│▓▓▓│  Area  │▓│       │▓▓▓└────────┘▓│
│▓▓▓└────────┘▓│       └──────────────┘
│▓▓▓▓▓▓▓▓▓▓▓▓▓▓│
└──────────────┘
▓ = Invalid pixels
```

**Alpha Parameter**:
```
alpha = 0:
  - Crop ทิ้งพื้นที่ invalid ทั้งหมด
  - ได้ภาพเล็กลง แต่ทุก pixel valid

alpha = 1:
  - Keep all pixels
  - ได้ภาพเท่าเดิม แต่มีพื้นที่ invalid

alpha = 0.5:
  - Compromise ระหว่าง 2 แบบ
```

### การใช้ ROI

```python
# จาก stereoRectify
roi_L = ...  # (x, y, width, height)
roi_R = ...

# Crop เฉพาะ ROI
x, y, w, h = roi_L
rectified_L_cropped = rectified_L[y:y+h, x:x+w]

x, y, w, h = roi_R
rectified_R_cropped = rectified_R[y:y+h, x:x+w]

# ใช้งาน cropped images สำหรับ stereo matching
```

## 5.6 สรุปบทที่ 5

### ขั้นตอน Rectification

1. **Stereo Calibrate**: หา R, T
2. **Stereo Rectify**: หา R_L, R_R, P_L, P_R, Q
3. **Create Maps**: สร้าง map_x, map_y
4. **Remap Images**: แปลงภาพให้ rectified
5. **Verify**: ตรวจสอบ epipolar lines เป็นแนวนอน

### ผลลัพธ์ที่ได้

| Before Rectification | After Rectification |
|----------------------|---------------------|
| ❌ Epipolar lines เอียง | ✅ Epipolar lines แนวนอน |
| ❌ Search 2D | ✅ Search 1D |
| ❌ y อาจต่างกัน | ✅ y เท่ากัน (correspondence) |
| ❌ ช้า | ✅ เร็ว |

### Parameters สำคัญ

| Parameter | ความหมาย | ค่าทั่วไป |
|-----------|----------|-----------|
| `alpha` | Cropping behavior | 0 (crop all invalid) |
| `Q` | Disparity→3D matrix | จาก stereoRectify |
| `ROI` | Valid image region | จาก stereoRectify |

### บทต่อไป

- บทที่ 6: Disparity และ Depth Estimation (หา disparity จาก rectified images)
- บทที่ 7: Stereo Matching Algorithms (StereoSGBM, StereoBM)

---

**(จะมีอีก 3 บท: Disparity & Depth, Stereo Matching, และ Applications - ต่อในข้อความถัดไป)**
