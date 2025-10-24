# 📷 IMX219 Stereo Camera Calibration Guide

## Asymmetric Circles Pattern (แนะนำสำหรับงานเกษตร)

### ทำไมใช้ Asymmetric Circles?

✅ **ข้อดี:**
- ทนทานต่อแสงไม่สม่ำเสมอ (เหมาะกับ natural lighting)
- Sub-pixel accuracy สูงกว่า checkerboard
- Unique pattern → ไม่มีปัญหา ambiguity
- เหมาะกับวัตถุเล็กๆ เช่น พริก, ผลไม้

📐 **Pattern Specifications:**
```
Type: Asymmetric Circles Grid
Rows: 5
Columns: 13
Diagonal Spacing: 18mm
Circle Diameter: 14mm
Physical Size: ~234mm × 72mm (พอดี A4 แนวนอน)
```

---

## 🖨️ Step 1: พิมพ์ Calibration Pattern

### Option 1: ใช้ Online Generator (แนะนำ)

1. ไปที่ https://calib.io/pages/camera-calibration-pattern-generator

2. เลือก **"Asymmetric Circles Grid"**

3. ตั้งค่า:
   - **Rows**: 5
   - **Columns**: 13
   - **Spacing**: 18mm
   - **Circle Diameter**: 14mm
   - **Target Width**: A4 (210mm × 297mm)

4. กด **Download PDF**

5. พิมพ์ออกมา:
   - ใช้กระดาษ A4 ขาว (แนะนำกระดาษหนา 120g หรือมากกว่า)
   - ตั้งค่า printer: **Actual Size** (ไม่ใช่ "Fit to page")
   - ตรวจสอบขนาดด้วยไม้บรรทัด: ระยะห่างต้องเท่ากับ 18mm จริงๆ

### Option 2: ใช้ OpenCV Script

```python
# generate_pattern.py
import cv2

pattern = cv2.aruco.GridBoard_create(
    markersX=13,
    markersY=5,
    markerLength=14,
    markerSeparation=4,
    dictionary=cv2.aruco.DICT_4X4_50
)
```

---

## 📋 Step 2: เตรียม Calibration Board

1. **ติดกระดาษบนแผ่นแข็ง:**
   - ใช้ foam board, acrylic, หรือ cardboard แข็งๆ
   - ติดให้แน่น ไม่ย่น ไม่โค้ง
   - ขอบกระดาษต้องไม่ยื่นออกมา

2. **ตรวจสอบความแบน:**
   - วางบน flat surface
   - ต้องไม่โก่งหรือบิดเบี้ยว

3. **เพิ่มความคมชัด (optional):**
   - เคลือบด้วย laminate film (แนะนำ)
   - ช่วยป้องกันเปื้อน และเพิ่มความคมชัด

---

## 📸 Step 3: Capture Calibration Images

### Setup

```bash
# รันสคริปต์เก็บภาพ
python3 capture_calibration.py
```

### Tips สำหรับการถ่ายภาพ Calibration ที่ดี

✅ **DO:**
- ถ่ายอย่างน้อย **20-30 ภาพคู่**
- ครอบคลุม **ทุกมุมของ field of view**
- ครอบคลุม **หลายระยะห่าง** (40-80cm)
- ครอบคลุม **หลายมุม** (เอียง, หมุน, ชิด edge)
- ถือ board ให้นิ่งตอนกด 'c'
- แสงสว่างพอ สม่ำเสมอ

❌ **DON'T:**
- ถ่ายแต่มุมเดียวซ้ำๆ
- Board เบลอหรือเคลื่อนไหว
- Pattern โดนบดบังบางส่วน
- แสงสะท้อนจ้า (glare)
- เงาทับ pattern

### การจัด Pose ที่ดี

```
ตัวอย่าง 30 poses:

1-5:   ตรงกลาง, ระยะต่างกัน (40, 50, 60, 70, 80cm)
6-10:  เอียงซ้าย-ขวา (±15°, ±30°)
11-15: เอียงบน-ล่าง (±15°, ±30°)
16-20: หมุน (0°, 45°, 90°, 135°, 180°)
21-25: ชิดขอบ (บน, ล่าง, ซ้าย, ขวา, มุม)
26-30: มุมประกอบ (เอียง + หมุน)
```

### Controls

- **'c'** : Capture (ถ่ายภาพได้เมื่อ detect pattern ทั้ง 2 กล้อง)
- **'q'** : Quit

### ผลลัพธ์

ภาพจะถูกบันทึกที่:
```
calib_images/
├── left/
│   ├── img_000_20251023_150530.jpg
│   ├── img_001_20251023_150535.jpg
│   └── ...
└── right/
    ├── img_000_20251023_150530.jpg
    ├── img_001_20251023_150535.jpg
    └── ...
```

---

## 🧮 Step 4: Run Calibration

```bash
# คำนวณ calibration parameters
python3 stereo_calibration.py
```

### ผลลัพธ์ที่ได้

1. **stereo_calib.yaml** - Calibration parameters
   - Camera matrices (intrinsic)
   - Distortion coefficients
   - Rotation & Translation (extrinsic)
   - Q matrix (สำหรับ depth calculation)

2. **rectification_maps.npz** - Rectification maps
   - ใช้ rectify images แบบเร็ว (pre-computed)

### การประเมินคุณภาพ

**Stereo RMS Error:**
- < 0.5 pixels → ดีเยี่ยม ✓
- 0.5 - 1.0 → ดี ✓
- 1.0 - 2.0 → พอใช้
- > 2.0 → ควรทำใหม่

**Baseline:**
- IMX219-83: ~60mm
- เหมาะกับ working distance 40-100cm

---

## 🧪 Step 5: Test Depth Map (Coming Soon)

```bash
# ทดสอบ depth accuracy
python3 test_depth_map.py
```

จะแสดง:
- Rectified stereo images
- Disparity map
- Depth map
- 3D point cloud

---

## 🔧 Troubleshooting

### Pattern ไม่ detect

**สาเหตุ:**
- แสงน้อยเกินไป → เพิ่มแสง
- Pattern เบลอ → Focus camera, ถือให้นิ่ง
- Board บิดงอ → ใช้แผ่นแข็งกว่า
- ระยะไม่เหมาะ → ลองระยะ 50-70cm

**แก้ไข:**
```python
# ใน capture_calibration.py แก้ blob detector params:
params.minArea = 30      # ลดถ้า circles เล็กเกินไป
params.maxArea = 8000    # เพิ่มถ้า circles ใหญ่เกินไป
params.minCircularity = 0.7  # ลดถ้า detect ยาก
```

### Calibration RMS Error สูง (> 2.0)

**สาเหตุ:**
- ภาพไม่ครอบคลุมทั่ว FOV
- ภาพซ้ำกันมากเกินไป (มุมเดียวกัน)
- Board ไม่ flat
- ขนาด print ไม่ตรงกับ spec

**แก้ไข:**
- ลบภาพที่ไม่ดีออก
- เก็บภาพเพิ่มจากมุมที่ขาดหายไป
- ตรวจสอบขนาด pattern ด้วยไม้บรรทัด
- ใช้ board ที่แข็งกว่า

### Depth ไม่แม่นยำ

**สาเหตุ:**
- Calibration ไม่ดี → ทำใหม่
- Lighting เปลี่ยนแปลง → Recalibrate
- Camera moved → Check mounting

---

## 📊 Expected Results

หลัง calibration สำเร็จ คุณจะสามารถ:

✅ คำนวณ **depth (Z)** ของวัตถุใน working space
✅ คำนวณ **3D position (X, Y, Z)** ของพริก
✅ ความแม่นยำ depth ± 5-10mm ที่ระยะ 50-70cm
✅ ใช้ใน vision system ต่อไปได้

---

## 📚 References

- OpenCV Stereo Calibration: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
- Camera Calibration Pattern Generator: https://calib.io/
- Asymmetric Circles vs Checkerboard: https://calib.io/blogs/knowledge-base/calibration-best-practices

---

## 🎯 Next Steps

หลังจาก calibration เสร็จ:

1. ✅ Test depth accuracy (test_depth_map.py)
2. 📊 สร้างรายงาน Week 1
3. 🎨 เริ่ม Week 2: Dataset collection
4. 🤖 Train YOLO model

---

**Good luck with calibration! 🚀**
