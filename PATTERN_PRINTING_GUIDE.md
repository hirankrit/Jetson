# 📐 Calibration Pattern Printing Guide

**คู่มือการพิมพ์ Pattern สำหรับ Stereo Camera Calibration**

---

## 📦 ไฟล์ Pattern ที่สร้างให้

### Pattern Files:

**1. `calibration_pattern_18mm.svg` (แนะนำ - spacing ปัจจุบัน)**
- Rows: 5
- Columns: 6
- Spacing: **18mm** (diagonal)
- Circle diameter: 14mm
- Size: 252 × 126 mm

**2. `calibration_pattern_16mm.svg` (สำรองในกรณีที่ pattern เดิมเป็น 16mm)**
- Rows: 5
- Columns: 6
- Spacing: **16mm** (diagonal)
- Circle diameter: 14mm
- Size: 230 × 118 mm

---

## 🖨️ วิธีการพิมพ์ (CRITICAL!)

### ⚠️ **สิ่งสำคัญที่สุด: SPACING ต้องถูกต้อง!**

ถ้า spacing ผิด → calibration ผิด → baseline ผิด → depth map ผิด!

---

## 📋 ขั้นตอนการพิมพ์

### **Option 1: พิมพ์จาก SVG โดยตรง (แนะนำ)**

#### A. ใช้ Inkscape (Free, Accurate)

1. **เปิดไฟล์ SVG:**
   ```bash
   # Download Inkscape: https://inkscape.org/
   # Open: calibration_pattern_18mm.svg
   ```

2. **ตั้งค่าก่อนพิมพ์:**
   - File → Document Properties
   - ตรวจสอบ: Page size = Custom
   - Units: mm
   - Width: 252mm, Height: 126mm (สำหรับ 18mm pattern)

3. **Export เป็น PNG:**
   - File → Export PNG Image
   - **DPI: 300** (IMPORTANT!)
   - Export As: `calibration_pattern_18mm.png`

4. **พิมพ์ PNG:**
   - เปิดไฟล์ PNG
   - Print settings:
     - Scale: **100% / Actual Size**
     - NO "Fit to Page"
     - NO "Scale to Fit"
   - Paper: A4

#### B. ใช้ Web Browser

1. เปิด SVG ใน Chrome/Firefox
2. Print (Ctrl+P):
   - Scale: **100%**
   - Margins: None
   - Background graphics: ON

---

### **Option 2: สร้าง PNG ด้วย ImageMagick**

```bash
# บน Linux/Mac/Jetson:
convert -density 300 calibration_pattern_18mm.svg calibration_pattern_18mm.png

# พิมพ์ไฟล์ PNG ที่ได้
```

---

### **Option 3: ใช้ Online SVG to PNG Converter**

1. ไปที่: https://cloudconvert.com/svg-to-png
2. Upload: `calibration_pattern_18mm.svg`
3. Options: **DPI = 300**
4. Convert & Download
5. พิมพ์ไฟล์ PNG ที่ได้ (100% scale)

---

### **Option 4: สร้าง PNG ด้วย Python (บน Jetson)**

```bash
# บน Jetson Nano (มี OpenCV + NumPy):
python3 generate_calibration_pattern.py

# จะได้ไฟล์:
# - calibration_pattern_5x6_18mm.png
# - calibration_pattern_5x6_16mm.png
```

---

## ✅ ขั้นตอนหลังพิมพ์ (CRITICAL!)

### **1. วัดระยะ Spacing (สำคัญที่สุด!)**

ใช้ **เวอร์เนียร์** (caliper) หรือ **ไม้บรรทัดแม่นยำ:**

```
วัดระยะทแยงมุม (diagonal) ระหว่างศูนย์กลางวงกลม 2 วง:

Row 0, Col 0  →  Row 0, Col 1
    •  -------→  •

ควรได้: 18.0 mm (± 0.5mm)
```

**ถ้าไม่ตรง:**
- เครื่องพิมพ์มี scaling ผิด
- วัดค่าจริงที่ได้ และใช้ค่านั้นใน `stereo_calibration.py`:
  ```python
  spacing_mm=17.5  # ใช้ค่าที่วัดได้จริง!
  ```

---

### **2. ตรวจสอบคุณภาพพิมพ์**

✅ **ต้องผ่านเงื่อนไขนี้:**
- วงกลมดำชัดเจน ไม่มัว
- ไม่มี jagged edges (ถ้าใช้ 300 DPI)
- พื้นหลังขาวสะอาด
- กระดาษไม่เปียก ไม่ยับ

❌ **ถ้าพบปัญหา:**
- เปลี่ยนกระดาษคุณภาพสูงกว่า
- เพิ่ม DPI เป็น 600
- ใช้ laser printer แทน inkjet

---

### **3. ติด Pattern บนแผ่นแข็ง**

**วัสดุแนะนำ:**
- ✅ Foam board (5mm หนา)
- ✅ Acrylic sheet (3mm หนา)
- ✅ MDF board (แผ่นไม้)
- ❌ กระดาษเปล่าๆ (จะโค้งงอ!)

**วิธีติด:**
1. ใช้ double-sided tape หรือ spray adhesive
2. ปาดให้เรียบ ไม่มีฟองอากาศ
3. **ตรวจสอบให้แน่ใจว่าแบน 100%** (ไม่มี warping)

---

## 📊 Pattern Specifications

### Asymmetric Circles Grid Layout:

```
Pattern จะมีลักษณะดังนี้ (5 rows × 6 cols):

Row 0: •   •   •   •   •   •     (offset 0mm)
Row 1:   •   •   •   •   •   •   (offset +9mm)
Row 2: •   •   •   •   •   •     (offset 0mm)
Row 3:   •   •   •   •   •   •   (offset +9mm)
Row 4: •   •   •   •   •   •     (offset 0mm)

← spacing = 18mm (diagonal) →
```

**Formula (same as stereo_calibration.py):**
```python
for i in range(rows):
    for j in range(cols):
        x = (2 * j + i % 2) * spacing_mm
        y = i * spacing_mm
```

---

## 🎯 การใช้งานกับ OpenCV

### Configuration:

```python
pattern_rows = 5
pattern_cols = 6
spacing_mm = 18.0  # หรือค่าที่วัดได้จริง!
pattern_type = cv2.CALIB_CB_ASYMMETRIC_GRID
```

### ทดสอบว่า Pattern ใช้งานได้:

```bash
# ถ่ายรูปทดสอบ 1 รูป
python3 capture_calibration.py

# ดูว่า detect pattern ได้หรือไม่
# ถ้าขึ้น "DETECTED" → Pattern OK!
```

---

## 🔧 Troubleshooting

### ปัญหา: "Pattern not detected"

**สาเหตุที่เป็นไปได้:**

1. **Spacing ผิด:**
   - วัดระยะจริงๆ ด้วย caliper
   - ใช้ค่าที่วัดได้ใน code

2. **วงกลมมัว:**
   - พิมพ์ DPI ต่ำเกิน
   - ใช้ 300 DPI ขึ้นไป

3. **Pattern โค้งงอ:**
   - ติดบนแผ่นแข็ง
   - ให้แบน 100%

4. **แสงไม่พอ:**
   - ถ่ายในที่แสงสว่าง
   - หลีกเลี่ยง shadow

5. **Focus ไม่ชัด:**
   - ตั้ง focus ให้ชัดก่อนถ่าย
   - ดู focus values บนหน้าจอ

---

### ปัญหา: "Baseline ผิดปกติมาก (436mm)"

**สาเหตุหลัก:**

1. ⚠️ **Spacing ไม่ตรง!** (ผิด 16mm vs 18mm)
   - ถ้า pattern จริง = 16mm แต่ใช้ 18mm ใน code:
   - → Baseline จะมากเกิน 12.5%!
   - **วิธีแก้:** วัด spacing จริง, ใช้ค่าถูกต้อง

2. Pattern print scaling ผิด
   - เครื่องพิมพ์ scale อัตโนมัติ
   - **วิธีแก้:** พิมพ์ใหม่ที่ 100% scale

3. Pattern mounted ไม่แบน
   - มี warping ทำให้ depth ผิด
   - **วิธีแก้:** ติดบนแผ่นแข็งแบน

---

## 📏 วิธีวัด Spacing ที่ถูกต้อง

### ใช้เวอร์เนียร์ (Caliper):

```
1. หาวงกลม 2 วงที่อยู่ในแนวทแยง:

   Row 0, Col 0    Row 0, Col 1
       •                •
       |             ╱
       |          ╱
       |       ╱
       |    ╱
       | ╱
       •
   Row 1, Col 0

2. วัดระยะจากศูนย์กลางวง 1 → ศูนย์กลางวง 2

3. ควรได้ ~ 18mm (หรือ 16mm ถ้าเป็น pattern เก่า)

4. วัดหลายๆ คู่ ดูค่าเฉลี่ย
```

---

## 📐 ขนาดกระดาษที่แนะนำ

### Pattern 18mm:
- Size: 252 × 126 mm
- จะพอดีกับ **กระดาษ A4** (210 × 297 mm)
- อาจต้องหมุนเป็น landscape

### Pattern 16mm:
- Size: 230 × 118 mm
- พอดี A4 landscape สบายๆ

---

## 🎓 ทำไม Asymmetric Circles ถึงดีกว่า Chessboard?

**ข้อดี:**

1. **ไม่มีปัญหา 180° ambiguity:**
   - Chessboard ดูเหมือนกันถ้าหมุน 180°
   - Asymmetric ไม่เกิดปัญหานี้

2. **Detection robust กว่า:**
   - Circles ตรวจจับง่ายกว่า corners
   - ทำงานได้ดีแม้มุมเอียง

3. **แนะนำโดย OpenCV:**
   - สำหรับ high-precision calibration

---

## ✅ Checklist ก่อนใช้ Pattern

- [ ] พิมพ์ที่ 300 DPI
- [ ] ตั้งค่า scale = 100% / Actual Size
- [ ] วัด spacing ด้วย caliper (ต้องได้ 18mm ± 0.5mm)
- [ ] วงกลมชัดเจน ไม่มัว
- [ ] ติดบนแผ่นแข็ง (foam board/acrylic)
- [ ] ตรวจสอบว่าแบน ไม่มี warping
- [ ] ทดสอบ detect ด้วย capture_calibration.py

---

## 📝 สรุป

**สิ่งสำคัญที่สุด:**

1. ✅ **พิมพ์ที่ 100% scale, 300 DPI**
2. ✅ **วัด spacing จริงๆ ด้วย caliper**
3. ✅ **ใช้ค่าที่วัดได้ใน stereo_calibration.py**
4. ✅ **ติดบนแผ่นแข็งให้แบน**

**ถ้าทำถูกทุกข้อ → Calibration จะแม่นยำ!**

---

**Generated:** 2025-10-26
**Files:**
- `calibration_pattern_18mm.svg`
- `calibration_pattern_16mm.svg`
