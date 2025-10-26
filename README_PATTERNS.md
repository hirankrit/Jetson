# 📐 Calibration Patterns - Quick Start

**Pattern Files พร้อมใช้งาน สำหรับ Stereo Camera Calibration**

---

## 🎯 Pattern Files ที่สร้างให้แล้ว

### ✅ **calibration_pattern_18mm.svg** (แนะนำ)
- Asymmetric Circles Grid: **5 rows × 6 columns**
- Spacing: **18mm** (diagonal between circle centers)
- Circle diameter: **14mm**
- Size: **252 × 126 mm** (พอดี A4 landscape)
- Total circles: **30**

### ✅ **calibration_pattern_16mm.svg** (สำรอง)
- เหมือนข้อบน แต่ spacing = **16mm**
- Size: **230 × 118 mm**
- ใช้ในกรณีที่ pattern เดิมเป็น 16mm

---

## 🖨️ วิธีพิมพ์ (ง่ายที่สุด)

### **Option 1: พิมพ์จาก Browser (แนะนำ!)**

1. คลิกขวา `calibration_pattern_18mm.svg` → Open with → Chrome/Firefox
2. กด Ctrl+P (Print)
3. ตั้งค่า:
   - Scale: **100%**
   - Margins: None
   - Background graphics: **ON**
4. Print

### **Option 2: Export PNG แล้วพิมพ์**

**ใช้ Inkscape (Free):**
```bash
# Download: https://inkscape.org/

1. เปิด calibration_pattern_18mm.svg ใน Inkscape
2. File → Export PNG Image
3. DPI: 300
4. Export
5. พิมพ์ไฟล์ PNG ที่ได้ (100% scale)
```

**ใช้ ImageMagick (Command line):**
```bash
convert -density 300 calibration_pattern_18mm.svg pattern.png
```

**ใช้ Python (บน Jetson):**
```bash
python3 generate_calibration_pattern.py
# จะได้ PNG files พร้อม info
```

---

## ✅ ขั้นตอนหลังพิมพ์ (สำคัญ!)

### **1. วัดระยะ Spacing (CRITICAL!)**

ใช้ **เวอร์เนียร์** หรือ **ไม้บรรทัด** วัด:

```
วัดระยะทแยงมุม (diagonal) ระหว่างศูนย์กลางวงกลม:

   •  ←─── 18mm ───→  •

   (Row 0, Col 0)   (Row 0, Col 1)
```

**ควรได้:** 18.0 mm (± 0.5mm)

**⚠️ ถ้าไม่ตรง:**
- บันทึกค่าที่วัดได้จริง (เช่น 17.5mm)
- ใช้ค่านั้นใน `stereo_calibration.py`:
  ```python
  spacing_mm=17.5  # ค่าที่วัดได้จริง!
  ```

---

### **2. ติดบนแผ่นแข็ง**

**วัสดุแนะนำ:**
- ✅ Foam board (5mm)
- ✅ Acrylic sheet (3mm)
- ✅ MDF board
- ❌ กระดาษเปล่า (จะโค้ง!)

**วิธีติด:**
1. ใช้ double-sided tape หรือ spray adhesive
2. ปาดให้เรียบ ไม่มีฟองอากาศ
3. ตรวจสอบให้แบน 100% (ไม่มี warping)

---

## 📊 Pattern Layout

```
Asymmetric Circles Grid (5×6):

Row 0:  •   •   •   •   •   •     (offset 0)
Row 1:    •   •   •   •   •   •   (offset +9mm)
Row 2:  •   •   •   •   •   •     (offset 0)
Row 3:    •   •   •   •   •   •   (offset +9mm)
Row 4:  •   •   •   •   •   •     (offset 0)

←─── spacing = 18mm (diagonal) ───→
```

**ทำไมถึงเป็น Asymmetric?**
- ไม่มีปัญหา 180° rotation ambiguity
- Detection robust กว่า chessboard
- แนะนำโดย OpenCV สำหรับ high-precision calibration

---

## 🎯 ใช้กับ OpenCV

### Configuration:

```python
# ใน capture_calibration.py และ stereo_calibration.py:

pattern_rows = 5
pattern_cols = 6
spacing_mm = 18.0  # หรือค่าที่วัดได้จริง!
pattern_type = cv2.CALIB_CB_ASYMMETRIC_GRID
```

### ทดสอบว่า Pattern ใช้งานได้:

```bash
python3 capture_calibration.py

# ดูหน้าจอว่าขึ้น "DETECTED" หรือไม่
# ถ้าขึ้น → Pattern พร้อมใช้งาน!
```

---

## 🔧 Troubleshooting

### ปัญหา: "Pattern not detected"

**แก้ไข:**
1. ✅ ตรวจสอบว่าพิมพ์ที่ 100% scale
2. ✅ วงกลมชัดเจน (DPI ≥ 300)
3. ✅ Pattern แบน (ติดบนแผ่นแข็ง)
4. ✅ แสงเพียงพอ ไม่มี shadow
5. ✅ Focus ชัด (ดู focus values)

---

### ปัญหา: "Baseline ผิดปกติ (436mm)"

**สาเหตุ:**
- ⚠️ **Spacing ไม่ตรง!** (16mm vs 18mm)
- ถ้า pattern จริง = 16mm แต่ใช้ 18mm:
  - → Baseline error = 12.5%

**แก้ไข:**
1. วัด spacing จริงๆ ด้วย caliper
2. ใช้ค่าที่วัดได้ใน code
3. หรือพิมพ์ pattern ใหม่ให้ถูกต้อง

---

## 📋 Checklist

- [ ] ดาวน์โหลด `calibration_pattern_18mm.svg`
- [ ] พิมพ์ที่ **100% scale, 300 DPI**
- [ ] วัด spacing ด้วย caliper (18mm ± 0.5mm)
- [ ] ติดบนแผ่นแข็ง (foam board)
- [ ] ตรวจสอบว่าแบน ไม่มี warping
- [ ] ทดสอบ detection ด้วย `capture_calibration.py`
- [ ] ใช้ค่า spacing ที่วัดได้จริงใน `stereo_calibration.py`

---

## 📚 เอกสารเพิ่มเติม

- **PATTERN_PRINTING_GUIDE.md** - คู่มือการพิมพ์แบบละเอียด
- **generate_calibration_pattern.py** - สร้าง PNG pattern
- **generate_pattern_simple.py** - สร้าง SVG pattern

---

## 🎓 ทำไม Spacing สำคัญมาก?

**Example:**

```
ถ้า Pattern จริง = 16mm แต่ใช้ 18mm ใน code:

Scale error = 18/16 = 1.125 (12.5%)

ผลกระทบ:
- Baseline จริง = 60mm
- Baseline calculated = 60 × 1.125 = 67.5mm ❌

ทุกๆ distance measurement จะผิด 12.5%!
```

**วิธีแก้:**
- วัด spacing จริง → ใช้ค่านั้น → calibration ถูกต้อง ✅

---

**ไฟล์พร้อมใช้:**
- ✅ `calibration_pattern_18mm.svg`
- ✅ `calibration_pattern_16mm.svg`

**พิมพ์ได้เลย!** 🎯
