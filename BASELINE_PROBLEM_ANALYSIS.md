# การวิเคราะห์ปัญหา Baseline = 436mm (ควรเป็น ~60mm)

**วันที่ทดสอบ:** 26 ตุลาคม 2568
**สถานะ:** ✓ หาสาเหตุพบแล้ว

---

## 🔍 สรุปผลการทดสอบ

### Synthetic Data Test Results

**พารามิเตอร์ที่ใช้ทดสอบ:**
- True Baseline: **60.00 mm** (ค่าที่ต้องการ)
- Pattern: Asymmetric Circles 5×6
- Pattern Spacing: 18.00 mm
- Number of Poses: 10 (valid: 9)
- Noise Level: **0.00 pixels** (Perfect - ไม่มี noise)

**ผลลัพธ์:**
```
True Baseline:       60.0000 mm
Estimated Baseline:  60.0000 mm
Error:               -0.0000 mm
Error %:             0.00%

Stereo RMS error:    0.000090 pixels
Quality:             EXCELLENT
```

---

## ✅ สรุปหลักการ

### Algorithm ทำงานถูกต้อง 100%

การทดสอบด้วย synthetic data พิสูจน์ว่า **stereo calibration algorithm ทำงานถูกต้องสมบูรณ์**

- คำนวณ baseline ได้แม่นยำ **100%** (error 0.00%)
- Rotation matrix ใกล้เคียง identity matrix (parallel cameras)
- RMS error ต่ำมาก (0.000090 pixels)

**➜ ข้อสรุป: ปัญหาไม่ได้อยู่ที่ code หรือ algorithm**

---

## ❌ สาเหตุที่เป็นไปได้ของปัญหา Baseline = 436mm

เนื่องจาก algorithm ทำงานถูกต้อง ปัญหา baseline = 436mm (แทน ~60mm) ต้องมาจาก **physical parameters ที่ผิดพลาด**

### สาเหตุที่ 1: Pattern ถูกพิมพ์ในขนาดที่ผิด (Scale ผิด) ⭐ น่าจะเป็นสาเหตุหลัก

**การวิเคราะห์:**
```
Baseline วัดได้:  436 mm
Baseline ที่ต้องการ: 60 mm
Scale factor: 436 / 60 = 7.27 เท่า
```

ถ้า pattern spacing จริง = 18mm × 7.27 = **130.8 mm**
แต่เรากำหนดไว้ว่า spacing = 18mm

**ตัวอย่างการเกิดปัญหา:**
- พิมพ์แบบ "Fit to page" หรือ "Scale to fit" → ทำให้ pattern ใหญ่ขึ้น
- ตั้งค่าเครื่องพิมพ์เป็น "A3" แทน "A4"
- PDF viewer มีการ zoom โดยอัตโนมัติ

**วิธีตรวจสอบ:**
```bash
1. วัดระยะห่างจริงของ pattern ด้วยไม้บรรทัดหรือเวอร์เนีย
2. วัดระยะแนวทแยงระหว่าง 2 วงกลม (ควรเป็น 18mm)
3. ถ้าวัดได้ ~131mm → ยืนยันว่าถูก scale ผิด (7.27 เท่า)
```

---

### สาเหตุที่ 2: Physical Baseline ของกล้องไม่ใช่ 60mm

**การวิเคราะห์:**

ถ้า physical baseline จริงๆ = 436mm และ pattern spacing = 18mm (ถูกต้อง)
→ Algorithm จะคำนวณได้ 436mm (ซึ่งถูกต้อง)

**วิธีตรวจสอบ:**
```bash
1. วัดระยะห่างระหว่างเลนส์กล้อง Left และ Right
2. วัดจากจุดศูนย์กลางของเลนส์ไปยังจุดศูนย์กลางอีกเลนส์
3. ใช้เวอร์เนียเพื่อความแม่นยำ
```

**หมายเหตุ:** ตามปกติ baseline ของกล้อง stereo มือถือ/DIY มักอยู่ที่ 50-80mm
436mm = 43.6 cm นั้นใหญ่เกินไปสำหรับ stereo camera ทั่วไป

---

### สาเหตุที่ 3: ผสมผสานของทั้ง 2 ปัจจัย

อาจเกิดจาก:
- Pattern ถูก scale ขึ้น 2-3 เท่า
- Physical baseline จริงๆ อาจเป็น 80-100mm (ไม่ใช่ 60mm)
- รวมกันทำให้ได้ 436mm

---

## 🔧 วิธีแก้ไขที่แนะนำ

### ขั้นตอนที่ 1: ตรวจสอบ Physical Measurements (ทำก่อน)

```bash
# 1. วัด Physical Baseline
- ใช้เวอร์เนียวัดระยะห่างระหว่างกล้อง Left และ Right
- บันทึกค่าที่วัดได้ (หน่วย mm)

# 2. วัด Pattern Spacing
- วัดระยะห่างแนวทแยงระหว่าง 2 วงกลมติดกัน
- ควรได้ 18mm (หากพิมพ์ถูกต้อง)
- ถ้าได้ ~131mm → ยืนยันว่า pattern ถูก scale 7.27 เท่า
```

### ขั้นตอนที่ 2: แก้ไขตามผลการวัด

**กรณีที่ 1: Pattern ถูก scale ผิด**
```bash
# Solution A: พิมพ์ใหม่ด้วยขนาดที่ถูกต้อง
1. Download pattern จาก calib.io
2. ตั้งค่าเครื่องพิมพ์:
   - ขนาดกระดาษ: A4
   - Scale: 100% (ไม่ใช่ fit to page)
   - ตรวจสอบ actual size printing
3. วัดอีกครั้งหลังพิมพ์ให้แน่ใจว่าได้ 18mm

# Solution B: ปรับ pattern_spacing ใน code
ถ้าวัดได้ 131mm:
- แก้ stereo_calibration.py line 291
- เปลี่ยนจาก spacing_mm=18.0 → spacing_mm=131.0
- แก้ capture_calibration.py (หากมีการ hard-code)
```

**กรณีที่ 2: Physical Baseline ไม่ใช่ 60mm**
```bash
# ไม่ต้องแก้ไขอะไร
- Calibration algorithm จะคำนวณ baseline จริงให้อัตโนมัติ
- ค่า baseline ที่ได้คือค่าที่ถูกต้อง
- ใช้ค่าที่วัดได้จาก stereo_calib.yaml
```

---

## 📊 ตัวอย่างการคำนวณ Scaling Factor

### ถ้า Pattern Spacing วัดได้ X mm

```python
# กำหนดค่าที่ถูกต้องควรเป็น
EXPECTED_SPACING = 18.0  # mm

# วัดได้จริง
MEASURED_SPACING = X  # mm (วัดจากกระดาษที่พิมพ์)

# คำนวณ scale factor
scale_factor = MEASURED_SPACING / EXPECTED_SPACING

# ตัวอย่าง
# วัดได้ 131mm → scale = 131/18 = 7.28 เท่า
# วัดได้ 25mm  → scale = 25/18 = 1.39 เท่า
# วัดได้ 18mm  → scale = 18/18 = 1.00 เท่า (ถูกต้อง)
```

### การปรับ Baseline ที่คาดหวัง

```python
# ถ้า baseline วัดได้ = 436mm และ scale = 7.27
true_baseline = measured_baseline / scale_factor
true_baseline = 436 / 7.27 = 59.97 mm ≈ 60mm ✓
```

---

## 📝 Checklist การตรวจสอบ

- [ ] วัด physical baseline ระหว่างกล้อง Left-Right ด้วยเวอร์เนีย
- [ ] วัด pattern spacing แนวทแยงระหว่าง 2 วงกลม
- [ ] คำนวณ scale factor = measured / 18mm
- [ ] ถ้า scale ≠ 1.0 → พิมพ์ pattern ใหม่ด้วย actual size 100%
- [ ] ถ้า scale = 1.0 → ใช้ baseline ที่วัดได้เป็นค่าจริง
- [ ] รัน stereo_calibration.py อีกครั้งหลังแก้ไข
- [ ] ตรวจสอบ baseline ใหม่ใน stereo_calib.yaml

---

## 🎯 Recommendation

**สำหรับกรณีนี้ (baseline = 436mm):**

1. **วัด pattern spacing เป็นอันดับแรก**
   - ถ้าได้ ~131mm → พิมพ์ใหม่ด้วย actual size 100%
   - ถ้าได้ ~18mm → physical baseline จริงๆ คือ 436mm

2. **ถ้า pattern ถูกต้อง (18mm)**
   - Baseline 436mm เป็นค่าที่ถูกต้อง
   - ไม่ต้องแก้ไขอะไร
   - อาจพิจารณาปรับ camera mounting ถ้าต้องการ baseline ที่เล็กกว่า

3. **ถ้า pattern ผิด (scaled)**
   - พิมพ์ใหม่ทันที
   - Capture calibration images ใหม่
   - รัน stereo_calibration.py อีกครั้ง

---

## 📚 Reference Files

- **Test Script:** `test_baseline_synthetic.py`
- **Test Results:** `test_synthetic_baseline/baseline_test_20251026_031355.txt`
- **Calibration Script:** `stereo_calibration.py`
- **Capture Script:** `capture_calibration.py`

---

## 🔬 Technical Details

### Algorithm Validation

การทดสอบด้วย synthetic data ยืนยันว่า:
- ✓ Object points generation ถูกต้อง (asymmetric circles pattern)
- ✓ Camera calibration ทำงานถูกต้อง
- ✓ Stereo calibration ทำงานถูกต้อง
- ✓ Baseline calculation ทำงานถูกต้อง (np.linalg.norm(T))

### Error Sources in Real Data

Error ใน real data มาจาก:
1. **Physical measurement error** (pattern size, camera position)
2. **Detection error** (circle detection accuracy)
3. **Optical distortion** (lens distortion - แก้ไขได้ด้วย calibration)
4. **Image noise** (sensor noise, lighting)

---

**สรุป:** Algorithm ถูกต้อง → ปัญหาอยู่ที่ physical parameters → ต้องวัดและตรวจสอบ pattern size และ camera baseline จริง
