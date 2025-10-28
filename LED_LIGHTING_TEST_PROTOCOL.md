# 💡 LED Lighting Test Protocol

**วัตถุประสงค์**: ทดสอบผลของแสง LED ด้านบนต่อ depth coverage และความแม่นยำ

**วันที่เริ่มทดสอบ**: 2025-10-28

---

## 🎯 วัตถุประสงค์การทดสอบ

### สมมติฐาน (Hypothesis):
แสง LED ด้านบน → เพิ่ม texture visibility ที่ center → เพิ่ม coverage

### เป้าหมาย (Goals):
1. ✅ เปรียบเทียบ **Coverage** (Before vs After)
   - Overall coverage
   - Center region coverage (สำคัญมาก!)
   - Left/Right half coverage

2. ✅ เปรียบเทียบ **Depth Accuracy**
   - 10th percentile depth (recommended method)
   - Repeatability (std dev)

3. ✅ วิเคราะห์ **Trade-offs**
   - Over-exposure (ถ้าแสงมากเกินไป)
   - Specular reflection (แสงสะท้อนจากพริกมัน)

---

## 🔧 Setup

### Hardware:
- **Camera**: IMX219 Stereo (baseline 60.57mm)
- **Distance**: 32cm (working distance)
- **Object**: พริกแดง 1 ผล (consistent test object)

### Software:
- **Tool**: `test_pepper_foreground.py`
- **Resolution**: 1280x720 (matching calibration)
- **Method**: Percentile-based (10th percentile)

### Lighting Configurations:
1. **BEFORE**: แสงเดิม (ด้านข้างเท่านั้น)
   - ตำแหน่ง: ซ้ายหน้า, ทะแยง 10cm
   - Type: LED (existing setup)

2. **AFTER**: แสงด้านบน + ด้านข้าง
   - ตำแหน่ง: จากด้านบน (diffused)
   - Type: LED
   - Goal: เพิ่ม texture visibility ที่ center

---

## 📊 Test Procedure

### Phase 1: BEFORE LED Installation (Baseline)

**Test #1: Pattern Board @ 32cm** (Sanity check)
```bash
python3 test_pepper_foreground.py
# กด SPACE → capture
# บันทึก:
#   - Coverage (overall, left, right)
#   - 10th percentile depth
#   - Save image: 's'
```

**Expected Results (from previous tests):**
- Coverage: 80-90%
- 10th percentile: ~320mm (31.9-32.1 cm)
- Status: ✅ Calibration confirmed

---

**Test #2: Pepper @ 32cm** (Actual test)
```bash
# วางพริก 1 ผลที่ 32cm (วัดด้วยไม้บรรทัด)
python3 test_pepper_foreground.py
# กด SPACE → capture 5 ครั้ง (เพื่อดู repeatability)
# บันทึกทุกครั้ง: 's'

# บันทึกข้อมูล:
Capture #1:
  - Coverage (overall): __%
  - Coverage (left half): __%
  - Coverage (right half): __%
  - 10th percentile: __mm (__cm)
  - Median: __mm
  - Difference (Median - 10%ile): __mm
  - Std Dev: __mm

Capture #2-5: (repeat)
```

**After Test #2:**
```bash
# คำนวณค่าเฉลี่ย (N=5):
Average Coverage (overall): __% ± __% (std)
Average 10th percentile: __mm ± __mm (std)
```

---

### Phase 2: AFTER LED Installation

**Test #3: Pattern Board @ 32cm** (Confirm no regression)
```bash
# เหมือน Test #1
# ตรวจสอบว่า calibration ยังใช้ได้
# ระวัง: Over-exposure (ถ้าแสงมากเกินไป)
```

**Test #4: Pepper @ 32cm** (Main experiment)
```bash
# เหมือน Test #2
# วางพริกผลเดิม (หรือผลใหม่ที่คล้ายกัน)
# Capture 5 ครั้ง

# บันทึกข้อมูล: (same format as Test #2)
```

---

### Phase 3: Comparison Analysis

**Compare Results:**
```
Metric                  | BEFORE LED | AFTER LED | Improvement
------------------------|------------|-----------|-------------
Coverage (overall)      | __%        | __%       | +__%
Coverage (left half)    | __%        | __%       | +__%
Coverage (right half)   | __%        | __%       | +__%
10th percentile (avg)   | __mm       | __mm      | ±__mm
Repeatability (std)     | __mm       | __mm      | Better/Worse
```

**Key Questions:**
1. ✅ Coverage เพิ่มขึ้นหรือไม่? เท่าไหร่?
2. ✅ Coverage ที่ center region ดีขึ้นไหม?
3. ✅ Depth accuracy ยังคงแม่นยำไหม?
4. ⚠️ มี over-exposure หรือ specular reflection ไหม?

---

## 🌶️ Optional: Multi-Object Test

**Test #5: Multiple Peppers** (ถ้ามีเวลา)
```bash
# วางพริก 3 ผล ในระยะต่างกัน:
#   - Pepper A: 28cm
#   - Pepper B: 32cm
#   - Pepper C: 36cm

# Run test_pepper_foreground.py
# บันทึก:
#   - Overall coverage
#   - แต่ละผลมี coverage เท่าไหร่ (visual estimate)
```

---

## 📝 Expected Outcomes

### ✅ Success Criteria:
1. **Coverage improvement ≥ 10%** (overall)
2. **Center coverage ≥ 40%** (estimated from visual)
3. **No accuracy degradation** (10%ile ± 2mm from BEFORE)
4. **No over-exposure** (< 5% over-exposed pixels)

### ⚠️ Possible Issues:
1. **Over-exposure**: แสงมากเกินไป → ลด intensity
2. **Specular reflection**: พริกมันสะท้อนแสง → ใช้ diffuser
3. **Shadow artifacts**: แสงไม่สม่ำเสมอ → ปรับตำแหน่ง LED

### 🔧 Mitigation:
- ถ้า over-exposure → ลด LED intensity หรือเพิ่มระยะห่าง
- ถ้า specular → ใช้ diffuser (กระดาษไขหรือผ้าขาว)
- ถ้า shadow → ใช้ multiple LED sources (หลายตำแหน่ง)

---

## 📁 File Naming Convention

**BEFORE LED:**
```
foreground_[timestamp]_left.jpg    # Left camera image
foreground_[timestamp]_depth.jpg   # Depth map visualization
```

**AFTER LED:**
```
led_test_[timestamp]_left.jpg      # Left camera image
led_test_[timestamp]_depth.jpg     # Depth map visualization
```

**Rename after capture:**
```bash
# BEFORE
mv foreground_1234567890_left.jpg before_led_pepper_01_left.jpg
mv foreground_1234567890_depth.jpg before_led_pepper_01_depth.jpg

# AFTER
mv foreground_1234567890_left.jpg after_led_pepper_01_left.jpg
mv foreground_1234567890_depth.jpg after_led_pepper_01_depth.jpg
```

---

## 📊 Results Template

**Test Date**: 2025-10-28

### BEFORE LED Installation

**Pattern Board @ 32cm:**
- Coverage: __%
- 10th percentile: __mm
- Status: ✅ / ❌

**Pepper @ 32cm (N=5 captures):**

| Capture | Coverage | Left | Right | 10%ile | Median | Diff | Std |
|---------|----------|------|-------|--------|--------|------|-----|
| #1      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #2      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #3      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #4      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #5      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| **Avg** | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| **Std** | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|

---

### AFTER LED Installation

**LED Setup:**
- Position: __
- Type: __
- Intensity: __
- Diffuser: Yes / No

**Pattern Board @ 32cm:**
- Coverage: __%
- 10th percentile: __mm
- Status: ✅ / ❌
- Notes: (over-exposure? artifacts?)

**Pepper @ 32cm (N=5 captures):**

| Capture | Coverage | Left | Right | 10%ile | Median | Diff | Std |
|---------|----------|------|-------|--------|--------|------|-----|
| #1      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #2      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #3      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #4      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| #5      | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| **Avg** | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|
| **Std** | __%      | __%  | __%   | __mm   | __mm   | __mm | __mm|

---

### Comparison

| Metric                | BEFORE | AFTER | Δ (Improvement) | %Change |
|-----------------------|--------|-------|-----------------|---------|
| Coverage (overall)    | __%    | __%   | +__%            | +__%    |
| Coverage (left half)  | __%    | __%   | +__%            | +__%    |
| Coverage (right half) | __%    | __%   | +__%            | +__%    |
| 10%ile depth          | __mm   | __mm  | ±__mm           | ±__%    |
| Repeatability (std)   | __mm   | __mm  | ±__mm           | Better/Worse |

**Conclusion:**
- ✅ / ❌ Coverage improved significantly (≥10%)
- ✅ / ❌ Depth accuracy maintained (±2mm)
- ✅ / ❌ No over-exposure or artifacts
- 💡 Recommendation: Keep LED / Adjust LED / Remove LED

**Notes:**
(อธิบายสิ่งที่สังเกตเห็น, ปัญหา, ข้อเสนอแนะ)

---

## 🚀 Next Steps After Testing

**If Successful (coverage +10%, accuracy maintained):**
1. ✅ Keep LED setup
2. 📸 Start Week 2: Dataset collection
3. 📝 Update CAMERA_SETUP_GUIDE.md with LED setup

**If Marginal (coverage +5-10%, some issues):**
1. 🔧 Optimize LED position/intensity
2. 🔄 Re-test with adjustments
3. 📊 Compare results again

**If Unsuccessful (no improvement or artifacts):**
1. 🔍 Analyze why (over-exposure? reflection?)
2. 🔄 Try alternative approaches:
   - Diffused lighting
   - Polarized filters
   - Multiple LED angles
3. 💭 Consider: Current setup might be good enough (40-70% already decent!)

---

**Good luck with testing! 🌶️💡**
