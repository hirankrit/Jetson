## 📅 Dataset Collection Progress (Week 2) - Oct 29, 2025

### 🔬 Experiment: Grid Layout Auto Crop

**แนวคิด:** ประหยัดเวลาด้วยการวางพริกหลายเม็ดพร้อมกัน แล้วใช้ background subtraction crop อัตโนมัติ

**เป้าหมาย:**
- วางพริก 4-10 เม็ดพร้อมกัน (ห่างกัน 3-5cm)
- ถ่าย 4 มุม (หมุนพื้น 0°, 90°, 180°, 270°)
- Auto crop แต่ละตัว → ประหยัดเวลา ~10× (2 ชั่วโมง → 15 นาที)

---

### 🧪 ผลการทดลอง (3 รอบ)

#### **รอบที่ 1: พื้นขาว A4**
```
Setup:
- พื้น: กระดาษขาว A4
- พริก: 4 เม็ด (แดงใหญ่)
- Exposure: 30ms, Gain: 2
- Threshold: 180

ผลลัพธ์:
❌ Success Rate: 18.75% (3/16 peppers detected)

ปัญหา:
- Chromatic aberration สูงมาก (ขอบพริกเป็นสีน้ำเงิน-ม่วง)
- กล้อง stereo + พื้นขาวสว่าง → overexposure + color fringing
- Binary mask แยกไม่ได้ว่าส่วนไหนคือพริก
```

#### **รอบที่ 2: แผ่นดำ (Cutting Mat/สายพาน)**
```
Setup:
- พื้น: แผ่นสายพานดำ (มี texture)
- พริก: 4 เม็ด
- Exposure: 30ms → 24ms
- Threshold: 100-120

ผลลัพธ์:
❌ Success Rate: 6.25% (1/16 peppers detected)

ปัญหา:
- พื้นมี texture/ลวดลาย → threshold สับสน
- แสงไม่สม่ำเสมอ (กลางสว่าง, ขอบมืด) → gradient มาก
- Overexposure ทำให้พื้นดูเป็นสีเขียว (จริงๆ เป็นสีดำ)
```

#### **รอบที่ 3: ผ้าดำจริง + ลด Exposure**
```
Setup:
- พื้น: ผ้าดำ (มี texture ผ้า)
- พริก: 4 เม็ด
- Exposure: 24ms → 15ms (ลดแสง 50%)
- Gain: 2 → 1.5
- Threshold: 90-120

ผลลัพธ์:
❌ Success Rate: 0% (0/16 peppers detected)

ปัญหา:
- ผ้ามี texture มากกว่าแผ่นสายพาน
- Binary mask: พริกและพื้นเป็นสีเดียวกัน (แยกไม่ออก)
- แม้ลด exposure แล้ว พื้นยังดูเป็นสีเทา-ม่วง (ไม่ใช่ดำ)
```

---

### 📊 สรุปผลการทดลอง Grid Layout

| รอบ | พื้น | Exposure | Success Rate | ปัญหาหลัก |
|-----|------|----------|--------------|-----------|
| 1 | ขาว A4 | 30ms | **18.75%** | Chromatic aberration |
| 2 | แผ่นดำ texture | 24ms | **6.25%** | Texture + gradient |
| 3 | ผ้าดำ texture | 15ms | **0%** | Texture มาก + ไม่ uniform |

**สรุป:** Grid Layout Auto Crop **ไม่เหมาะกับ setup นี้**

---

### 🔍 Root Cause Analysis

**ปัญหา 3 ข้อหลัก:**

1. **❌ พื้นมี Texture**
   - ผ้า/สายพาน มีลวดลาย ไม่เรียบ uniform
   - Brightness threshold แยกไม่ได้ว่าส่วนไหนคือพื้น ส่วนไหนคือพริก
   - Binary mask ผิดพลาดอย่างรุนแรง

2. **❌ แสงไม่สม่ำเสมอ**
   - LED 3 ดวง (Top, Left, Right) → มี gradient (กลางสว่าง, ขอบมืด)
   - Background subtraction ต้องการแสง uniform 100%
   - ไม่มี lightbox หรือ LED panel diffuser

3. **❌ กล้อง Stereo มี Chromatic Aberration**
   - Lens distortion + wide-angle → สีผิดเพี้ยนที่ขอบภาพ
   - พื้นขาวสว่าง → เกิด color fringing (ขอบสีน้ำเงิน-ม่วง)
   - ลด exposure ช่วยได้บ้าง แต่ไม่หายหมด

---

### ✅ ข้อสรุปและการตัดสินใจ

**Background Subtraction Method ต้องการ:**
- ✅ พื้นเรียบ 100% uniform (กระดาษ, โฟมบอร์ด - **ไม่ใช่ผ้า**)
- ✅ แสงสม่ำเสมอ 100% (LED panel/lightbox professional)
- ✅ กล้อง RGB ธรรมดา (ไม่ใช่ stereo wide-angle)

**ทางเลือกที่มี:**
1. **Option A:** ซื้ออุปกรณ์ใหม่ (กระดาษดำ A3 + LED panel) - งบ 500-1,500 บาท
2. **Option B:** Manual Capture ⭐ **เลือกทางนี้**

---

### 🎯 แผนใหม่: Manual Capture Method

**เหตุผล:**
- ✅ ใช้ได้แน่นอน 100%
- ✅ ไม่ต้องซื้ออุปกรณ์เพิ่ม
- ✅ เริ่มได้ทันที
- ⏰ ใช้เวลา ~2-3 ชั่วโมง (ยอมรับได้)

**วิธีการ:**
```
1. วางพริก 1 ผล กลางพื้น (ใช้ผ้าดำที่มีอยู่)
2. ถ่ายภาพ 12 มุม (หมุนพริก 30° ทีละครั้ง)
   - 0°, 30°, 60°, 90°, 120°, 150°, 180°, 210°, 240°, 270°, 300°, 330°
3. เปลี่ยนผลใหม่ ทำซ้ำ
4. รวม: 31 ผล × 12 มุม = 372 ภาพ

Setup:
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset_manual
```

**Timeline:**
- 1 ผล ≈ 3-4 นาที
- 31 ผล ≈ 2 ชั่วโมง
- แบ่งเป็น 3 sessions (แดงใหญ่, แดงกลาง, เขียว)

**Status:** 
- 🟡 กำลังทดสอบ 1 ผล 12 มุม ก่อนเก็บเต็มรูปแบบ
- ⏳ รอผลทดสอบเพื่อปรับแต่ง setup (ถ้าจำเป็น)

---

### 📝 Lessons Learned

**สิ่งที่เรียนรู้:**
1. ✅ Grid Layout ดูดีบนกระดาษ แต่ต้องการ setup แบบ lab professional
2. ✅ กล้อง stereo wide-angle มี chromatic aberration มาก (พื้นขาว + overexposure)
3. ✅ Texture พื้นสำคัญมากสำหรับ background subtraction (ผ้า < กระดาษ)
4. ✅ แสง LED 3 ดวงไม่เพียงพอสำหรับ auto crop (ต้อง LED panel diffuser)
5. ✅ Manual capture ช้ากว่า แต่ **reliable 100%**

**แนะนำสำหรับอนาคต:**
- Grid Layout เหมาะกับ: Lab setup, budget >2,000฿, RGB camera
- Manual capture เหมาะกับ: Home/small lab, budget <500฿, reliable

---

### 🛠️ Tools Created

**Script: `auto_crop_grid.py`**
```bash
# Auto-crop peppers from grid layout images
python3 auto_crop_grid.py \
  --label pepper_red_fresh \
  --images set1_*.jpg \
  --threshold 180 \
  --min-area 1500 \
  --save-debug
```

**Features:**
- Background subtraction (Otsu threshold + manual)
- Contour detection with filtering (area, aspect ratio, solidity)
- Automatic crop with padding
- Debug visualization (mask + detection)
- Batch processing multiple images

**Status:** ⚠️ ใช้งานได้ แต่ต้องการ setup ที่เหมาะสม (พื้นเรียบ + แสง uniform)

---

**Updated:** Oct 29, 2025 09:10 AM
**Next Step:** ทดสอบ Manual capture 1 ผล 12 มุม → Review → เก็บ dataset เต็มรูปแบบ


---

## 🎉 Manual Capture Method - Success! (Oct 29, 2025 09:30 AM)

### ✅ ผลการทดสอบ

**Pilot Test: 1 ผล × 12 มุม**
```
Setup:
- พื้น: ผ้าดำ (มี texture - ไม่เป็นปัญหา)
- Exposure: 24ms (ลด 20% จากเดิม)
- Gain: 2
- แสง: LED 3 ดวง (Top, Left, Right)

ผลลัพธ์: ✅ 100% Success
- 12 ภาพครบ, ชัดเจนทุกมุม
- Contrast ดีเยี่ยม (พริกชัด vs พื้นดำ)
- แสงสม่ำเสมอ, ไม่มี glare
- พริกอยู่กลางกรอบทุกภาพ

เวลา: ~3 นาที/ผล
```

---

### 📊 Session 1: พริกแดงใหญ่ - เสร็จสมบูรณ์

**Dataset Collected:**
```
Session: session1_red_large
Peppers: 8 ผล
Images: 96 ภาพ (8 × 12 มุม)
Time: 9 นาที (09:36-09:45)
Average: 1.1 นาที/ผล (เร็วกว่าคาด 3×!)

Quality: ✅ Excellent
- Resolution: 1280×720 JPEG
- Lighting: Consistent
- Focus: Sharp
- Coverage: 360° per pepper
```

**Label Distribution (ต้องบันทึก):**
```
pepper_0000-0011: fresh/rotten (ผลที่ 1)
pepper_0012-0023: fresh/rotten (ผลที่ 2)
pepper_0024-0035: fresh/rotten (ผลที่ 3)
pepper_0036-0047: fresh/rotten (ผลที่ 4)
pepper_0048-0059: fresh/rotten (ผลที่ 5)
pepper_0060-0071: fresh/rotten (ผลที่ 6)
pepper_0072-0083: fresh/rotten (ผลที่ 7)
pepper_0084-0095: fresh/rotten (ผลที่ 8)
```

---

### 📋 Session 2: Defect Types - เสร็จสมบูรณ์ ✅

**Strategy: แยกโฟลเดอร์ตามประเภท**

เหตุผล:
- ✅ ชื่อโฟลเดอร์ = Label (ชัดเจน)
- ✅ เพิ่ม dataset ในอนาคตง่าย
- ✅ Train/Val split ง่าย (แบ่งตามโฟลเดอร์)

**Dataset Collected:**
```
Session 2.1: session2_red_rotten/    ✅ 12 ภาพ (พริกแดงเน่า)
Session 2.2: session2_red_insect/    ✅ 12 ภาพ (พริกแดงแมลงเจาะ)
Session 2.3: session2_red_deformed/  ✅ 12 ภาพ (พริกแดงงอ)
Session 2.4: session2_red_wrinkled/  ✅ 12 ภาพ (พริกแดงเหี่ยว)

Total: 48 ภาพ (4 ผล × 12 มุม)
Quality: ✅ Excellent
Settings: exposure=24ms, gain=2
```

**คำสั่งที่ใช้:**
```bash
# Session 2.1: พริกแดงเน่า
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_rotten

# Session 2.2: พริกแดงแมลงเจาะ
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_insect

# Session 2.3: พริกแดงงอ
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_deformed

# Session 2.4: พริกแดงเหี่ยว
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_wrinkled
```

---

### 🗂️ Dataset Structure (Current)

```
pepper_dataset/
├── session1_red_large/           ✅ 96 ภาพ (8 ผล × 12 มุม)
│   ├── raw/left/
│   │   └── pepper_0000-0095.jpg
│   └── metadata/
│       └── collection_log.yaml
│
├── session2_red_rotten/          ✅ 12 ภาพ (1 ผล × 12 มุม)
├── session2_red_insect/          ✅ 12 ภาพ (1 ผล × 12 มุม)
├── session2_red_deformed/        ✅ 12 ภาพ (1 ผล × 12 มุม)
├── session2_red_wrinkled/        ✅ 12 ภาพ (1 ผล × 12 มุม)
│
└── session3_green_*/             ⏳ ยังไม่ได้ทำ

Total: 144 images (12 peppers)
```

---

### 📈 Progress Summary

**Completed:**
- ✅ Grid Layout Experiments (3 rounds) - Failed but learned
- ✅ Manual Capture Setup - Success
- ✅ Pilot Test (1 pepper) - 100%
- ✅ Session 1: Red large peppers (8 ผล) - 96 images
- ✅ Session 2: Defect types (4 ผล) - 48 images
  - ✅ 2.1: Red rotten (เน่า)
  - ✅ 2.2: Red insect (แมลงเจาะ)
  - ✅ 2.3: Red deformed (งอ)
  - ✅ 2.4: Red wrinkled (เหี่ยว)

**Total Dataset: 144 images (12 peppers × 12 angles)**

**Pending:**
- ⏳ Session 3 (Green peppers - optional)
- ⏳ Label annotation (Roboflow/LabelImg)
- ⏳ Train/Val split
- ⏳ YOLO training (Week 3)

---

### ⏱️ Time Tracking

| Task | Planned | Actual | Note |
|------|---------|--------|------|
| Grid Layout Exp | 30 min | 2 hours | Failed - learned lessons |
| Manual Setup | 10 min | 10 min | ✅ |
| Pilot Test | 5 min | 3 min | ✅ Faster! |
| Session 1 | 30 min | 9 min | 🚀 3× faster! |
| Session 2 (4 sessions) | 16 min | ~15 min | ✅ On target! |

**Total Dataset Collection Time:** ~24 minutes (144 images)
**Average:** 2 minutes per pepper, 10 seconds per image
**Efficiency:** 🚀 Exceeded expectations!

---

### 🎯 Key Learnings

**What Worked:**
1. ✅ **Manual capture** reliable 100%
2. ✅ **ผ้าดำ** sufficient (texture OK for manual)
3. ✅ **Exposure 24ms** perfect balance
4. ✅ **12 angles** comprehensive coverage
5. ✅ **Single session workflow** very fast (1 min/pepper)

**What Didn't Work:**
1. ❌ Grid layout auto crop (texture + lighting issues)
2. ❌ White background (chromatic aberration)
3. ❌ Overexposure (30ms too bright)

**For Future:**
- Grid layout requires: smooth background + LED panel/lightbox
- Manual capture: faster than expected, scales well
- Defect separation by folder: good for organization

---

**Updated:** Oct 29, 2025 02:45 PM
**Status:** Session 1-2 Complete! ✅ (144 images)
**Next:** Annotation → YOLO Training (Week 3)

---

## 🔍 Critical Discovery: Auto-Focus Problem (Oct 29, 2025 Evening)

### ⚠️ Problem Discovered

ระหว่างเก็บ Session 3.1 (Green peppers) สังเกตปัญหา:

**พฤติกรรมที่พบ:**
```
1. วางพริก → มือยังอยู่ในเฟรม
   → กล้อง focus ที่มือ (sharp)

2. เอามือออก → วินาทีแรก
   → ภาพพริกชัดมาก! texture เห็นชัดเจน ✅

3. หลังจาก 1-2 วินาทีถัดมา
   → ภาพพริกเบลอ ❌
   → texture ไม่ชัด, surface details หาย
```

**ผลกระทบ:**
- ✅ Dataset Session 1 (96 images): **เบลอ, texture ไม่ชัด**
- ✅ Dataset Session 2 (48 images): **เบลอ, texture ไม่ชัด**
- ❌ ไม่เหมาะสำหรับ YOLO training (ต้องการ sharp images)

---

### 🔬 Investigation

**Root Cause:** GStreamer pipeline ใช้ auto-focus เริ่มต้น

กล้อง IMX219 มีพฤติกรรม:
1. มือเข้าเฟรม → focus ที่มือ (วัตถุใกล้)
2. มือออก → focus ยังอยู่ที่ระยะเดิม (1 วินาที - sharp!)
3. Auto-focus ปรับใหม่ → focus ที่พื้นหลัง (พริกเบลอ)

**First Fix Attempt: ❌ Countdown**
- เพิ่ม countdown 3 วินาที
- คิดว่าต้องการเวลาให้กล้อง stabilize
- ผลลัพธ์: **ยังเบลออยู่**

**Root Cause: Missing aelock parameter**

---

### ✅ Solution: Auto-Exposure Lock (aelock=true)

**Modified GStreamer Pipeline:**

```python
# collect_dataset.py (Line 41)
return (
    f"nvarguscamerasrc sensor-id={sensor_id} "
    f"wbmode=0 "
    f"aelock=true "  # ← NEW: Locks auto-exposure & auto-focus
    f'exposuretimerange="{exposure_ns} {exposure_ns}" '
    f'gainrange="{gain} {gain}" '
    f"! "
    # ... rest of pipeline
)
```

**What aelock does:**
- ล็อค auto-exposure (ค่าแสง)
- ล็อค auto-focus (จุดโฟกัส)
- ป้องกันไม่ให้กล้องปรับเองขณะถ่าย

---

### 🧪 Testing with test_aelock.py

**Created Tool:** `test_aelock.py`
- วัดค่า sharpness ด้วย Laplacian variance
- Real-time monitoring ขณะเอามือเข้า-ออกเฟรม
- คำนวณ variation % (ยิ่งต่ำ = ยิ่ง stable)

**Test Results:**

**Test 1: มือเข้า-ออกเฟรมหลายครั้ง**
```
Frames: 240
Avg Sharpness: 168.8
Variation: 11.5%
Result: ⚠️ MODERATE (but expected - hand causes changes)
```

**Test 2: ไม่ขยับมือ (วางพริกนิ่ง)**
```
Frames: 50
Avg Sharpness: 176.9
Variation: 0.7%
Result: ✅ EXCELLENT! Focus very stable
```

**Test 3: ไม่ขยับมือ (ทดสอบซ้ำ)**
```
Frames: 189
Avg Sharpness: 172.7
Variation: 1.7%
Result: ✅ EXCELLENT!
```

**Conclusion:**
- ✅ `aelock=true` ทำงาน!
- ✅ เมื่อวางพริกแล้ว focus คงที่ (< 2% variation)
- ✅ มือเข้า-ออก variation สูงขึ้น (11-15%) แต่ไม่เป็นปัญหา (เพราะเราถ่ายหลังมือออกแล้ว)

---

### 🎯 Session 3.1 Test: Green Medium

**Test Dataset:**
```
Session: session3_green_medium
Peppers: 1 ผล
Images: 12 ภาพ (12 มุม)

Workflow:
1. วางพริก → เอามือออกทันที
2. รอ 3-5 วินาที (ให้กล้อง stabilize)
3. กด SPACE → countdown → 📸
4. หมุนพริก 30° → repeat

ผลลัพธ์: ✅ ชัดมาก! texture เห็นชัดเจนทุกภาพ!
```

**User Confirmation:**
> "แต่ภาพของ code นี้ชัดครับ เป็น texture ของผิวชัดมาก"
> "ผมบันทึก session3_green_medium 12 ภาพ ชัดมากไม่เบลอเลย"

---

### 🔄 Decision: Re-collect Dataset V2

**Comparison:**

| Version | Sessions | Images | Focus | Texture Visible | YOLO Ready |
|---------|----------|--------|-------|-----------------|------------|
| **V1 (OLD)** | 1-2 | 144 | ❌ เบลอ | ❌ ไม่ชัด | ❌ No |
| **V2 (NEW)** | 1-2-3 | 192 | ✅ sharp | ✅ ชัดมาก | ✅ Yes |

**Why Re-collect?**
1. **Consistency:** ภาพเก่า (เบลอ) + ภาพใหม่ (ชัด) = dataset ไม่สม่ำเสมอ
2. **YOLO Confusion:** model จะเรียนรู้ blurry features (ผิด)
3. **Accuracy Impact:** ความแม่นยำอาจต่ำลง 5-15%
4. **Time Cost:** Re-collect 60 นาที vs Train/Test/Debug (hours)

**Decision:** ✅ **Re-collect ทั้งหมด (Dataset V2)**

---

### 📋 Re-collection Plan (Dataset V2)

**Target: 192 images (16 peppers × 12 angles)**

**Session 1: Red Large Peppers**
- 8 peppers × 12 angles = 96 images
- Time: ~32 minutes

**Session 2: Red Defect Types**
- 4 peppers × 12 angles = 48 images
  - 2.1: Red rotten (เน่า)
  - 2.2: Red insect (แมลงเจาะ)
  - 2.3: Red deformed (งอ)
  - 2.4: Red wrinkled (เหี่ยว)
- Time: ~16 minutes

**Session 3: Green Varieties**
- 4 peppers × 12 angles = 48 images
  - 3.1: Green medium (ขนาดกลาง)
  - 3.2: Green small (ขนาดเล็ก)
  - 3.3: Green tiny (ขนาดเล็กมาก)
  - 3.4: Green rotten (เน่า)
- Time: ~16 minutes

**Total Time: ~64 minutes (1 hour)**

---

### 🛠️ New Tools Created

**1. `test_aelock.py`**
- Test focus stability with/without aelock
- Real-time sharpness monitoring
- Variation analysis

**2. `setup_new_dataset.sh`**
- Backup old dataset → `pepper_dataset_OLD_YYYYMMDD_HHMMSS/`
- Create new empty folder → `pepper_dataset/`

**3. `DATASET_RECOLLECTION_GUIDE.md`**
- Complete step-by-step guide
- All commands for 192 images
- Verification checklist

**4. `collect_all_commands.sh`**
- Interactive script
- All collection commands in one place
- Session-by-session display

---

### 📊 Technical Improvements

**Before (V1):**
```python
# No aelock parameter
pipeline = (
    f"nvarguscamerasrc sensor-id={sensor_id} "
    f"wbmode=0 "
    f'exposuretimerange="{exposure_ns} {exposure_ns}" '
    # ...
)

Result: Auto-focus changes → blurry images
```

**After (V2):**
```python
# With aelock parameter
pipeline = (
    f"nvarguscamerasrc sensor-id={sensor_id} "
    f"wbmode=0 "
    f"aelock=true "  # ← Locks focus!
    f'exposuretimerange="{exposure_ns} {exposure_ns}" '
    # ...
)

Result: Fixed focus → sharp images with texture!
```

---

### ✅ Key Learnings

**Camera Behavior:**
1. IMX219 มี auto-focus และ auto-exposure เริ่มต้น
2. Auto-focus ปรับตามวัตถุที่เข้าเฟรม (มือ → พริก → พื้นหลัง)
3. ต้องใช้ `aelock=true` เพื่อล็อค focus

**Workflow Best Practices:**
1. วางพริก → เอามือออกทันที
2. รอ 3-5 วินาที (ให้ stabilize)
3. กด SPACE → countdown → capture
4. หมุนพริกเร็วๆ (2-3 วินาที)
5. Repeat 12 มุม

**Quality Criteria:**
- ✅ Texture visible (surface details clear)
- ✅ Sharp focus (no blur)
- ✅ Consistent lighting (no shadows)
- ✅ Centered object (middle of frame)

---

### 🎯 Current Status

**Dataset V1 (Backed up):**
- ❌ 144 images with focus issues
- ❌ Not suitable for YOLO training
- ✅ Backup location: `pepper_dataset_OLD_YYYYMMDD_HHMMSS/`

**Dataset V2 (In Progress):**
- ⏳ Ready to collect: 192 images
- ✅ `aelock=true` enabled
- ✅ Test successful (Session 3.1: 12 sharp images)
- ✅ All tools and guides prepared

---

**Updated:** Oct 29, 2025 Evening
**Status:** 🔄 Ready for Dataset V2 Re-collection
**Next:** Execute re-collection plan (60 minutes) → Annotation → YOLO Training (Week 3)

