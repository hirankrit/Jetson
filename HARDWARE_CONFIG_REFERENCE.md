# 📸 Hardware Configuration Reference

## Overview

ไฟล์ `hardware_config.yaml` บันทึก **Camera Parameters ทั้ง 7 หมวด** และข้อมูล hardware setup ครบถ้วน เพื่อให้สามารถ **reproduce** การทดลองได้แม่นยำ

ไฟล์นี้จะถูกสร้างอัตโนมัติทุกครั้งที่รัน `collect_dataset.py` และบันทึกใน:
```
pepper_dataset/<session_name>/metadata/hardware_config.yaml
```

---

## 📋 7 Categories of Camera Parameters

### 1. Exposure & Light Sensitivity
พารามิเตอร์ที่ควบคุมปริมาณแสงที่เข้ากล้อง

```yaml
exposure_and_light:
  exposure_time_ms: 30              # เวลาเปิดชัตเตอร์ (milliseconds)
  exposure_time_ns: 30000000        # เวลาเปิดชัตเตอร์ (nanoseconds)
  gain_iso: 2                       # ความไวแสง (ISO equivalent)
  auto_exposure: false              # ปิด auto-exposure
  ae_lock: true                     # ล็อค auto-exposure (ป้องกันกระพริบ)
  notes: "MANUAL mode with fixed exposure/gain to prevent flickering"
```

**Why it matters:**
- Fixed exposure = ความสว่างคงที่ทุกภาพ
- Prevents flickering = ภาพไม่กระพริบ
- Reproducibility = ถ่ายภาพซ้ำได้เหมือนเดิม

---

### 2. White Balance & Color
พารามิเตอร์ที่ควบคุมสมดุลสีและโทนสี

```yaml
white_balance_and_color:
  white_balance_mode: "manual"      # โหมด white balance (manual/auto)
  awb_lock: true                    # ล็อค auto white balance
  color_temperature_k: null         # อุณหภูมิสี (Kelvin) - ไม่ได้ตั้ง
  saturation_percent: null          # ความอิ่มตัวสี - ใช้ default
  hue_shift_deg: 0                  # เลื่อนโทนสี (degrees) - ไม่เลื่อน
  notes: "Manual white balance to ensure color consistency"
```

**Why it matters:**
- Manual WB = สีคงที่ทุกภาพ (สำคัญสำหรับ classification!)
- Color consistency = YOLO เรียนรู้สีได้แม่นยำ

---

### 3. Image Enhancement
พารามิเตอร์ที่ปรับแต่งภาพหลังประมวลผล

```yaml
image_enhancement:
  brightness: null                  # ความสว่าง - ใช้ default
  contrast: null                    # ความคมชัด - ใช้ default
  gamma: null                       # การแปลงค่าความสว่าง - ใช้ default
  sharpness: null                   # ความคมชัด - ใช้ default
  notes: "Using GStreamer defaults, no post-processing applied"
```

**Why it matters:**
- No post-processing = ภาพดิบ (raw-like)
- Consistent pipeline = ไม่มี hidden adjustments

---

### 4. Noise & Dynamic Range
พารามิเตอร์ที่จัดการสัญญาณรบกวนและช่วงไดนามิก

```yaml
noise_and_dynamic_range:
  denoise: null                     # ลดสัญญาณรบกวน - ใช้ default
  temporal_noise_reduction: null    # ลด noise แบบ temporal - ใช้ default
  backlight_compensation: false     # ชดเชยแสงหลัง - ปิด
  hdr: false                        # High Dynamic Range - ปิด
  notes: "Standard dynamic range, no special noise reduction"
```

**Why it matters:**
- Standard settings = predictable behavior
- No HDR = ภาพไม่ถูก blend หลายค่า exposure

---

### 5. Focus & Aperture
พารามิเตอร์ที่ควบคุมโฟกัสและรูรับแสง

```yaml
focus_and_aperture:
  focus_mode: "manual"              # โหมดโฟกัส
  focus_value_left: 176.5           # ค่าโฟกัสกล้องซ้าย (manual adjustment)
  focus_value_right: 171.0          # ค่าโฟกัสกล้องขวา (manual adjustment)
  focus_difference: 5.5             # ความต่างโฟกัสซ้าย-ขวา
  aperture: "fixed"                 # รูรับแสง (IMX219 = fixed aperture)
  notes: "Focus manually adjusted and locked before collection"
```

**Why it matters:**
- Manual focus = ความชัดคงที่
- Focus values logged = reproduce ได้แม่นยำ
- Critical for stereo accuracy!

---

### 6. Frame & Timing
พารามิเตอร์ที่เกี่ยวกับ frame rate และ resolution

```yaml
frame_and_timing:
  frame_rate_fps: 15                # Frame rate (frames per second)
  resolution_width: 1280            # ความกว้าง (pixels)
  resolution_height: 720            # ความสูง (pixels)
  pixel_format: "NV12 -> BGRx -> BGR"  # Pipeline format conversions
  ae_antibanding_hz: null           # ป้องกันไฟกะพริบ (50/60Hz) - ไม่ตั้ง
  notes: "15 FPS for stable capture with 3s countdown"
```

**Why it matters:**
- 15 FPS = stable, not too fast
- Resolution = ตรงกับ calibration
- Pixel format = ต้องรู้เพื่อ debug color issues

---

### 7. External Lighting
พารามิเตอร์ที่เกี่ยวกับแสงภายนอก (สำคัญมาก!)

```yaml
external_lighting:
  led_count: 3                      # จำนวน LED
  led_type: "White LED strips"      # ชนิด LED
  led_positions:
    - name: "Top LED"
      position: "overhead"
      distance_from_target_cm: 10
      angle_deg: 0
      notes: "Mounted above camera, reduces shadows"
    - name: "Left LED"
      position: "left-front diagonal"
      distance_from_target_cm: 10
      angle_deg: 45
      notes: "Diagonal lighting from left side"
    - name: "Right LED"
      position: "right-front diagonal"
      distance_from_target_cm: 10
      angle_deg: 45
      notes: "Diagonal lighting from right side"
  ambient_light: "indoor"
  notes: "3-point LED setup optimized for even illumination"
```

**Why it matters:**
- Lighting = ปัจจัยสำคัญที่สุดใน vision!
- 3-point setup = even illumination, minimal shadows
- Distance & angle logged = reproduce ได้

---

## 🔧 Hardware Setup

ข้อมูล physical setup ของกล้อง

```yaml
hardware_setup:
  camera_model: "IMX219-83 Stereo Camera"
  sensor_resolution: "8MP (3280x2464)"
  fov_deg: 160                      # Field of view (degrees)
  baseline_mm: 60.57                # ระยะห่างระหว่างกล้อง (จาก calibration)
  focal_length_px: 1234.56          # Focal length (pixels, จาก calibration)
  camera_height_mm: 320             # ความสูงกล้องจากพื้น
  camera_angle: "top-down view (perpendicular to surface)"
  mounting: "fixed tripod mount"
```

**Why it matters:**
- Baseline & focal length = depth accuracy depends on these!
- Camera height = affects working distance
- Mounting = stability affects image quality

---

## 🌍 Environment

ข้อมูลสภาพแวดล้อม

```yaml
environment:
  background: "gray cloth"
  background_material: "non-reflective fabric"
  surface: "flat table with gray cloth"
  working_distance_cm: "23-35"      # ระยะจากกล้องถึงวัตถุ
  temperature_c: null               # อุณหภูมิ (ไม่ได้วัด)
  humidity_percent: null            # ความชื้น (ไม่ได้วัด)
  notes: "Controlled indoor environment with gray background"
```

**Why it matters:**
- Background = affects depth computation (texture!)
- Working distance = optimal range for accuracy

---

## 🔍 GStreamer Pipeline

ข้อมูล GStreamer pipeline ที่ใช้

```yaml
gstreamer_pipeline:
  source: "nvarguscamerasrc"
  wbmode: 0                         # Manual white balance
  aelock: true                      # Lock auto-exposure
  flip_method: 0                    # ไม่หมุนภาพ
  full_pipeline: "nvarguscamerasrc sensor-id=<ID> wbmode=0 aelock=true ..."
```

**Why it matters:**
- Complete pipeline = ทำซ้ำได้เป๊ะ
- Debug issues = รู้ว่าใช้ parameters อะไร

---

## 📐 Stereo Calibration

ข้อมูลการ calibrate กล้อง stereo

```yaml
stereo_calibration:
  calibration_file: "stereo_calib.yaml"
  calibration_date: null            # จะอ่านจาก file ถ้ามี
  pattern_type: "Asymmetric Circles Grid (5x6, 33 circles)"
  pattern_spacing_mm: 18            # ระยะห่างจุดบน pattern
  baseline_measured_mm: 60.57       # วัดจาก calibration
  focal_length_px: 1234.56          # คำนวณจาก calibration
```

**Why it matters:**
- Calibration = foundation of stereo vision!
- Pattern info = ทำซ้ำการ calibrate ได้
- Baseline & focal = ต้องรู้เพื่อคำนวณ depth

---

## 📊 Dataset Information

ข้อมูล dataset session

```yaml
dataset_info:
  session_name: "session1_red_large"
  output_directory: "pepper_dataset/session1_red_large"
  save_mode: "full (left + right + depth)"
  depth_computation: "StereoSGBM"
  depth_range_mm: "150-1200"        # ช่วงความลึกที่ valid
```

**Why it matters:**
- Session tracking = รู้ว่า dataset มาจากไหน
- Save mode = รู้ว่ามีข้อมูลอะไรบ้าง
- Depth range = ทราบข้อจำกัดของระบบ

---

## 🎯 Use Cases

### 1. Reproduce Experiment
เมื่อต้องการเก็บข้อมูลชุดใหม่ด้วย settings เดียวกัน:
- อ่าน `hardware_config.yaml` จาก session ก่อน
- ตั้งค่า camera, lighting, environment ให้เหมือนกัน
- รัน `collect_dataset.py` ด้วย parameters เดียวกัน

### 2. Compare Sessions
เมื่อต้องการเปรียบเทียบ dataset 2 ชุด:
- เปิด `hardware_config.yaml` จากทั้ง 2 sessions
- เปรียบเทียบ parameters ที่แตกต่าง
- วิเคราะห์ผลกระทบต่อ model performance

### 3. Debug Issues
เมื่อเจอปัญหาภาพไม่ดี:
- เช็ค exposure/gain → สว่างเกิน/น้อยไป?
- เช็ค focus → ไม่ชัด?
- เช็ค lighting → เงามาก? ต้องเพิ่ม LED?

### 4. Academic Paper
เมื่อต้องเขียน paper วิจัย:
- Copy-paste จาก `hardware_config.yaml`
- ใส่ใน Methods section
- Reviewers จะชอบเพราะมีรายละเอียดครบ!

---

## ✅ Checklist: Parameters ต้องตรงกันทุก Session

เมื่อเก็บข้อมูลหลาย sessions ต้องแน่ใจว่า:

**Must Match (ต้องเหมือนกัน):**
- ✅ Exposure time & Gain
- ✅ White balance mode
- ✅ Focus values
- ✅ Resolution
- ✅ LED positions & distances
- ✅ Camera height
- ✅ Background

**Can Vary (แตกต่างกันได้):**
- ✅ Session name
- ✅ Number of peppers
- ✅ Collection date

---

## 📝 Example: Session Comparison

```yaml
# Session 1: Red Large
exposure_time_ms: 30
gain_iso: 2
focus_value_left: 176.5
led_count: 3

# Session 2: Green Medium
exposure_time_ms: 30        # ✅ Same
gain_iso: 2                 # ✅ Same
focus_value_left: 176.5     # ✅ Same
led_count: 3                # ✅ Same

→ Good! Consistent setup
```

---

## 🚀 Summary

`hardware_config.yaml` บันทึก:
1. ✅ Camera parameters ทั้ง 7 หมวด
2. ✅ Hardware setup (baseline, focal, height)
3. ✅ Environment (background, distance)
4. ✅ Lighting (LED positions)
5. ✅ GStreamer pipeline
6. ✅ Calibration info
7. ✅ Dataset metadata

**Result:** Complete reproducibility! 🎉

---

**Created:** 2025-10-31
**Version:** 1.0
**Tool:** collect_dataset.py (modified with hardware_config support)
