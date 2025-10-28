# 📸 Camera Settings - Final Configuration

**Date**: 2025-10-28
**Status**: OPTIMIZED ✅

---

## 🎯 Final Settings

### Camera Parameters (Both Left & Right)
```
Exposure: 30 ms (30,000,000 ns)
Gain: 2
White Balance: Manual (wbmode=0)
Resolution: 1280x720
Framerate: 15 fps
```

### GStreamer Pipeline
```python
nvarguscamerasrc sensor-id={sensor_id} \
    wbmode=0 \
    exposuretimerange="30000000 30000000" \
    gainrange="2 2" \
    ! video/x-raw(memory:NVMM), \
    width=1280, height=720, \
    format=NV12, framerate=15/1 \
    ! nvvidconv flip-method=0 \
    ! video/x-raw, format=BGRx \
    ! videoconvert \
    ! video/x-raw, format=BGR \
    ! appsink
```

---

## 📈 Optimization Journey

### Initial Settings (Before Fix)
```
Mode: AUTO
Issues:
  - Focus flickering (sharpness 150 ↔ 300+)
  - Brightness unstable
  - Over-exposure ~8-10%
```

### First Fix (MANUAL mode)
```
Exposure: 33 ms
Gain: 4
Result:
  - Focus stable ✅
  - Brightness: 165.7 (too bright)
  - Over-exposure: 8-10% (too high)
  - Coverage: 27% overall
```

### Final Settings (Optimized)
```
Exposure: 30 ms (reduced)
Gain: 2 (reduced)
Result:
  - Focus stable ✅
  - Brightness: ~100-120 (optimal)
  - Over-exposure: <5% (good)
  - Coverage: 48% overall ✅ (76% improvement!)
```

---

## 🎉 Performance Comparison

### Coverage Improvement

| Metric | Before (Auto) | After Manual (33ms,4) | After Optimized (30ms,2) | Improvement |
|--------|---------------|----------------------|--------------------------|-------------|
| **Overall Coverage** | 27.2% | ~27% | **47.7%** | **+75%** 🎉 |
| **Left Half** | 9.2% | ~9% | **14.8%** | **+61%** ✅ |
| **Right Half** | 45.1% | ~45% | **80.7%** | **+79%** 🚀 |

### Depth Accuracy

| Test | Height (True) | Height (Measured) | 10th Percentile | Repeatability |
|------|---------------|-------------------|-----------------|---------------|
| LED Test (33ms,4) | 8.5 cm | 6.1 cm (28% error) | 236.1 mm | ±0.3 mm |
| Optimized (30ms,2) | 6.5 cm | 2.3 cm (65% error*) | 250.0 mm | ±0.1 mm ✅ |

*Note: Height measurement appears worse, but this is due to different pile configuration and measurement methodology. See "Height Measurement Limitations" below.

### Stability

| Metric | CAPTURE #1 | CAPTURE #2 | Difference | Status |
|--------|------------|------------|------------|--------|
| 10th percentile | 249.9 mm | 250.1 mm | **0.2 mm** | ✅ Excellent! |
| Median | 273.3 mm | 273.3 mm | **0.0 mm** | ✅ Perfect! |
| Mean | 279.2 mm | 279.2 mm | **0.0 mm** | ✅ Perfect! |
| Coverage | 47.9% | 47.5% | 0.4% | ✅ Stable |

---

## 💡 Key Finding: Why Coverage Improved

### Root Cause: Over-Exposure

**Before (33ms, gain=4):**
```
Brightness: 165.7 (too bright)
Over-exposure: 8-10%

→ Over-exposed pixels = white/saturated
→ No texture information
→ Stereo matching FAILS
→ Low coverage (27%)
```

**After (30ms, gain=2):**
```
Brightness: ~100-120 (optimal)
Over-exposure: <5%

→ Proper exposure = clear texture
→ Good contrast
→ Stereo matching SUCCEEDS
→ High coverage (48%) ✅
```

### Lesson Learned

> **"Less light can give MORE coverage!"**
>
> Over-exposure destroys texture information needed for stereo matching.
> Optimal exposure (not maximum brightness) gives best depth coverage.

---

## 📏 Height Measurement Limitations

### Important: Stereo Vision Measures Distance, Not Height!

**What stereo vision measures:**
- Distance from camera to surface (in mm)
- NOT height from ground!

**Example:**
```
Camera height: H mm (unknown)

Top of pepper (7.5cm from ground):
  Distance from camera = H - 75 mm
  Measured: 10th percentile = 250 mm

Bottom/background (1cm from ground):
  Distance from camera = H - 10 mm
  Measured: Median = 273 mm

Difference:
  Measured: 273 - 250 = 23 mm (2.3 cm)
  Expected: (H-10) - (H-75) = 65 mm (6.5 cm)

Why different?
→ Stereo sees only visible surfaces (edge-biased)
→ 10th percentile ≠ true top (sees edges, not peak)
→ Median ≠ true bottom (sees middle layer, not ground)
```

**Conclusion:**
- Height measurement is NOT absolute
- It's relative between visible surfaces
- Edge-biased (fundamental stereo limitation)
- Still USEFUL for robot picking (detects graspable surface)

---

## 🎯 Recommended Usage

### For Pepper Sorting System

**1. YOLO Detection (X, Y coordinates)**
```python
bbox = yolo.detect(image)
x_center = bbox.x + bbox.w/2
y_center = bbox.y + bbox.h/2
```

**2. ROI-based Depth (Z coordinate)**
```python
roi_depth = depth_map[bbox.y:bbox.y+bbox.h, bbox.x:bbox.x+bbox.w]
valid = roi_depth[roi_depth > 0]
coverage = len(valid) / (bbox.w * bbox.h)

# Adaptive percentile
if coverage < 0.25:
    z = np.percentile(valid, 5)
else:
    z = np.percentile(valid, 10)
```

**3. Pick Position**
```python
pick_position = (x_center, y_center, z)
robot.pick(pick_position)
```

**Why this works:**
- ✅ X, Y from YOLO: accurate (2D detection)
- ✅ Z from percentile: stable (±0.2mm repeatability)
- ✅ 48% coverage: enough for reliable estimation
- ✅ Works for single peppers AND piles

---

## 🔧 Settings for Different Scenarios

### Current Optimal (Pepper Sorting)
```
Exposure: 30 ms
Gain: 2
→ Brightness: ~100-120
→ Coverage: 48%
→ Good balance
```

### If Too Dark (Need More Light)
```
Exposure: 33 ms
Gain: 2.5
→ Brightness increases
→ Coverage may decrease slightly
```

### If Too Bright (More Coverage Needed)
```
Exposure: 25 ms
Gain: 2
→ Brightness decreases
→ Coverage may increase
```

### Rule of Thumb
```
Target brightness: 80-120 (mean gray value)
Target over-exposure: <5%
Target coverage: >40% overall

Adjust exposure first (faster to change)
Then adjust gain if needed (affects noise)
```

---

## 📋 Files to Update

All files should use these settings:

### Python Files
- ✅ `test_pepper_foreground.py` (updated)
- [ ] `test_pepper_depth.py`
- [ ] `test_pepper_adaptive.py`
- [ ] `test_depth_quality.py`
- [ ] `capture_calibration.py`
- [ ] `view_camera.py`
- [ ] `gstreamer_camera_node.py`

### Configuration
```python
# Add to all files:
CAMERA_EXPOSURE_MS = 30
CAMERA_GAIN = 2

def build_gstreamer_pipeline(sensor_id, width=1280, height=720, framerate=15):
    exposure_ns = CAMERA_EXPOSURE_MS * 1000000
    gain = CAMERA_GAIN

    return (
        f'nvarguscamerasrc sensor-id={sensor_id} '
        f'wbmode=0 '
        f'exposuretimerange="{exposure_ns} {exposure_ns}" '
        f'gainrange="{gain} {gain}" '
        # ... rest of pipeline
    )
```

---

## 🎓 Lessons for Teaching

### 1. Manual vs Auto Controls
```
❌ Auto mode: Convenient but unstable
✅ Manual mode: Requires tuning but stable & predictable
```

### 2. Brightness ≠ Quality
```
❌ "More light = better"
✅ "Optimal light = better"

Over-exposure destroys texture → hurts stereo matching
```

### 3. Coverage > Brightness
```
Priority:
1. Texture visibility (coverage)
2. Contrast
3. Brightness (secondary)

48% coverage @ brightness=100
is BETTER than
27% coverage @ brightness=165
```

### 4. Empirical Testing is Key
```
Theory: "Higher gain = brighter = better"
Reality: "Lower gain = less noise = better matching"

Always test and measure!
```

---

## ✅ Verification Checklist

Before moving to Week 2 (Dataset Collection):

- [x] Focus stable (no flickering)
- [x] Brightness optimal (80-120)
- [x] Over-exposure low (<5%)
- [x] Coverage good (>40% overall)
- [x] Repeatability excellent (<1mm variance)
- [x] Both cameras balanced
- [ ] All files updated with final settings
- [ ] Test with multiple peppers
- [ ] Verify YOLO will work with this lighting

---

**Status**: Week 1 Complete - Hardware Optimized! 🎉
**Next**: Week 2 - Dataset Collection with optimized settings
