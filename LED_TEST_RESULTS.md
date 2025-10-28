# 💡 LED Lighting Test Results

**Test Date**: 2025-10-28
**Tool**: test_pepper_foreground.py
**Camera**: IMX219 Stereo (baseline 60.57mm)
**Resolution**: 1280x720

---

## 🌶️ Test Setup

### Object Configuration:
- **Type**: พริกกอง (multiple peppers in a pile)
- **Highest point**: 9.5 cm from ground (top of pile)
- **Lowest point**: 1.0 cm from ground (bottom pepper)
- **True height difference**: **8.5 cm**

### Lighting (BEFORE LED):
- **Type**: Side lighting only (LED ด้านข้าง)
- **Position**: ซ้ายหน้า, ทะแยงเข้าหาวัตถุ 10cm
- **Status**: Baseline test (ก่อนติด LED ด้านบน)

---

## 📊 BEFORE LED Installation - Results

### Test: Pepper Pile (N=2 captures)

| Capture | Coverage | Left Half | Right Half | 10%ile (mm) | Median (mm) | Diff (mm) | Std (mm) |
|---------|----------|-----------|------------|-------------|-------------|-----------|----------|
| #1      | 27.4%    | 9.3%      | 45.5%      | 236.3       | 297.4       | 61.1      | 55.8     |
| #2      | 26.9%    | 9.0%      | 44.7%      | 235.9       | 296.6       | 60.6      | 51.0     |
| **Avg** | **27.2%**| **9.2%**  | **45.1%**  | **236.1**   | **297.0**   | **60.9**  | **53.4** |
| **Std** | 0.4%     | 0.2%      | 0.6%       | 0.3         | 0.6         | 0.4       | 3.4      |

### Key Metrics (BEFORE LED):

**Depth Measurements:**
- ✅ **10th percentile** (nearest): 236.1 mm (23.6 cm) ± 0.3 mm
- ✅ **Median** (middle): 297.0 mm (29.7 cm) ± 0.6 mm
- ✅ **Height difference** (Median - 10%ile): **6.1 cm** ± 0.04 cm

**Coverage:**
- ⚠️ **Overall**: 27.2% ± 0.4%
- ❌ **Left half**: 9.2% ± 0.2% (ต่ำมาก!)
- ✅ **Right half**: 45.1% ± 0.6%

**Repeatability:**
- ✅ **Excellent!** 10%ile std = 0.3 mm (< 1mm)

---

## 🔍 Analysis

### 1. Height Measurement Accuracy

**Comparison:**
```
True height difference:     8.5 cm (ยอดพริก - พื้น)
Measured (Median - 10%ile): 6.1 cm
Error:                      2.4 cm (28% under-estimate)
```

**Interpretation:**
- ✅ **Better than previous test!** (ครั้งก่อน: 5cm → 0.5cm = 90% error)
- ⚠️ **Still under-estimates** by ~28%
- 💡 **Why?**
  - 10%ile catches near the top (ไม่ใช่ยอดสุด)
  - Median catches middle peppers (ไม่ใช่พื้น)
  - Both are biased towards **visible edges** (not actual extremes)

**Expected behavior:**
- ✅ 10%ile should be close to **highest visible surface** (not air above peppers)
- ✅ Median should be close to **middle layer** (not ground)
- ✅ This is **physics limitation** of stereo vision (edge-biased)

**If we account for camera height:**
- Camera is at height H from ground
- Pepper top @ 9.5cm → Distance = H - 9.5cm
- Pepper bottom @ 1cm → Distance = H - 1cm
- **But we don't know H!**

**Alternative interpretation:**
- Maybe camera sees background (table/floor) as reference
- 10%ile = 23.6cm = closest visible surface (pepper top edges)
- Median = 29.7cm = middle depth (pepper pile center + background)
- Diff = 6.1cm = rough estimate of pile "thickness" in depth direction

---

### 2. Coverage Analysis

**Left vs Right Asymmetry:**
```
Right half: 45.1% ✅ (พอใช้ได้)
Left half:   9.2% ❌ (ต่ำมาก - ปัญหาหลัก!)
Ratio:      4.9x difference
```

**Possible Causes:**

**A. Occlusion (most likely)**
- พริกกองสูง → blocks left camera view
- Right camera sees more of the pile
- Left camera only sees edges

**B. Lighting**
- LED ด้านข้างซ้าย → shadows on left camera view?
- Need to verify lighting symmetry

**C. Geometry**
- Pile shape not symmetric → naturally occludes left camera

**Expected Improvement with Top LED:**
- 💡 Top lighting → illuminates center/top → should improve left coverage
- 🎯 Target: Left half > 20% (2x improvement)

---

### 3. Depth Statistics Deep Dive

**Full Statistics (CAPTURE #1):**
```
Min depth:         185 mm (18.5 cm) - probably pepper top edge
Max depth:        1165 mm (116.5 cm) - background (floor/wall)

5th percentile:    233 mm (23.3 cm) - aggressive foreground
10th percentile:   236 mm (23.6 cm) ⭐ RECOMMENDED
15th percentile:   239 mm (23.9 cm)
20th percentile:   242 mm (24.2 cm)
25th percentile:   245 mm (24.5 cm)

Median:            297 mm (29.7 cm) - middle layer
Mean:              283 mm (28.3 cm) - biased by background

Std Dev:           55.8 mm (5.6 cm) - high spread (pile + background)
```

**Observations:**
- ✅ 5-10%ile very stable (233-236mm = 3mm spread)
- ✅ 10-25%ile smooth progression (236-245mm = 9mm over 15%ile)
- ⚠️ Median far from 10%ile (61mm diff = pile depth + background)
- ⚠️ High std dev (56mm) = mix of pile + background

**Recommendation:**
- ✅ Use **10th percentile** for picking depth (nearest surface)
- ✅ Very stable across captures (±0.3mm)
- ✅ Represents "accessible top surface" for robot arm

---

### 4. Repeatability Assessment

**Consistency between captures:**
```
Metric              | CAPTURE #1 | CAPTURE #2 | Diff  | % Diff
--------------------|------------|------------|-------|--------
Coverage (overall)  | 27.4%      | 26.9%      | 0.5%  | 1.8%
Coverage (left)     | 9.3%       | 9.0%       | 0.3%  | 3.2%
Coverage (right)    | 45.5%      | 44.7%      | 0.8%  | 1.8%
10th percentile     | 236.3mm    | 235.9mm    | 0.4mm | 0.2%
Median              | 297.4mm    | 296.6mm    | 0.8mm | 0.3%
```

**Verdict:**
- ✅ **Excellent repeatability!**
- ✅ Coverage variation < 2%
- ✅ Depth variation < 1mm
- ✅ System is **stable and reliable**

---

## 🎯 Hypotheses for LED Top Lighting

### Expected Improvements:

**1. Coverage** (Primary goal)
- ❌ Current left half: 9.2%
- 🎯 Target left half: > 20% (2x improvement)
- 🎯 Target overall: > 35% (30% improvement)

**Why?**
- Top lighting → illuminates center of pile → more texture visible
- Reduces shadows → better stereo matching on left camera

**2. Depth Accuracy** (Should maintain)
- ✅ Current 10%ile: 236.1mm ± 0.3mm
- 🎯 Target: ±2mm (no degradation)

**Why might it degrade?**
- ⚠️ Over-exposure → saturation → loss of texture
- ⚠️ Specular reflection → different intensity between cameras

**3. Height Measurement** (Might improve)
- ❌ Current: 6.1cm (true: 8.5cm, error: 28%)
- 🎯 Target: > 7.0cm (error < 20%)

**Why might it improve?**
- 💡 Better coverage at top → 10%ile closer to actual peak
- 💡 Better coverage at bottom → Median closer to actual bottom

---

## 📋 TODO: LED Installation Test

### Test Protocol:

**1. Install Top LED**
- Position: Above workspace, centered
- Type: LED (diffused if possible)
- Goal: Illuminate top/center of pepper pile

**2. Verify No Over-Exposure**
```bash
python3 capture_calibration.py
# Check:
#   - Over-exposure < 5%
#   - Brightness 50-200
#   - Status: GOOD or OK (not CHECK!)
```

**3. Repeat Pepper Pile Test (AFTER LED)**
```bash
python3 test_pepper_foreground.py
# Same pile configuration (9.5cm top, 1cm bottom)
# Capture 2-3 times
# Record all metrics
```

**4. Compare Results**
```
Metric                | BEFORE LED | AFTER LED | Δ        | Goal
----------------------|------------|-----------|----------|-------------
Coverage (overall)    | 27.2%      | ??%       | ??%      | +8-10%
Coverage (left half)  | 9.2%       | ??%       | ??%      | +10-15%
Coverage (right half) | 45.1%      | ??%       | ??%      | maintain
10th percentile       | 236.1mm    | ??mm      | ??mm     | ±2mm
Repeatability         | ±0.3mm     | ??mm      | --       | maintain
```

---

## 💡 Insights for System Design

### 1. Use 10th Percentile for Picking Depth
```python
# For robot arm picking:
pepper_depth = np.percentile(roi_depth[valid], 10)

# Why?
# - Stable (±0.3mm repeatability)
# - Represents nearest accessible surface
# - Robust to background noise
```

### 2. Accept Coverage Asymmetry
- Left/Right asymmetry is **normal** for piled objects
- Focus on improving **left half coverage** with top lighting
- Right half already good (45%)

### 3. Height Measurement Limitations
- Stereo vision measures **visible surface depth**, not **object height from ground**
- 28% under-estimate is **expected** (edge-biased)
- For true height: need **reference plane** (table/floor detection)

### 4. Next Steps After LED Test
**If Coverage Improves:**
- ✅ Keep LED setup
- 📸 Start dataset collection (Week 2)
- 🎯 Target: 500-1000 images with good coverage

**If No Improvement:**
- 🔍 Analyze why (photos, exposure metrics)
- 🔧 Try: diffuser, different angle, multiple LEDs
- 💭 Consider: current 27% might be good enough for sorted peppers (not piled)

---

## 📸 Saved Images

**Files:**
- `foreground_[timestamp]_left.jpg` - Left camera view
- `foreground_[timestamp]_depth.jpg` - Depth map visualization

**Rename for archival:**
```bash
mv foreground_[timestamp]_left.jpg before_led_pile_01_left.jpg
mv foreground_[timestamp]_depth.jpg before_led_pile_01_depth.jpg
```

---

**Status**: ✅ BEFORE LED baseline complete
**Next**: 🔧 Install top LED → Re-test → Compare
