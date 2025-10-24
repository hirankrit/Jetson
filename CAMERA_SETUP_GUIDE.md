# 📸 Camera Setup Guide - IMX219 Stereo Camera

> **Last Updated**: 2025-10-24
> **Status**: ✅ Optimized for 30-50cm working range

---

## 📋 Table of Contents

1. [Focus Settings](#focus-settings)
2. [Lighting Setup](#lighting-setup)
3. [Working Range](#working-range)
4. [Verification Checklist](#verification-checklist)
5. [Troubleshooting](#troubleshooting)

---

## 🎯 Focus Settings

### Optimal Focus Configuration

**Measurement Distance**: **32cm** (center of working range)

| Camera | Focus Value | Status |
|--------|-------------|--------|
| **Left** | **176.5** | ✅ Excellent |
| **Right** | **171.0** | ✅ Excellent |
| **Difference** | **6.0** | ✅ < 10 (Acceptable) |

### Focus Measurement Method

- **Metric**: Laplacian Variance (higher = sharper)
- **Tool**: `test_camera_focus.py` (if available) or manual measurement
- **Target**: Diff < 10 between cameras

### 🔒 Important Notes

- ⚠️ **Lock focus rings** after adjustment (use tape if needed)
- ⚠️ **Don't touch focus** during operation
- ⚠️ **Re-check focus** if images appear blurry
- ⚠️ **Lighting changes** may require focus re-adjustment

---

## 💡 Lighting Setup

### Configuration (2025-10-24)

```
📍 Position: ซ้ายหน้ากล้อง (Left-front of camera)
📐 Angle:    ทะแยงเข้าหาวัตถุ (Diagonal toward object)
💡 Type:     LED
📏 Distance: 10cm (from light to working area)
```

### Lighting Diagram

```
                Camera (Top View)
                    ↓↓
              [Left] [Right]
                  ||
                  ||
                  \/
           Working Area (32cm)
              [Object]
             /
            /  (Diagonal)
           /
      [💡 LED]
      (10cm distance)
```

### Lighting Best Practices

✅ **Do:**
- Use consistent lighting (same LED, same position)
- Position light to minimize shadows
- Ensure even illumination across working area
- Check for reflections on shiny objects (peppers)

❌ **Don't:**
- Use direct overhead lighting (creates harsh shadows)
- Mix natural and artificial light (inconsistent)
- Place light too close (overexposure)
- Change lighting without re-checking focus

### Recommended Settings

- **LED Power**: Medium intensity (avoid saturation)
- **Color Temperature**: Daylight (5000-6500K) preferred
- **Diffusion**: Optional diffuser for softer light

---

## 📏 Working Range

### Tested Ranges

| Distance | Focus Quality | Expected Accuracy |
|----------|--------------|-------------------|
| **30cm** | ✅ Sharp | ±1-2cm (excellent) |
| **32cm** | ✅✅ Optimized | ±1cm (best) |
| **40cm** | ✅ Sharp | ±2-3cm (good) |
| **50cm** | ⚠️ TBD | ±3-4cm (target) |
| **60cm+** | ❌ Out of range | N/A |

### Focus Depth of Field

- **Sweet Spot**: 30-40cm (±5cm from focus distance)
- **Acceptable**: 28-45cm
- **Target for Pepper Sorting**: 30-50cm

---

## ✅ Verification Checklist

### Before Each Session

- [ ] Check focus lock (not rotated)
- [ ] Verify lighting position (left-front, diagonal)
- [ ] Confirm LED is on (10cm distance)
- [ ] Run quick focus test:
  ```bash
  python3 view_camera.py
  ```
- [ ] Visual check: both images sharp at 32cm

### Focus Verification Test

```bash
# 1. Run camera viewer
python3 view_camera.py

# 2. Place object at 32cm
# 3. Visual check: both images sharp
# 4. If using focus measurement script:
#    - Left should be ~176
#    - Right should be ~171
#    - Diff should be <10
```

### Depth Map Verification

```bash
# 1. Run depth map test
python3 test_depth_map_enhanced.py

# 2. Test at known distances:
#    - 30cm → should read 28-32cm
#    - 32cm → should read 30-34cm
#    - 40cm → should read 37-43cm
#    - 50cm → should read 47-53cm

# 3. Click on object to measure
# 4. Compare measured vs actual distance
```

**Pass Criteria:**
- ✅ Error < ±2cm at 30-32cm
- ✅ Error < ±3cm at 40-50cm

---

## 🔧 Troubleshooting

### Problem: Images are blurry

**Possible Causes:**
1. Focus ring rotated accidentally
2. Camera moved
3. Lighting changed

**Solutions:**
- Re-check focus values (~176 left, ~171 right)
- Lock focus rings with tape
- Verify lighting position

### Problem: Depth map inaccurate

**Possible Causes:**
1. Focus mismatch between cameras (Diff > 10)
2. Lighting too dim/bright
3. Object out of working range

**Solutions:**
- Re-run focus verification
- Adjust lighting intensity
- Keep objects within 30-50cm

### Problem: One camera sharper than other

**Possible Causes:**
1. Focus values too different
2. One lens dirty

**Solutions:**
- Re-adjust focus to minimize Diff
- Clean lenses with microfiber cloth
- Target: Diff < 10

### Problem: Inconsistent results

**Possible Causes:**
1. Lighting position changed
2. Ambient light interference
3. Camera vibration

**Solutions:**
- Mark lighting position (tape on desk)
- Block ambient light (curtains/shade)
- Ensure stable camera mount

---

## 📸 Setup Photos (TODO)

- [ ] Photo: Lighting position (left-front, diagonal)
- [ ] Photo: Overall setup (camera + light + working area)
- [ ] Photo: Distance measurement (32cm reference)
- [ ] Photo: Focus lock mechanism (if any)

---

## 🔄 Maintenance Schedule

### Daily
- [ ] Visual check: images sharp
- [ ] Verify lighting position

### Weekly
- [ ] Run full focus verification
- [ ] Run depth map accuracy test
- [ ] Clean lenses if needed

### Monthly
- [ ] Full re-calibration (if accuracy degrades)
- [ ] Document any changes

---

## 📝 Change Log

| Date | Change | Reason |
|------|--------|--------|
| 2025-10-24 | Initial setup documented | Focus optimized at 176.5/171.0 |
| | LED light: left-front, 10cm | Achieved Diff < 10 |

---

## 🎯 Next Steps

After verifying this setup works:

1. ✅ **Backup this configuration**
   ```bash
   git add CAMERA_SETUP_GUIDE.md
   git commit -m "docs: Camera setup guide with focus + lighting"
   git push
   ```

2. ⏳ **Run depth map tests** (see Verification Checklist)

3. ⏳ **Document results** in Weekly Report

4. ⏳ **If accuracy is good**: Lock this as baseline setup

5. ⏳ **If needs adjustment**: Update this guide with new values

---

**Happy Calibrating! 📸**
