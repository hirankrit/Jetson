# 🔄 Dataset Re-Collection Guide (Version 2.0)

**Date:** Oct 29, 2025
**Reason:** Re-collect with `aelock=true` for consistent focus and sharp texture
**Status:** Ready to start

---

## 📊 Overview

**Original Dataset Issues:**
- ❌ Auto-focus changing (blurry images)
- ❌ Texture not visible
- ❌ Inconsistent quality

**New Dataset (V2):**
- ✅ `aelock=true` enabled (fixed focus)
- ✅ Sharp texture on all images
- ✅ 100% consistent quality
- ✅ Ready for YOLO training

---

## 🎯 Collection Plan

### Total Dataset: 192 images (16 peppers)

**Session 1: Red Large Peppers** - 96 images
- 8 peppers × 12 angles each

**Session 2: Red Defect Types** - 48 images
- 4 peppers × 12 angles each
  - 2.1: Red rotten (เน่า)
  - 2.2: Red insect (แมลงเจาะ)
  - 2.3: Red deformed (งอ)
  - 2.4: Red wrinkled (เหี่ยว)

**Session 3: Green Varieties** - 48 images
- 4 peppers × 12 angles each
  - 3.1: Green medium (ขนาดกลาง)
  - 3.2: Green small (ขนาดเล็ก)
  - 3.3: Green tiny (ขนาดเล็กมาก)
  - 3.4: Green rotten (เน่า)

---

## ⏱️ Time Estimate

| Session | Peppers | Images | Time |
|---------|---------|--------|------|
| Session 1 | 8 | 96 | ~32 min |
| Session 2 | 4 | 48 | ~16 min |
| Session 3 | 4 | 48 | ~16 min |
| **Total** | **16** | **192** | **~64 min** |

---

## 🚀 Quick Start

### Step 1: Setup (Backup old data)

```bash
# Run setup script (backup old + create new structure)
bash setup_new_dataset.sh
```

This will:
- Backup old dataset → `pepper_dataset_OLD_YYYYMMDD_HHMMSS/`
- Create new empty folder → `pepper_dataset/`

---

### Step 2: Session 1 - Red Large Peppers (8 peppers)

**Prepare:** 8 red large peppers

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session1_red_large
```

**Workflow per pepper:**
1. Place pepper in center
2. Remove hand immediately
3. Wait 3-5 seconds (camera stabilize)
4. Press SPACE → countdown 3-2-1 → 📸
5. Rotate pepper ~30° (quickly in 2-3s)
6. Repeat 12 times (12 angles)

**Expected:** 96 images total (8 × 12)

---

### Step 3: Session 2 - Red Defects (4 peppers)

**Prepare:** 1 rotten, 1 insect, 1 deformed, 1 wrinkled

#### Session 2.1: Red Rotten (พริกแดงเน่า)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_rotten
```

**Expected:** 12 images (1 pepper × 12 angles)

#### Session 2.2: Red Insect (พริกแดงแมลงเจาะ)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_insect
```

**Expected:** 12 images

#### Session 2.3: Red Deformed (พริกแดงงอ)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_deformed
```

**Expected:** 12 images

#### Session 2.4: Red Wrinkled (พริกแดงเหี่ยว)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session2_red_wrinkled
```

**Expected:** 12 images

**Total Session 2:** 48 images (4 × 12)

---

### Step 4: Session 3 - Green Varieties (4 peppers)

**Prepare:** 1 medium, 1 small, 1 tiny, 1 rotten green peppers

#### Session 3.1: Green Medium (เขียวขนาดกลาง)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_medium
```

**Expected:** 12 images

#### Session 3.2: Green Small (เขียวขนาดเล็ก)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_small
```

**Expected:** 12 images

#### Session 3.3: Green Tiny (เขียวขนาดเล็กมาก)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_tiny
```

**Expected:** 12 images

#### Session 3.4: Green Rotten (เขียวเน่า)

```bash
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_rotten
```

**Expected:** 12 images

**Total Session 3:** 48 images (4 × 12)

---

## ✅ Verification

### After each session:

```bash
# Count images in current session
find pepper_dataset/session*_* -name "*.jpg" | wc -l

# Count per folder
for dir in pepper_dataset/session*_*; do
  echo "$(basename $dir): $(find $dir -name '*.jpg' | wc -l) images"
done
```

### After all sessions:

```bash
# Total count
find pepper_dataset -name "*.jpg" | wc -l
# Expected: 192 images

# Summary
echo "=== Dataset Summary ==="
echo "Session 1: $(find pepper_dataset/session1_* -name '*.jpg' | wc -l) images"
echo "Session 2: $(find pepper_dataset/session2_* -name '*.jpg' | wc -l) images"
echo "Session 3: $(find pepper_dataset/session3_* -name '*.jpg' | wc -l) images"
echo "Total: $(find pepper_dataset -name '*.jpg' | wc -l) images"
```

---

## 📋 Checklist

### Before Starting:
- [ ] Run `bash setup_new_dataset.sh` (backup old data)
- [ ] Prepare all peppers (16 total):
  - [ ] 8 red large
  - [ ] 1 red rotten, 1 red insect, 1 red deformed, 1 red wrinkled
  - [ ] 1 green medium, 1 green small, 1 green tiny, 1 green rotten
- [ ] Check camera (exposure=24ms, gain=2, aelock=true)
- [ ] Check lighting (3 LEDs on)
- [ ] Check background (dark cloth ready)

### During Collection:
- [ ] Place pepper → remove hand → wait 3-5s
- [ ] Press SPACE → countdown → capture
- [ ] Rotate ~30° → repeat 12 times
- [ ] Verify images are sharp (texture visible)

### After Each Session:
- [ ] Count images (should be 12 × peppers)
- [ ] Quick check image quality
- [ ] Mark session as complete

### After All Sessions:
- [ ] Total count: 192 images ✅
- [ ] All images sharp ✅
- [ ] Texture visible on all ✅
- [ ] Ready for annotation ✅

---

## 💡 Tips for Best Results

1. **Wait 3-5 seconds** after placing pepper (most important!)
2. **Remove hand quickly** after placing
3. **Don't move hand during countdown**
4. **Rotate pepper smoothly** (~30° each time)
5. **Keep consistent distance** (320mm camera height)
6. **Use same lighting** (3 LEDs, same position)
7. **Check focus occasionally** (texture should be visible)

---

## 🎯 Success Criteria

**Each image should have:**
- ✅ Sharp focus (no blur)
- ✅ Visible texture (surface details clear)
- ✅ Good contrast (pepper vs background)
- ✅ Centered object (pepper in middle of frame)
- ✅ No hand visible (completely out of frame)
- ✅ Consistent lighting (no shadows, no glare)

**If image doesn't meet criteria:**
- Re-capture that angle immediately
- Don't continue if quality is bad

---

## 📊 Expected Output Structure

```
pepper_dataset/
├── session1_red_large/
│   ├── raw/left/
│   │   └── pepper_0000-0095.jpg  (96 images)
│   └── metadata/
│       └── collection_log.yaml
│
├── session2_red_rotten/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│
├── session2_red_insect/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│
├── session2_red_deformed/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│
├── session2_red_wrinkled/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│
├── session3_green_medium/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│
├── session3_green_small/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│
├── session3_green_tiny/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│
└── session3_green_rotten/
    ├── raw/left/
    │   └── pepper_0000-0011.jpg  (12 images)
    └── metadata/

Total: 192 images (16 peppers × 12 angles)
```

---

## 🆘 Troubleshooting

**Problem: Images still blurry**
- Wait longer (5-10s) after placing pepper
- Check if aelock is enabled in code
- Verify exposure=24, gain=2

**Problem: Hand visible in frame**
- Remove hand faster after placing
- Wait longer before pressing SPACE
- Use countdown time to get hand out

**Problem: Camera quality changes**
- Restart camera between sessions
- Check lighting hasn't changed
- Verify settings: exposure=24, gain=2

**Problem: Taking too long**
- Don't wait too long (3-5s is enough)
- Rotate pepper quickly (2-3s)
- Have all peppers ready before starting

---

## 🎉 After Completion

**You will have:**
- ✅ 192 high-quality images
- ✅ Consistent sharp texture
- ✅ Perfect for YOLO training
- ✅ 100% usable dataset

**Next steps:**
1. Annotation (Roboflow/LabelImg)
2. Train/Val split
3. YOLO training (Week 3)

---

**Good luck! You're creating a professional-quality dataset! 🌶️🤖**
