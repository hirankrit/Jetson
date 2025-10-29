# 🌿 Session 3: Green Peppers Dataset Collection

**Date:** Oct 29, 2025
**Target:** 48 images (4 peppers × 12 angles)

---

## 📋 Session Overview

### 4 Sub-sessions:

1. **Session 3.1: Green Medium** (เขียวขนาดกลาง)
2. **Session 3.2: Green Small** (เขียวขนาดเล็ก)
3. **Session 3.3: Green Tiny** (เขียวขนาดเล็กมาก)
4. **Session 3.4: Green Rotten** (เขียวเน่า)

**Total:** 48 images (4 peppers × 12 angles)
**Estimated Time:** ~15-20 minutes

---

## 🎯 Collection Commands

### **Session 3.1: Green Medium (เขียวขนาดกลาง)**

```bash
python3 collect_dataset.py \
  --exposure 24 \
  --gain 2 \
  --output pepper_dataset/session3_green_medium
```

**เตรียม:**
- เตรียมพริกเขียวขนาดกลาง 1 ผล
- วางกลางพื้นผ้าดำ
- เริ่มที่มุม 0°

**Steps:**
1. กด SPACE = ถ่ายภาพ
2. หมุนพริก ~30° (ประมาณ 1/12 รอบ)
3. ทำซ้ำ 12 ครั้ง (12 มุม)
4. กด Q = เสร็จสิ้น

---

### **Session 3.2: Green Small (เขียวขนาดเล็ก)**

```bash
python3 collect_dataset.py \
  --exposure 24 \
  --gain 2 \
  --output pepper_dataset/session3_green_small
```

**เตรียม:**
- เตรียมพริกเขียวขนาดเล็ก 1 ผล
- วางกลางพื้นผ้าดำ
- เริ่มที่มุม 0°

**Steps:** (เหมือน Session 3.1)

---

### **Session 3.3: Green Tiny (เขียวขนาดเล็กมาก)**

```bash
python3 collect_dataset.py \
  --exposure 24 \
  --gain 2 \
  --output pepper_dataset/session3_green_tiny
```

**เตรียม:**
- เตรียมพริกเขียวขนาดเล็กมาก 1 ผล (mini pepper)
- วางกลางพื้นผ้าดำ
- เริ่มที่มุม 0°

**Steps:** (เหมือน Session 3.1)

---

### **Session 3.4: Green Rotten (เขียวเน่า)**

```bash
python3 collect_dataset.py \
  --exposure 24 \
  --gain 2 \
  --output pepper_dataset/session3_green_rotten
```

**เตรียม:**
- เตรียมพริกเขียวเน่า 1 ผล
- วางกลางพื้นผ้าดำ
- เริ่มที่มุม 0°

**Steps:** (เหมือน Session 3.1)

---

## 📊 Expected Output

```
pepper_dataset/
├── session3_green_medium/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│       └── collection_log.yaml
│
├── session3_green_small/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│       └── collection_log.yaml
│
├── session3_green_tiny/
│   ├── raw/left/
│   │   └── pepper_0000-0011.jpg  (12 images)
│   └── metadata/
│       └── collection_log.yaml
│
└── session3_green_rotten/
    ├── raw/left/
    │   └── pepper_0000-0011.jpg  (12 images)
    └── metadata/
        └── collection_log.yaml
```

**Total:** 48 images

---

## ⏱️ Time Estimate

| Session | Peppers | Images | Time |
|---------|---------|--------|------|
| 3.1: Green Medium | 1 | 12 | ~4 min |
| 3.2: Green Small | 1 | 12 | ~4 min |
| 3.3: Green Tiny | 1 | 12 | ~4 min |
| 3.4: Green Rotten | 1 | 12 | ~4 min |
| **Total** | **4** | **48** | **~16 min** |

---

## ✅ Checklist

### Before Starting:
- [ ] เตรียมพริกเขียว 4 ผล (medium, small, tiny, rotten)
- [ ] ตรวจสอบกล้อง (exposure=24ms, gain=2)
- [ ] ผ้าดำรองพื้นพร้อม
- [ ] แสง LED 3 ดวงเปิดอยู่

### During Collection:
- [ ] วางพริกกลางเฟรม
- [ ] หมุนพริกประมาณ 30° ทุกครั้ง
- [ ] ถ่าย 12 มุมต่อพริก
- [ ] ตรวจสอบภาพว่าชัดและครอบคลุม

### After Each Session:
- [ ] ตรวจสอบจำนวนภาพ (12 images)
- [ ] ตรวจสอบคุณภาพภาพ (ชัด, แสงดี)
- [ ] เปลี่ยนพริกผลใหม่

---

## 🎯 Quality Check

**หลังเก็บเสร็จ ให้ตรวจสอบ:**

```bash
# นับจำนวนภาพทั้งหมด Session 3
find pepper_dataset/session3_* -name "*.jpg" | wc -l
# Expected: 48

# นับแต่ละ session
for dir in pepper_dataset/session3_*; do
  echo "$(basename $dir): $(find $dir -name '*.jpg' | wc -l) images"
done
# Expected: 12 images each
```

---

## 📈 Dataset Summary (After Session 3)

```
Total Dataset:
- Session 1: 96 images (Red large)
- Session 2: 48 images (Red defects)
- Session 3: 48 images (Green varieties) ← NEW!

Grand Total: 192 images
Peppers: 16 peppers × 12 angles
```

---

## 💡 Tips

1. **หมุนพริกสม่ำเสมอ** - ใช้นาฬิกาช่วยประมาณมุม 30°
2. **ตรวจภาพทุกครั้ง** - ดูหน้าจอว่าพริกอยู่กลางเฟรม
3. **แสงคงที่** - ใช้ค่าเดิม exposure=24ms, gain=2
4. **ทำทีละ session** - อย่าเร่งรีบ
5. **พักระหว่าง session** - พัก 1-2 นาทีระหว่าง session

---

## 🚀 Quick Start (Copy-Paste All)

```bash
# Session 3.1: Green Medium
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_medium

# Session 3.2: Green Small
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_small

# Session 3.3: Green Tiny
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_tiny

# Session 3.4: Green Rotten
python3 collect_dataset.py --exposure 24 --gain 2 --output pepper_dataset/session3_green_rotten

# Verify all images
find pepper_dataset/session3_* -name "*.jpg" | wc -l
```

---

**Good luck! 🌶️✨**
