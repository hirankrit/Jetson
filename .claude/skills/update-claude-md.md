# Update claude.md Management

วิธีการอัพเดตและจัดการ claude.md อย่างมีประสิทธิภาพ

## หลักการ

1. **claude.md = Overview + Quick Reference**
   - เก็บข้อมูลสำคัญ, quick start, roadmap
   - ไม่เก็บ detailed progress (ให้ลิงก์ไปไฟล์แยก)

2. **docs/weekX/ = Detailed Progress**
   - เก็บ progress แต่ละสัปดาห์แยกกันไป
   - เขียนละเอียดได้ไม่จำกัด

3. **Archive เก่า → ไฟล์แยก**
   - Progress เก่า ≥ 1 สัปดาห์ → ย้ายไป docs/
   - claude.md เก็บแค่ลิงก์

## โครงสร้างไฟล์

```
Project/
├── claude.md                          (1,500-2,000 บรรทัด MAX)
│   ├── Quick Start
│   ├── Phase Overview
│   ├── Current Week Progress (summary)
│   └── Links to detailed docs
│
└── docs/
    ├── week1/
    │   ├── stereo_calibration.md
    │   └── progress_summary.md
    ├── week2/
    │   ├── dataset_collection.md      ← Week 2 progress
    │   ├── grid_layout_experiment.md
    │   └── manual_capture_results.md
    └── week3/
        └── (future)
```

## วิธีอัพเดต claude.md

### ❌ ไม่ควรทำ:
```bash
# Append ท้ายไฟล์เรื่อยๆ (ทำให้ไฟล์ใหญ่เกิน)
cat >> claude.md << 'EOF'
...ข้อมูลเยอะๆ...
EOF
```

### ✅ ควรทำ:
```bash
# 1. เขียน progress ไป docs/week2/
echo "..." > docs/week2/dataset_collection.md

# 2. อัพเดตเฉพาะ section ใน claude.md (ใช้ Edit tool)
# แก้ไข "## Week 2 Progress" section
# เปลี่ยนจาก detailed → summary + link
```

## Template: Week Progress Section

```markdown
## 📅 Week 2: Dataset Collection (Oct 28-Nov 3, 2025)

**Status:** 🟡 In Progress (50% complete)

**Key Achievements:**
- ✅ Grid Layout Auto Crop - Tested (failed, but learned)
- ✅ Manual Capture - Success (96 images, 9 minutes)
- 🟡 Defect Types - In progress (Session 2.1/2.4)

**Detailed Progress:**
- [Dataset Collection Report](docs/week2/dataset_collection.md)
- [Grid Layout Experiments](docs/week2/grid_layout_experiment.md)
- [Manual Capture Results](docs/week2/manual_capture_results.md)

**Next Steps:**
- Complete Session 2-3 (defect types + green peppers)
- Annotation with Roboflow
- Start YOLO training (Week 3)
```

## วิธีสร้างไฟล์ Progress ใหม่

```bash
# 1. สร้างโฟลเดอร์
mkdir -p docs/week2

# 2. ดึง content จาก claude.md (บรรทัด X-Y)
sed -n 'X,Yp' claude.md > docs/week2/dataset_collection.md

# 3. แก้ไข claude.md ให้มีแค่ summary + link
# (ใช้ Edit tool)

# 4. Commit
git add claude.md docs/week2/
git commit -m "docs: refactor Week 2 progress to separate file"
```

## เมื่อไหร่ควร Archive

- ✅ เมื่อจบสัปดาห์ (Week X เสร็จแล้ว)
- ✅ เมื่อ claude.md > 1,800 บรรทัด
- ✅ เมื่อเริ่ม Week ใหม่

## Benefits

1. **claude.md กระชับ** - อ่านง่าย, โหลดเร็ว
2. **Progress มีโครงสร้าง** - แยกตาม week
3. **ค้นหาง่าย** - รู้ว่าข้อมูลอยู่ไฟล์ไหน
4. **Scale ได้** - เพิ่ม week ใหม่ไม่กระทบเก่า

## Example Usage

```bash
# สัปดาห์ปัจจุบัน: อัพเดต docs/week2/ โดยตรง
echo "Session 1 complete!" >> docs/week2/dataset_collection.md

# claude.md: อัพเดตแค่ summary
# "Status: 50% → 60%"

# สัปดาห์ถัดไป: เริ่ม week3
mkdir docs/week3
# claude.md: เปลี่ยน "## Current Week" → link to week3
```

---

**Updated:** Oct 29, 2025
**Author:** Jay + Claude
