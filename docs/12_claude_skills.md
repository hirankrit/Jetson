# Claude Code Skills

**Last Updated**: 2025-11-06

Claude Skills คือชุดคำสั่งเล็กๆ (lightweight instructions) ที่สอน Claude Code CLI ให้ทำงานเฉพาะทางได้สม่ำเสมอและมีมาตรฐานเดียวกัน

---

## 📍 ที่เก็บ Skills

Skills ทั้งหมดเก็บอยู่ที่: `.claude/skills/`

```
/home/jay/Project/
└── .claude/
    └── skills/
        ├── thai-commit.md       # สร้าง commit message ภาษาไทย
        ├── weekly-report.md     # สร้างรายงานประจำสัปดาห์
        ├── ros2-review.md       # รีวิวโค้ด ROS2 + Python
        ├── python-tools.md      # เครื่องมือ lint, type check, logging
        ├── step-by-step.md      # การสื่อสารแบบทีละขั้นตอน
        └── update-claude-md.md  # จัดการไฟล์ claude.md
```

---

## 🎯 Skills ที่มีในโปรเจคนี้

### 1. Thai Commit Message (`thai-commit.md`)

**วัตถุประสงค์**: สร้าง commit message เป็นภาษาไทยตามรูปแบบมาตรฐาน

**รูปแบบ**:
```
<ประเภท>: <สรุปสั้นๆ>

<รายละเอียดเพิ่มเติม>
```

**ประเภท Commit**:
- **เพิ่ม** - เพิ่มฟีเจอร์ใหม่
- **แก้ไข** - แก้ไข code ที่มีอยู่
- **แก้บั๊ก** - แก้ไขข้อผิดพลาด
- **เอกสาร** - เปลี่ยนแปลง documentation
- **รีแฟค** - refactor code
- **ทดสอบ** - เพิ่มหรือแก้ไข tests
- **ปรับปรุง** - ปรับปรุงประสิทธิภาพ

**ตัวอย่างการใช้**:
```bash
# ในโหมด interactive
claude
> /skill thai-commit
> ช่วย commit การเปลี่ยนแปลงหน่อย

# หรือแบบ one-liner
claude "ช่วย commit ไฟล์ที่แก้ไขหน่อย"
```

**ตัวอย่าง Output**:
```
เพิ่ม: ระบบ stereo calibration สำหรับกล้อง IMX219

- เพิ่มฟังก์ชัน calibrate_stereo_camera()
- รองรับ checkerboard pattern detection
- บันทึกผลลัพธ์เป็น YAML file
```

---

### 2. Weekly Report (`weekly-report.md`)

**วัตถุประสงค์**: สร้างรายงานประจำสัปดาห์ตามรูปแบบมาตรฐาน

**โครงสร้างรายงาน**:
- 📋 เป้าหมายสัปดาห์นี้
- ✅ ผลงานที่ทำสำเร็จ
- 📊 ผลการทดสอบ/วัดผล
- 🐛 ปัญหาที่พบ
- 📝 บันทึกสำคัญ
- 🎯 แผนสัปดาห์หน้า

**ตัวอย่างการใช้**:
```bash
claude --skill weekly-report "สร้าง report สัปดาห์ที่ 1 เกี่ยวกับ stereo calibration"
```

**เหมาะสำหรับ**:
- รายงานความก้าวหน้าประจำสัปดาห์
- สรุปผลการทดลอง
- จดบันทึกปัญหาและวิธีแก้

---

### 3. ROS2 Code Review (`ros2-review.md`)

**วัตถุประสงค์**: รีวิวโค้ด ROS2 และ Python ตามมาตรฐาน best practices

**หัวข้อที่ตรวจสอบ**:

#### 1. ROS2 Best Practices
- Node naming convention
- Topic/Service naming
- QoS profiles
- Shutdown handling

#### 2. Python Code Quality (รายละเอียด)
- **PEP 8 Style Guide**: indentation, line length, naming, imports, whitespace
- **Type Hints**: function parameters, return types, Optional, List, Dict
- **Docstrings**: Google Style format พร้อม Args, Returns, Raises, Example
- **Error Handling**: try-except เฉพาะเจาะจง, ROS2 logger, finally block
- **Code Structure**: function length, single responsibility, no magic numbers
- **ROS2 Python Specific**: node init, parameters, callbacks, cleanup

#### 3. C++ Code Quality
- Smart pointers
- RAII principles
- Const correctness

#### 4. Performance
- ไม่มี busy waiting loops
- Timer/Callback frequency
- Message size

#### 5. Safety
- Null pointer checks
- Array bounds checking
- Thread safety

**ตัวอย่างการใช้**:
```bash
# รีวิวไฟล์เดียว
claude --skill ros2-review "ช่วยรีวิว camera_node.py หน่อย"

# รีวิวทั้งแพ็กเกจ
claude --skill ros2-review "ช่วยรีวิวโค้ดใน src/vision_pkg/ หน่อย"
```

**รูปแบบผลลัพธ์**:
- ✅ **ดีแล้ว**: สิ่งที่ทำได้ดี
- ⚠️ **ควรปรับปรุง**: ข้อเสนอแนะ
- ❌ **ต้องแก้ไข**: ปัญหาที่ต้องแก้ก่อน merge

---

### 4. Python Development Tools (`python-tools.md`)

**วัตถุประสงค์**: คำสั่งและวิธีใช้เครื่องมือสำหรับ linting, type checking, และ logging

**เนื้อหาหลัก**:

#### 1. Code Linting
- **Flake8** - PEP 8 checker พร้อม configuration
- **Pylint** - More strict linting
- **Black** - Auto formatter

#### 2. Type Checking
- **MyPy** - Static type checker
- **Pyright** - Fast alternative

#### 3. Logging Best Practices
- **ROS2 Logger** - สำหรับ ROS2 nodes
  - Log levels (debug, info, warn, error, fatal)
  - ตั้งค่า log level ตอนรัน
  - บันทึก logs ลงไฟล์
- **Python Logging** - สำหรับ non-ROS2 code
  - Basic setup
  - Advanced setup (rotating file handlers)
  - Structured logging (JSON format)

#### 4. Combined Workflow
- Pre-commit hooks setup
- Single command check script
- All-in-one ตรวจสอบครั้งเดียว

**ตัวอย่างการใช้**:
```bash
# ขอคำสั่ง lint
claude --skill python-tools "ช่วยแนะนำวิธีใช้ flake8 หน่อย"

# ขอตั้งค่า logging
claude --skill python-tools "ช่วยสร้าง logging setup สำหรับโปรเจคหน่อย"

# ขอตั้งค่า pre-commit hooks
claude --skill python-tools "ช่วย setup pre-commit hooks หน่อย"
```

**Quick Reference ที่มีในไฟล์**:
```bash
# ติดตั้ง tools ทั้งหมด
pip install black flake8 mypy pylint pre-commit python-json-logger

# ตรวจสอบโค้ดครั้งเดียว
black src/ && flake8 src/ && mypy src/
```

---

### 5. Step-by-Step Communication (`step-by-step.md`)

**วัตถุประสงค์**: สื่อสารและให้คำแนะนำแบบทีละขั้นตอนเพื่อไม่ให้ผู้ใช้สับสนหรือพลาดขั้นตอน

**หลักการสำคัญ**:

#### ✅ ควรทำ:
1. **ให้ทีละขั้นตอน** - บอกแค่ขั้นตอนถัดไปที่ต้องทำ
2. **ทำเลยถ้าทำได้** - งานที่ชัดเจน (แก้ code, รันทดสอบ, เช็คไฟล์) ทำเลยไม่ต้องถาม
3. **รอผู้ใช้เฉพาะงานที่ต้องมนุษย์** - เช่น ถ่ายภาพ, วัดระยะ, ตัดสินใจเลือก
4. **กระชับ ชัดเจน** - ใช้ภาษาสั้นๆ เข้าใจง่าย
5. **แก้ไข code โดยตรง** - ใช้ Edit tool แก้เลย ไม่ต้องใช้ nano หรือถามผู้ใช้
6. **รันคำสั่งที่ปลอดภัย** - Bash, Read, Grep, Edit ทำได้เลย

#### ❌ หลีกเลี่ยง:
1. **ห้ามให้หลายขั้นตอนพร้อมกัน** - ผู้ใช้จะดูแค่อันสุดท้ายและพลาดขั้นตอนก่อนหน้า
2. **ห้ามอธิบายยาว** - ไม่ต้องให้ background, tips, troubleshooting ถ้าไม่ถาม
3. **ห้ามให้ทางเลือกเยอะ** - เลือกทางที่ดีที่สุดให้เลย
4. **ห้ามคาดการณ์ล่วงหน้า** - ไม่ต้องบอกว่าขั้นต่อไปจะทำอะไร

**รูปแบบการตอบมาตรฐาน**:
```markdown
## ขั้นตอนถัดไป

[คำอธิบายสั้นๆ 1 บรรทัด]

[คำสั่ง/วิธีทำ]

[บอกว่าต้องรอผลอะไร]
```

**งานที่ทำได้เลย (ไม่ต้องถาม)**:
- ✅ แก้ไข code (ใช้ Edit tool)
- ✅ รันคำสั่งอ่าน (Read, Grep, Glob, ls)
- ✅ รันโปรแกรมทดสอบ (python3 test_*.py)
- ✅ เช็คผลลัพธ์ (ดู output, compare ค่า)
- ✅ สร้าง/ลบไฟล์ชั่วคราว
- ✅ อัพเดตเอกสาร

**งานที่ต้องรอผู้ใช้**:
- ⏸️ ถ่ายภาพ (ต้องจัดมุมกล้อง)
- ⏸️ วัดระยะ (ต้องใช้ไม้บรรทัด)
- ⏸️ ปรับ hardware (focus กล้อง, วางวัตถุ)
- ⏸️ ตัดสินใจสำคัญ (เลือก approach, ยืนยันลบข้อมูล)

**ตัวอย่างการใช้**:
```bash
# กรณีที่ผู้ใช้บอก "ลอง spacing 18mm"
# Claude จะ: [แก้ไข code] → [รันทดสอบ] → [แสดงผล] → [ถามต่อ]
```

**เมื่อใช้ Skill นี้**: ใช้ทุกครั้งที่ให้คำแนะนำทีละขั้นตอน, งานหลายขั้นตอนต่อเนื่อง, หรือผู้ใช้บอกว่าสับสน

---

### 6. Claude.md Management (`update-claude-md.md`)

**วัตถุประสงค์**: จัดการและอัพเดต claude.md อย่างมีประสิทธิภาพ

**หลักการ**:
1. **claude.md = Overview + Quick Reference**
   - เก็บข้อมูลสำคัญ, quick start, roadmap
   - ไม่เก็บ detailed progress (ให้ลิงก์ไปไฟล์แยก)
   - เป้าหมาย: 1,500-2,000 บรรทัด MAX

2. **docs/weekX/ = Detailed Progress**
   - เก็บ progress แต่ละสัปดาห์แยกกันไป
   - เขียนละเอียดได้ไม่จำกัด

3. **Archive เก่า → ไฟล์แยก**
   - Progress เก่า ≥ 1 สัปดาห์ → ย้ายไป docs/
   - claude.md เก็บแค่ลิงก์

**โครงสร้างไฟล์แนะนำ**:
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
    │   ├── dataset_collection.md
    │   └── grid_layout_experiment.md
    └── week3/
        └── (future)
```

**Template: Week Progress Section**:
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

**Next Steps:**
- Complete Session 2-3 (defect types + green peppers)
- Annotation with Roboflow
```

**เมื่อไหร่ควร Archive**:
- ✅ เมื่อจบสัปดาห์ (Week X เสร็จแล้ว)
- ✅ เมื่อ claude.md > 1,800 บรรทัด
- ✅ เมื่อเริ่ม Week ใหม่

**Benefits**:
- claude.md กระชับ - อ่านง่าย, โหลดเร็ว
- Progress มีโครงสร้าง - แยกตาม week
- ค้นหาง่าย - รู้ว่าข้อมูลอยู่ไฟล์ไหน
- Scale ได้ - เพิ่ม week ใหม่ไม่กระทบเก่า

---

## 🚀 วิธีใช้งาน Skills

### วิธีที่ 1: เรียกใช้ Skill โดยตรง
```bash
claude --skill <skill-name> "<คำสั่ง>"

# ตัวอย่าง
claude --skill thai-commit "ช่วย commit หน่อย"
claude --skill ros2-review "รีวิว camera_node.py"
```

### วิธีที่ 2: Interactive Mode
```bash
claude
> /skill thai-commit
> ช่วย commit การเปลี่ยนแปลงหน่อย
```

### วิธีที่ 3: ให้ Claude เลือกอัตโนมัติ
```bash
# Claude จะดูจากบริบทและเลือก skill ที่เหมาะสมเอง
claude "ช่วยรีวิวโค้ดนี้หน่อย" camera_node.py
```

---

## 📚 Workflow แนะนำ

### สำหรับการพัฒนา ROS2 Node:

1. **เขียนโค้ด** → ใช้ editor ตามปกติ

2. **ตรวจสอบอัตโนมัติ** → ใช้ `python-tools` skill
   ```bash
   # รัน lint และ type check
   claude --skill python-tools "ช่วยตรวจสอบโค้ดด้วย flake8 และ mypy"
   ```

3. **แก้ไขตาม warnings/errors**

4. **Manual Review** → ใช้ `ros2-review` skill
   ```bash
   claude --skill ros2-review "ช่วยรีวิวโค้ดหน่อย"
   ```

5. **แก้ไขตามข้อเสนอแนะ**

6. **Commit** → ใช้ `thai-commit` skill
   ```bash
   claude --skill thai-commit "ช่วย commit หน่อย"
   ```

7. **Push ขึ้น GitHub**
   ```bash
   git push
   ```

### สำหรับการสรุปงานประจำสัปดาห์:

```bash
# สร้าง weekly report
claude --skill weekly-report "สร้าง report สัปดาห์ที่ [เลข] เกี่ยวกับ [หัวข้อ]"

# บันทึกลง docs/ หรือแชร์กับทีม
```

---

## ✏️ การแก้ไขและสร้าง Skills ใหม่

### แก้ไข Skill ที่มีอยู่:
```bash
# เปิดไฟล์แก้ไขได้เลย
nano .claude/skills/thai-commit.md
# หรือ
code .claude/skills/thai-commit.md
```

### สร้าง Skill ใหม่:
1. สร้างไฟล์ใหม่ใน `.claude/skills/`
2. ตั้งชื่อเป็น `<skill-name>.md`
3. เขียนคำแนะนำเป็น Markdown
4. ทดสอบใช้งาน

**ตัวอย่าง Template**:
```markdown
# Skill Name

เมื่อผู้ใช้ขอให้... ให้ทำตามนี้:

## หัวข้อที่ 1
- รายละเอียด
- ตัวอย่าง

## หัวข้อที่ 2
- รายละเอียด
- ตัวอย่าง

## หลักการ:
- หลักการสำคัญ
```

---

## 💡 Tips & Best Practices

1. **ตั้งชื่อ Skill ให้สื่อความหมาย**
   - ใช้ lowercase + hyphen (เช่น `thai-commit`, `ros2-review`)

2. **เขียนคำแนะนำให้ชัดเจน**
   - มีตัวอย่างประกอบ
   - ระบุ do's and don'ts

3. **แบ่ง Skills ตามหน้าที่**
   - 1 Skill = 1 งานเฉพาะทาง
   - ไม่ควรรวมหลายอย่างในไฟล์เดียว

4. **Update Skills เป็นประจำ**
   - เมื่อเจอ pattern ใหม่ที่ดี
   - เมื่อทีมตกลงมาตรฐานใหม่

5. **ใช้ร่วมกับ Git**
   - Commit Skills ขึ้น GitHub
   - ทีมจะได้ใช้งานร่วมกัน

---

## 🔗 Related Documents

- [Git Push/Pull Guide](../claude.md#-git-pushpull-guide) - วิธีใช้ Git พื้นฐาน
- [Setup Guide](10_setup_guide.md) - ติดตั้ง development environment
- [Development Roadmap](11_vision_first_roadmap.md) - แผนการพัฒนาโปรเจค

---

## 📝 Skills Changelog

| Date | Skill | Change |
|------|-------|--------|
| 2025-11-06 | `update-claude-md.md` | สร้าง skill ใหม่ - จัดการ claude.md แบบมีโครงสร้าง |
| 2025-11-06 | `step-by-step.md` | สร้าง skill ใหม่ - การสื่อสารแบบทีละขั้นตอน |
| 2025-10-23 | `python-tools.md` | สร้าง skill ใหม่ - linting, type check, logging |
| 2025-10-23 | `ros2-review.md` | ขยายความ Python Code Quality |
| 2025-10-23 | `weekly-report.md` | สร้าง skill ใหม่ |
| 2025-10-23 | `thai-commit.md` | สร้าง skill ใหม่ |

---

**Happy Coding with Claude Skills! 🚀**
