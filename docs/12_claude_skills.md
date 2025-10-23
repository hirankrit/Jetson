# Claude Code Skills

**Last Updated**: 2025-10-23

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
        └── python-tools.md      # เครื่องมือ lint, type check, logging
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
| 2025-10-23 | `python-tools.md` | สร้าง skill ใหม่ - linting, type check, logging |
| 2025-10-23 | `ros2-review.md` | ขยายความ Python Code Quality |
| 2025-10-23 | `weekly-report.md` | สร้าง skill ใหม่ |
| 2025-10-23 | `thai-commit.md` | สร้าง skill ใหม่ |

---

**Happy Coding with Claude Skills! 🚀**
