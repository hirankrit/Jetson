# Python Development Tools Skill

เมื่อผู้ใช้ขอให้ตรวจสอบโค้ด Python ด้วย automated tools หรือต้องการ setup logging ให้ใช้คำสั่งและแนวทางเหล่านี้:

---

## 1. Code Linting

### 1.1 Flake8 (PEP 8 Checker)
**ติดตั้ง:**
```bash
pip install flake8
```

**คำสั่งพื้นฐาน:**
```bash
# ตรวจสอบไฟล์เดียว
flake8 my_node.py

# ตรวจสอบทั้งโฟลเดอร์
flake8 src/

# ตรวจสอบพร้อมแสดงสถิติ
flake8 --statistics --count src/

# กำหนด max line length (default 79)
flake8 --max-line-length=100 src/

# Ignore specific errors
flake8 --ignore=E501,W503 src/
```

**Configuration file (.flake8):**
```ini
[flake8]
max-line-length = 100
ignore = E501, W503
exclude =
    .git,
    __pycache__,
    build,
    install,
    log
per-file-ignores =
    __init__.py:F401
```

### 1.2 Pylint (More Strict)
**ติดตั้ง:**
```bash
pip install pylint
```

**คำสั่งพื้นฐาน:**
```bash
# ตรวจสอบไฟล์
pylint my_node.py

# แสดงเฉพาะ errors
pylint --errors-only my_node.py

# กำหนด score ขั้นต่ำ
pylint --fail-under=8.0 my_node.py

# Export เป็น JSON
pylint --output-format=json my_node.py > report.json
```

**Configuration file (.pylintrc):**
```ini
[MASTER]
ignore=CVS,.git,__pycache__

[MESSAGES CONTROL]
disable=C0111,  # missing-docstring
        C0103,  # invalid-name
        R0913   # too-many-arguments

[FORMAT]
max-line-length=100
indent-string='    '

[BASIC]
good-names=i,j,k,x,y,z,_
```

### 1.3 Black (Auto Formatter)
**ติดตั้ง:**
```bash
pip install black
```

**คำสั่งพื้นฐาน:**
```bash
# ดูว่าจะแก้อะไรบ้าง (ไม่แก้จริง)
black --check my_node.py

# แก้ไขจริง
black my_node.py

# แก้ทั้งโฟลเดอร์
black src/

# กำหนด line length
black --line-length=100 src/
```

---

## 2. Type Checking

### 2.1 MyPy (Static Type Checker)
**ติดตั้ง:**
```bash
pip install mypy
```

**คำสั่งพื้นฐาน:**
```bash
# ตรวจสอบไฟล์เดียว
mypy my_node.py

# ตรวจสอบทั้งโฟลเดอร์
mypy src/

# Strict mode (แนะนำ)
mypy --strict my_node.py

# Ignore missing imports
mypy --ignore-missing-imports src/

# แสดง error codes
mypy --show-error-codes src/

# Export HTML report
mypy --html-report ./mypy-report src/
```

**Configuration file (mypy.ini):**
```ini
[mypy]
python_version = 3.10
warn_return_any = True
warn_unused_configs = True
disallow_untyped_defs = True
ignore_missing_imports = True

[mypy-setup]
ignore_errors = True

[mypy-test_*]
ignore_errors = True
```

### 2.2 Pyright (Fast Alternative)
**ติดตั้ง:**
```bash
npm install -g pyright
# หรือ
pip install pyright
```

**คำสั่งพื้นฐาน:**
```bash
# ตรวจสอบทั้งโปรเจค
pyright

# ตรวจสอบไฟล์เฉพาะ
pyright src/my_node.py

# Verbose output
pyright --verbose
```

---

## 3. Logging Best Practices

### 3.1 ROS2 Logger (สำหรับ ROS2 Nodes)
**ในโค้ด:**
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

        # Log levels
        self.get_logger().debug('Debug information')
        self.get_logger().info('Information message')
        self.get_logger().warn('Warning message')
        self.get_logger().error('Error occurred')
        self.get_logger().fatal('Fatal error')

    def process_data(self, data):
        """Process data with logging."""
        self.get_logger().info(f'Processing data: {data}')

        try:
            result = self.compute(data)
            self.get_logger().info(f'Result: {result}')
            return result
        except Exception as e:
            self.get_logger().error(f'Failed to process: {e}')
            return None
```

**ตั้งค่า log level ตอนรัน:**
```bash
# Info level (default)
ros2 run my_package my_node

# Debug level
ros2 run my_package my_node --ros-args --log-level debug

# Specific node debug
ros2 run my_package my_node --ros-args --log-level my_node:=debug
```

**บันทึก logs ลงไฟล์:**
```bash
# ROS2 จะบันทึกอัตโนมัติที่ ~/.ros/log/

# ดู log ล่าสุด
cat ~/.ros/log/latest/my_node/stdout.log

# หรือใช้ ros2 command
ros2 run my_package my_node 2>&1 | tee my_node.log
```

### 3.2 Python Logging (สำหรับ non-ROS2 code)
**Basic Setup:**
```python
import logging

# ตั้งค่าพื้นฐาน
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('app.log'),
        logging.StreamHandler()  # แสดงบน console ด้วย
    ]
)

logger = logging.getLogger(__name__)

# ใช้งาน
logger.debug('Debug message')
logger.info('Info message')
logger.warning('Warning message')
logger.error('Error message')
logger.critical('Critical message')
```

**Advanced Setup (แยกไฟล์ตาม level):**
```python
import logging
from logging.handlers import RotatingFileHandler

def setup_logger(name: str) -> logging.Logger:
    """Setup logger with rotating file handlers."""
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    # Formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
    )

    # File handler - all logs (max 10MB, keep 5 backups)
    fh_all = RotatingFileHandler(
        'logs/all.log',
        maxBytes=10*1024*1024,
        backupCount=5
    )
    fh_all.setLevel(logging.DEBUG)
    fh_all.setFormatter(formatter)

    # File handler - errors only
    fh_error = RotatingFileHandler(
        'logs/errors.log',
        maxBytes=10*1024*1024,
        backupCount=5
    )
    fh_error.setLevel(logging.ERROR)
    fh_error.setFormatter(formatter)

    # Console handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)

    # Add handlers
    logger.addHandler(fh_all)
    logger.addHandler(fh_error)
    logger.addHandler(ch)

    return logger

# ใช้งาน
logger = setup_logger('my_app')
logger.info('Application started')
```

### 3.3 Structured Logging (JSON Format)
**ติดตั้ง:**
```bash
pip install python-json-logger
```

**ใช้งาน:**
```python
import logging
from pythonjsonlogger import jsonlogger

logger = logging.getLogger()

logHandler = logging.FileHandler('app.json')
formatter = jsonlogger.JsonFormatter()
logHandler.setFormatter(formatter)
logger.addHandler(logHandler)

logger.info('User logged in', extra={'user_id': 123, 'ip': '192.168.1.1'})
# Output: {"message": "User logged in", "user_id": 123, "ip": "192.168.1.1", "timestamp": "..."}
```

---

## 4. Combined Workflow

### 4.1 Pre-commit Hook Setup
**ติดตั้ง:**
```bash
pip install pre-commit
```

**Configuration file (.pre-commit-config.yaml):**
```yaml
repos:
  - repo: https://github.com/psf/black
    rev: 23.3.0
    hooks:
      - id: black
        language_version: python3.10

  - repo: https://github.com/pycqa/flake8
    rev: 6.0.0
    hooks:
      - id: flake8
        args: [--max-line-length=100]

  - repo: https://github.com/pre-commit/mirrors-mypy
    rev: v1.3.0
    hooks:
      - id: mypy
        args: [--ignore-missing-imports]
```

**ติดตั้ง hooks:**
```bash
pre-commit install
```

### 4.2 Single Command Check
**สร้างไฟล์ check.sh:**
```bash
#!/bin/bash

echo "🔍 Running Black formatter..."
black --check src/

echo "🔍 Running Flake8..."
flake8 src/

echo "🔍 Running MyPy..."
mypy src/

echo "✅ All checks passed!"
```

**ใช้งาน:**
```bash
chmod +x check.sh
./check.sh
```

---

## 5. Quick Reference

### ตรวจสอบโค้ดครั้งเดียว (All-in-one):
```bash
# Format + Lint + Type Check
black src/ && flake8 src/ && mypy src/
```

### ติดตั้ง tools ทั้งหมดพร้อมกัน:
```bash
pip install black flake8 mypy pylint pre-commit python-json-logger
```

### แนะนำสำหรับโปรเจค ROS2:
1. ใช้ **Black** สำหรับ auto-format
2. ใช้ **Flake8** สำหรับ quick lint check
3. ใช้ **MyPy** สำหรับ type checking
4. ใช้ **ROS2 logger** สำหรับ logging
5. ตั้งค่า **pre-commit hooks** เพื่อตรวจสอบอัตโนมัติก่อน commit

---

## หลักการใช้งาน:
- รัน lint และ type check **ก่อน commit** ทุกครั้ง
- ใช้ **Black** เพื่อให้ code style สม่ำเสมอในทีม
- เปิด **Debug logging** เฉพาะตอน development
- บันทึก **Error logs** แยกไฟล์เพื่อง่ายต่อการดู
- ใช้ **Rotating logs** เพื่อไม่ให้ไฟล์ใหญ่เกินไป
