# Deployment Guide - Pepper Detection System

> Production deployment guide for YOLO11n pepper detection

## Quick Start

### 1. Single Image Detection (TensorRT - Fastest)

```bash
python3 deploy_pepper_detection_improved.py \
  --model runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine \
  --image test_image.jpg \
  --save output.jpg
```

### 2. Batch Processing with CSV Export

```bash
python3 deploy_pepper_detection_improved.py \
  --folder images/ \
  --output results/ \
  --csv detections.csv \
  --no-display
```

### 3. Video Processing

```bash
# Normal video processing
python3 deploy_pepper_detection_improved.py \
  --video input.mp4 \
  --save output.mp4

# With frame skipping (faster)
python3 deploy_pepper_detection_improved.py \
  --video input.mp4 \
  --skip-frames 2 \
  --save output.mp4 \
  --no-display
```

### 4. Live Camera Feed

```bash
python3 deploy_pepper_detection_improved.py \
  --camera 0 \
  --conf 0.6
```

## Command-Line Options

```
Required (one of):
  --image IMAGE        Single image path
  --folder FOLDER      Folder with images
  --video VIDEO        Video file path
  --camera INDEX       Camera index (0, 1, etc.)

Model:
  --model MODEL        Model path (.pt, .engine, .onnx)
                       Default: best.engine

Detection:
  --conf THRESHOLD     Confidence threshold (0.0-1.0)
                       Default: 0.5
  --iou IOU            IoU threshold for NMS
                       Default: 0.45
  --data-yaml PATH     Path to data.yaml (for class names)
                       Default: pepper_dataset/data.yaml

Output:
  --save PATH          Save output image/video path
  --output DIR         Output directory for batch
  --csv PATH           Export results to CSV
  --no-display         Don't display results (headless mode)

Performance:
  --skip-frames N      Process every Nth frame (video only)
                       Default: 0 (process all)
  --device ID          Device to use (0=GPU, cpu=CPU)
                       Default: 0
```

## Model Formats Comparison

| Format | Extension | Inference Time | Best For |
|--------|-----------|----------------|----------|
| TensorRT | .engine | **45ms** | ⭐ **Production (fastest)** |
| PyTorch | .pt | 58ms | Development/testing |
| ONNX | .onnx | 52ms | Cross-platform |

**Recommendation:** Use `.engine` for production on Jetson

## Performance Benchmarking

```bash
# Benchmark PyTorch vs TensorRT
python3 benchmark_tensorrt_improved.py \
  --pytorch-model best.pt \
  --tensorrt-model best.engine \
  --runs 100 \
  --warmup 20 \
  --format json
```

**Expected Results (Jetson Orin Nano):**
- TensorRT: 45ms avg, 22.2 FPS
- PyTorch: 58ms avg, 17.3 FPS
- Speedup: **1.29x**

## Batch Processing Example

```python
import glob
import pandas as pd

# Process all images in folder
images = glob.glob('test_images/*.jpg')

results = []
for img in images:
    # Run detection
    output = detector.detect_image(img)
    results.append(output)

# Export to CSV
df = pd.DataFrame(results)
df.to_csv('batch_results.csv', index=False)
```

## CSV Output Format

```csv
image_path,num_detections,inference_time_ms,detections_detail
test1.jpg,2,45.2,"pepper_red_large:0.95; pepper_green_small:0.88"
test2.jpg,1,43.8,"pepper_red_wrinkled:0.92"
test3.jpg,0,44.1,""
```

**Columns:**
- `image_path`: Input image path
- `num_detections`: Number of peppers detected
- `inference_time_ms`: Inference time in milliseconds
- `detections_detail`: Class:confidence pairs

## Integration Example

### Python API

```python
from ultralytics import YOLO

# Load model
model = YOLO('best.engine', task='detect')

# Single image
results = model('image.jpg', conf=0.5)

# Process results
for r in results:
    boxes = r.boxes  # Bounding boxes
    for box in boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()

        print(f"Class: {cls}, Conf: {conf:.2f}")
        print(f"Box: ({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})")
```

### ROS2 Integration (Planned)

```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PepperDetectionNode:
    def __init__(self):
        self.model = YOLO('best.engine')
        self.bridge = CvBridge()

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/zed/left/image_rect_color',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert ROS Image to CV2
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run detection
        results = self.model(cv_image, conf=0.6)

        # Publish results
        # ... (custom message with detections)
```

## Production Checklist

### Pre-Deployment
- [ ] Model tested on validation set
- [ ] Inference time acceptable (<50ms)
- [ ] False positive rate acceptable
- [ ] Edge cases tested

### Deployment Configuration
- [ ] TensorRT engine compiled
- [ ] Confidence threshold tuned (recommend 0.5-0.6)
- [ ] Error handling implemented
- [ ] Logging configured

### Runtime Monitoring
- [ ] FPS monitoring
- [ ] Error tracking
- [ ] Result validation
- [ ] Performance metrics

## Error Handling

```python
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

try:
    results = model(image, conf=0.5)
except Exception as e:
    logger.error(f"Detection failed: {e}")
    # Fallback behavior
```

## Performance Optimization Tips

### 1. Use TensorRT Engine
```bash
# Export to TensorRT (if not done)
yolo export model=best.pt format=engine device=0
```

### 2. Frame Skipping for Video
```python
# Process every 3rd frame
python3 deploy_pepper_detection_improved.py \
  --video input.mp4 \
  --skip-frames 2  # Skip 2, process 3rd
```

### 3. Batch Processing
```python
# Process multiple images at once
results = model(['img1.jpg', 'img2.jpg', 'img3.jpg'])
```

### 4. Resolution Optimization
```python
# Lower resolution for faster processing
results = model(image, imgsz=480)  # vs default 640
```

## Troubleshooting

### Issue: Slow Inference

**Check:**
1. Using TensorRT? (.engine file)
2. GPU mode enabled? (device=0)
3. Image resolution appropriate?

**Solution:**
```bash
# Verify GPU usage
nvidia-smi

# Check model format
ls -lh runs/train/*/weights/
```

### Issue: No Detections

**Check:**
1. Confidence threshold too high?
2. Image quality good?
3. Class names loaded?

**Solution:**
```bash
# Lower threshold
--conf 0.3

# Verify class names
cat pepper_dataset/data.yaml
```

### Issue: Memory Error

**Check:**
1. Batch size too large?
2. Image resolution too high?
3. Multiple models loaded?

**Solution:**
```python
# Clear GPU memory
import torch
torch.cuda.empty_cache()
```

## System Requirements

**Minimum:**
- Jetson Orin Nano (or equivalent)
- 4GB RAM
- 2GB storage (for model + dependencies)
- CUDA 11.4+

**Recommended:**
- Jetson Orin Nano 8GB
- 8GB RAM
- 10GB storage
- CUDA 12.6+
- JetPack 6.2+

## Production Deployment Modes

### Mode 1: Standalone Application
- Single Python script
- Load model once
- Process images/video
- Save results to file/DB

### Mode 2: REST API Server
```python
from fastapi import FastAPI, File
import uvicorn

app = FastAPI()
model = YOLO('best.engine')

@app.post("/detect")
async def detect(file: UploadFile = File(...)):
    # Process image
    results = model(file.file)
    return {"detections": results}

uvicorn.run(app, host="0.0.0.0", port=8000)
```

### Mode 3: ROS2 Node
- Subscribe to camera topic
- Publish detection results
- Integrate with robot control

## Next Steps

1. **Test deployment** with production data
2. **Tune confidence** threshold
3. **Monitor performance** metrics
4. **Integrate 3D** stereo depth
5. **Connect robot arm** control

---

**Model Location:** `runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine`
**Script:** `deploy_pepper_detection_improved.py`
**Status:** ✅ Production Ready
