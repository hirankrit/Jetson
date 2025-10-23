# 1. Hardware Architecture

## Physical Layout

### Overview Diagram

```
                    [IMX219 Stereo Camera]
                     (Height: ~50-80cm)
                     📷 CSI Connection
                            │
                            │ Looking down (Top-down view)
                            ▼
        ┌────────────────────────────────────────────────┐
        │   Sorting Output Area (3-4 zones)             │
        │  ┌─────────┬─────────┬─────────┬──────────┐   │
        │  │ Good    │ Good    │ Good    │ Reject   │   │
        │  │ Red     │ Green   │ Yellow  │ Rotten   │   │
        │  │  🟥     │   🟩    │   🟨    │   ❌     │   │
        │  └─────────┴─────────┴─────────┴──────────┘   │
        └────────────────────────────────────────────────┘
                            │
                            │ ~30-40cm gap
                            ▼
        ┌────────────────────────────────────────────────┐
        │         Working Area (Input Zone)              │
        │                                                │
        │  [Left Arm]       [Pile]       [Right Arm]    │
        │      🦾          🌶️🌶️🌶️🌶️           🦾        │
        │   Arduino #1     Input        Arduino #2      │
        │   (Servo-based)  Peppers     (Servo-based)    │
        │                                                │
        └────────────────────────────────────────────────┘
                            │
                            │ USB Serial
                            ▼
                   [Jetson Orin Nano]
                   💻 Main Controller
                   - ROS2 Runtime
                   - Vision Processing
                   - AI Inference
```

---

## Hardware Components

### 1. Jetson Orin Nano Developer Kit

**Specifications**:
- **CPU**: 6-core ARM Cortex-A78AE
- **GPU**: NVIDIA Ampere (1024 CUDA cores)
- **RAM**: 8GB LPDDR5
- **Storage**: microSD card (recommend 64GB+ for ROS2)
- **OS**: Ubuntu 20.04/22.04 (recommend 22.04 for ROS2 Humble)
- **Power**: 5V/4A DC supply

**Connections**:
- CSI-2 camera ports (for stereo camera)
- 4x USB 3.0 ports (for Arduinos)
- Ethernet/WiFi (for development/debugging)

---

### 2. IMX219-83 Stereo Camera

**Key Specifications**:
- **Sensor**: Sony IMX219 (8MP) × 2
- **Resolution**: 3280 × 2464 pixels per camera
- **Frame Rate**: Up to 30 fps @ 1080p
- **Focal Length**: 1.6mm
- **Field of View**: 83° diagonal, 73° horizontal, 50° vertical
- **Aperture**: F/2.4
- **Baseline**: 60mm (distance between two cameras)
- **IMU**: ICM20948 (9-axis: accel + gyro + mag) - optional for stabilization
- **Connection**: 15-pin FFC cable to Jetson CSI ports

**Mounting**:
- Position: Above workspace, looking straight down
- Height: 50-80cm (adjustable based on desired FOV)
- Orientation: Horizontal alignment (cameras side-by-side)
- Stability: Fixed mount (no vibration)

**Why Stereo Camera?**
- ✅ Provides depth information (Z-axis)
- ✅ No need for external sensors
- ✅ Works in various lighting (unlike ToF sensors)
- ✅ Good accuracy for small objects at close range

---

### 3. Robot Arms (2x Arduino-based)

**Reference Design**: Mini Brazo robótico con Arduino
- **DOF**: 4-6 degrees of freedom (typical)
- **Actuators**: Servo motors (MG90S or similar)
- **Controller**: Arduino Mega/Uno (recommend Mega for more PWM pins)
- **Gripper**: 2-finger servo gripper
- **Reach**: ~20-30cm (typical for mini arms)
- **Payload**: ~50-100g (sufficient for peppers)

**Servo Specifications (typical)**:
- 4-6 servos per arm for joints
- 1 servo for gripper
- Torque: 1.8-2.5 kg·cm (depending on model)
- Speed: 0.1s/60° (typical)

**Connection to Jetson**:
- **Left Arm**: USB Serial (/dev/ttyUSB0 or /dev/ttyACM0)
- **Right Arm**: USB Serial (/dev/ttyUSB1 or /dev/ttyACM1)
- **Protocol**: Custom serial commands (9600-115200 baud)

**Power Supply**:
- Separate 5V/2A+ supply for each arm (servos draw high current)
- DO NOT power servos from Arduino 5V pin (insufficient current)

---

## Physical Layout Details

### Workspace Dimensions (Example)

```
                    Camera FOV
                    ↓
    ←─────────── ~40cm ────────────→

    ┌─────────────────────────────┐  ↑
    │   Sorting Zone (Output)     │  │
    │  [10cm × 10cm per bin]      │  │ 15cm
    └─────────────────────────────┘  ↓

    ┌─────────────────────────────┐  ↑
    │   Working Area              │  │
    │                             │  │
    │  ← L →   [Pile]   ← R →    │  │ 30cm
    │  Arm    15cm dia   Arm      │  │
    │                             │  │
    └─────────────────────────────┘  ↓
```

**Coordinates** (Origin at pile center):
- **Pile Area**: Circle, diameter ~15cm, centered at (0, 0, 0)
- **Left Arm Base**: (-15cm, 0, 0)
- **Right Arm Base**: (+15cm, 0, 0)
- **Sorting Bins**: Y = +25cm, X varies
  - Red: (-10cm, 25cm, 0)
  - Green: (0, 25cm, 0)
  - Yellow: (+10cm, 25cm, 0)
  - Reject: (+20cm, 25cm, 0)

---

## Hardware Connections Diagram

```
┌──────────────────────────────────────────────┐
│         Jetson Orin Nano Developer Kit       │
│                                              │
│  CSI-0 ────┐                                 │
│            ├──→ [IMX219 Stereo Camera]      │
│  CSI-1 ────┘    (via 15-pin FFC cable)      │
│                                              │
│  USB 3.0 Port 1 ──→ [Arduino Mega #1]       │
│                      ↓                       │
│                   Left Arm (5-6 servos)     │
│                                              │
│  USB 3.0 Port 2 ──→ [Arduino Mega #2]       │
│                      ↓                       │
│                   Right Arm (5-6 servos)    │
│                                              │
│  Ethernet/WiFi ─────→ Dev Computer          │
│  (for SSH/ROS2 remote access)               │
│                                              │
│  Power: 5V/4A DC Input                       │
└──────────────────────────────────────────────┘

Arduino #1 Power: 5V/2A (separate) ──→ Servo Power Bus
Arduino #2 Power: 5V/2A (separate) ──→ Servo Power Bus

Lighting (Optional): LED panels above workspace
```

---

## Camera Mounting Setup

### Mounting Requirements

1. **Rigid Frame**:
   - Aluminum extrusion frame or 3D printed bracket
   - No wobbling or vibration
   - Adjustable height (50-80cm range)

2. **Alignment**:
   - Camera must be level (parallel to workspace)
   - Two lenses should be horizontal (not tilted)
   - Use spirit level or IMU data for calibration

3. **Lighting**:
   - Consistent, diffuse lighting (avoid harsh shadows)
   - LED panels recommended (5000K-6500K daylight)
   - Position to minimize reflections on peppers

---

## Arm Positioning Strategy

### Human-Like Configuration (Recommended)

```
        Camera (overhead)
              ↓
    ┌─────────────────────┐
    │   Output Bins       │
    └─────────────────────┘
              ↑
        Working Area
              ↓
    Left Arm  ←  Pile  →  Right Arm
       🦾              🦾

    Similar to human hands:
    - Left arm handles left side (X < 0)
    - Right arm handles right side (X > 0)
    - Can reach forward to center pile
    - Place behind in sorting bins
```

**Advantages**:
- Natural workspace division
- Minimized collision risk
- Parallel operation (2× throughput)
- Easy to expand (add more arms)

---

## Power Budget

| Component | Voltage | Current | Power |
|-----------|---------|---------|-------|
| Jetson Orin Nano | 5V | 3-4A | 15-20W |
| Stereo Camera | 5V (via Jetson) | 0.5A | 2.5W |
| Arduino #1 + Servos | 5V | 2-3A | 10-15W |
| Arduino #2 + Servos | 5V | 2-3A | 10-15W |
| **Total** | - | - | **~40-50W** |

**Recommendations**:
- Use separate power supplies for Jetson and each Arduino
- Servo power: External 5V/3A+ supply with capacitor (1000µF)
- Add surge protection (especially for servo spikes)

---

## Bill of Materials (BOM)

### Core Components

| Item | Quantity | Notes |
|------|----------|-------|
| Jetson Orin Nano Dev Kit | 1 | Main controller |
| IMX219-83 Stereo Camera | 1 | Waveshare module |
| Arduino Mega 2560 | 2 | Or clone |
| MG90S Servo Motors | 10-12 | 5-6 per arm |
| USB-A to USB-B cables | 2 | Jetson ↔ Arduino |
| 5V/4A Power Supply | 1 | For Jetson |
| 5V/3A Power Supply | 2 | For Arduinos/Servos |
| microSD Card (64GB+) | 1 | OS + ROS2 storage |

### Mechanical Parts

| Item | Quantity | Notes |
|------|----------|-------|
| Robot Arm Kit (Arduino) | 2 | 3D printed or acrylic |
| Camera Mount Frame | 1 | Aluminum extrusion |
| Workspace Platform | 1 | Wood/acrylic board |
| Sorting Bins | 4 | Small containers |
| Cable Management | - | Zip ties, channels |

### Optional

| Item | Quantity | Notes |
|------|----------|-------|
| LED Light Panels | 2 | Consistent lighting |
| USB Hub (powered) | 1 | If need more USB ports |
| Cooling Fan | 1 | For Jetson (if heavy load) |

---

## Assembly Checklist

- [ ] Build robot arms following reference design
- [ ] Test each servo individually
- [ ] Mount arms on workspace platform
- [ ] Install camera mount frame at correct height
- [ ] Attach stereo camera securely
- [ ] Connect camera to Jetson CSI ports
- [ ] Connect Arduinos to Jetson via USB
- [ ] Verify all power connections (separate supplies!)
- [ ] Check camera alignment (horizontal, level)
- [ ] Test basic movements (Arduino serial test)
- [ ] Measure and document workspace dimensions
- [ ] Set up consistent lighting

---

## Safety Considerations

⚠️ **Important Safety Notes**:

1. **Electrical**:
   - Never mix power supplies
   - Add fuses to servo power lines
   - Isolate servo ground from Arduino (if noise issues)

2. **Mechanical**:
   - Servo arms can move unexpectedly
   - Add E-stop button if possible
   - Limit servo speed during testing

3. **Operational**:
   - Keep workspace clear during operation
   - Emergency stop: kill ROS2 nodes or power off
   - Test arm movements in slow motion first

---

## Next Steps

After hardware assembly:
1. → Go to [Setup Guide](10_setup_guide.md) for software installation
2. → Perform camera calibration (see [Coordinate Frames](05_coordinate_frames.md))
3. → Test Arduino communication
4. → Proceed to Phase 1 of [Development Roadmap](07_development_roadmap.md)

---

**Hardware Architecture Complete ✓**
