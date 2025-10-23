# 5. Coordinate Frames & Transformations

## Frame Hierarchy

```
world (fixed reference frame)
  │
  ├── camera_link (stereo camera base)
  │     ├── camera_left_optical
  │     └── camera_right_optical
  │
  ├── arm_left_base
  │     ├── arm_left_link1
  │     ├── arm_left_link2
  │     ├── ...
  │     └── arm_left_gripper
  │
  └── arm_right_base
        ├── arm_right_link1
        ├── arm_right_link2
        ├── ...
        └── arm_right_gripper
```

---

## Frame Definitions

### 1. World Frame

- **Origin**: Center of input pile area
- **Orientation**:
  - **X-axis**: Points right (→)
  - **Y-axis**: Points forward/away from camera (↑)
  - **Z-axis**: Points up (⬆)
- **Purpose**: Common reference frame for all components

```
        Y (forward)
        ↑
        │
        │
        └────→ X (right)
       /
      /
     ↙ Z (up)
```

### 2. Camera Frames

#### camera_link
- **Parent**: world
- **Translation**: [0, -0.5, 0.6] (50cm back, 60cm up from world origin)
- **Rotation**: Looking down (-90° pitch)

#### camera_left_optical / camera_right_optical
- **Parent**: camera_link
- **Translation**: Left: [-0.03, 0, 0], Right: [+0.03, 0, 0] (60mm baseline)
- **Optical frame**: ROS standard (Z forward, X right, Y down)

### 3. Arm Frames

#### arm_left_base
- **Parent**: world
- **Translation**: [-0.15, 0, 0] (15cm to the left)
- **Rotation**: Identity (aligned with world)

#### arm_right_base
- **Parent**: world
- **Translation**: [+0.15, 0, 0] (15cm to the right)
- **Rotation**: Identity

---

## Required Transformations

### Camera Calibration

#### Intrinsic Parameters (per camera)

Camera matrix **K**:
```
K = [fx  0  cx]
    [0  fy  cy]
    [0   0   1]
```

Distortion coefficients **D**:
```
D = [k1, k2, p1, p2, k3]
```

**Calibration Process**:

```python
# stereo_calibration.py
import cv2
import numpy as np
import glob

# Checkerboard parameters
CHECKERBOARD = (9, 6)  # Inner corners
SQUARE_SIZE = 25  # mm

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# Arrays to store points
objpoints = []  # 3D points in real world space
imgpoints_left = []  # 2D points in left image
imgpoints_right = []  # 2D points in right image

# Load calibration images
images_left = glob.glob('calib_left/*.jpg')
images_right = glob.glob('calib_right/*.jpg')

for img_left_path, img_right_path in zip(images_left, images_right):
    img_left = cv2.imread(img_left_path)
    img_right = cv2.imread(img_right_path)

    gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

    # Find checkerboard corners
    ret_left, corners_left = cv2.findChessboardCorners(gray_left, CHECKERBOARD)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, CHECKERBOARD)

    if ret_left and ret_right:
        objpoints.append(objp)
        imgpoints_left.append(corners_left)
        imgpoints_right.append(corners_right)

# Calibrate left camera
ret_left, K_left, D_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
    objpoints, imgpoints_left, gray_left.shape[::-1], None, None
)

# Calibrate right camera
ret_right, K_right, D_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
    objpoints, imgpoints_right, gray_right.shape[::-1], None, None
)

# Stereo calibration
ret, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    K_left, D_left, K_right, D_right,
    gray_left.shape[::-1],
    flags=cv2.CALIB_FIX_INTRINSIC
)

# Stereo rectification
R_left, R_right, P_left, P_right, Q, roi_left, roi_right = cv2.stereoRectify(
    K_left, D_left, K_right, D_right,
    gray_left.shape[::-1], R, T,
    alpha=0
)

# Save calibration
calib_data = {
    'K_left': K_left,
    'D_left': D_left,
    'K_right': K_right,
    'D_right': D_right,
    'R': R,
    'T': T,
    'R_left': R_left,
    'R_right': R_right,
    'P_left': P_left,
    'P_right': P_right,
    'Q': Q
}

np.save('stereo_calibration.npy', calib_data)
```

---

### Hand-Eye Calibration

Purpose: Find transformation from camera frame to arm base frame

**Method**: Eye-to-hand calibration (camera fixed, arm moves)

```python
# hand_eye_calibration.py
import numpy as np
from scipy.spatial.transform import Rotation
import cv2

def calibrate_hand_eye(camera_points, arm_points):
    """
    camera_points: List of 3D points in camera frame [(x,y,z), ...]
    arm_points: List of 3D points in arm frame [(x,y,z), ...]

    Returns: 4x4 transformation matrix from camera to arm
    """
    # Convert to numpy arrays
    P_camera = np.array(camera_points)
    P_arm = np.array(arm_points)

    # Find centroids
    centroid_camera = np.mean(P_camera, axis=0)
    centroid_arm = np.mean(P_arm, axis=0)

    # Center the points
    P_camera_centered = P_camera - centroid_camera
    P_arm_centered = P_arm - centroid_arm

    # Compute covariance matrix
    H = P_camera_centered.T @ P_arm_centered

    # SVD
    U, S, Vt = np.linalg.svd(H)

    # Rotation matrix
    R = Vt.T @ U.T

    # Handle reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Translation vector
    t = centroid_arm - R @ centroid_camera

    # Build 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

# Data collection procedure:
# 1. Place calibration target (e.g., checkerboard corner) at known positions
# 2. Detect position from camera (P_camera)
# 3. Move arm tip to touch the target, record arm tip position (P_arm)
# 4. Repeat for 10-20 different positions
# 5. Run calibration

camera_points = [
    [0.05, 0.10, 0.50],  # Example points in camera frame
    [0.10, 0.15, 0.48],
    # ... 10-20 points total
]

arm_points = [
    [0.20, 0.10, 0.05],  # Corresponding points in arm frame
    [0.25, 0.15, 0.03],
    # ...
]

T_camera_to_arm = calibrate_hand_eye(camera_points, arm_points)
print("Transformation matrix:")
print(T_camera_to_arm)

# Save to file
np.save('hand_eye_calib_left.npy', T_camera_to_arm)
```

---

## Coordinate Transformations

### Camera to World

```python
def camera_to_world(point_camera, T_camera_to_world):
    """
    Transform point from camera frame to world frame

    point_camera: [x, y, z] in camera frame
    T_camera_to_world: 4x4 transformation matrix

    Returns: [x, y, z] in world frame
    """
    point_homo = np.array([point_camera[0], point_camera[1], point_camera[2], 1.0])
    point_world_homo = T_camera_to_world @ point_homo
    return point_world_homo[:3]
```

### Pixel to 3D (from stereo depth)

```python
def pixel_to_3d(u, v, disparity, Q):
    """
    Convert pixel coordinates + disparity to 3D point

    u, v: pixel coordinates in left image
    disparity: disparity value at (u, v)
    Q: Disparity-to-depth mapping matrix (from stereoRectify)

    Returns: [X, Y, Z] in camera frame
    """
    # Homogeneous coordinates
    point_2d = np.array([u, v, disparity, 1.0])

    # Reproject to 3D
    point_3d_homo = Q @ point_2d
    point_3d = point_3d_homo[:3] / point_3d_homo[3]

    return point_3d

# Usage:
Q = calib_data['Q']
X, Y, Z = pixel_to_3d(640, 360, disparity_value, Q)
```

### World to Arm

```python
def world_to_arm(point_world, T_arm_to_world):
    """
    Transform point from world frame to arm base frame

    point_world: [x, y, z] in world frame
    T_arm_to_world: 4x4 transformation matrix (arm base in world frame)

    Returns: [x, y, z] in arm frame
    """
    # Inverse transformation
    T_world_to_arm = np.linalg.inv(T_arm_to_world)

    point_homo = np.array([point_world[0], point_world[1], point_world[2], 1.0])
    point_arm_homo = T_world_to_arm @ point_homo
    return point_arm_homo[:3]
```

---

## TF2 Integration (ROS2)

### Publishing Static Transforms

```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import numpy as np

class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish all static transforms
        self.publish_camera_transforms()
        self.publish_arm_transforms()

    def publish_camera_transforms(self):
        # world -> camera_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'camera_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = -0.5
        t.transform.translation.z = 0.6
        # Rotation: -90° pitch (looking down)
        t.transform.rotation.x = -0.7071
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.7071

        self.tf_broadcaster.sendTransform(t)

    def publish_arm_transforms(self):
        # world -> arm_left_base
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'arm_left_base'
        t.transform.translation.x = -0.15
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # Identity rotation

        self.tf_broadcaster.sendTransform(t)

        # Similar for arm_right_base
```

---

## Calibration Checklist

- [ ] **Stereo Camera Calibration**
  - [ ] Collect 20+ checkerboard images (various angles)
  - [ ] Run `stereo_calibration.py`
  - [ ] Verify reprojection error < 0.5 pixels
  - [ ] Save `stereo_calib.yaml`

- [ ] **Hand-Eye Calibration (Left Arm)**
  - [ ] Place target at 10-15 positions in workspace
  - [ ] Record camera detections + arm positions
  - [ ] Run `hand_eye_calibration.py`
  - [ ] Verify error < 5mm
  - [ ] Save `hand_eye_calib_left.npy`

- [ ] **Hand-Eye Calibration (Right Arm)**
  - [ ] Repeat for right arm
  - [ ] Save `hand_eye_calib_right.npy`

- [ ] **Validation**
  - [ ] Place known object in workspace
  - [ ] Verify camera detection matches arm reach
  - [ ] Test pick-and-place accuracy

---

**Coordinate Frames Complete ✓**
