#!/usr/bin/env python3
"""
Synthetic Baseline Test - ทดสอบว่า calibration algorithm ทำงานถูกต้องหรือไม่

จุดประสงค์:
  สร้างข้อมูล synthetic stereo images ด้วย baseline ที่ทราบแน่นอน (60mm)
  แล้วรัน stereo calibration algorithm เพื่อดูว่าคำนวณ baseline ได้ถูกต้องหรือไม่

หากผลทดสอบ:
  ✓ Baseline ใกล้เคียง 60mm → Algorithm ถูกต้อง, ปัญหาอยู่ที่ภาพจริง
    - Pattern อาจถูกพิมพ์ในขนาดที่ผิด (scaling)
    - Physical baseline จริงๆ อาจไม่ใช่ 60mm
    - Pattern spacing จริงอาจไม่ใช่ 18mm

  ✗ Baseline ผิดพลาดมาก → ปัญหาอยู่ที่ algorithm หรือ parameters

Parameters:
  - True baseline: 60mm
  - Pattern: Asymmetric circles 5x6
  - Pattern spacing: 18mm
  - Noise level: 0 (perfect)
  - Number of poses: 10
"""

import cv2
import numpy as np
import os
from datetime import datetime


def create_asymmetric_circles_pattern(rows, cols, spacing_mm):
    """
    สร้าง 3D coordinates ของ asymmetric circles pattern

    Args:
        rows: จำนวนแถว
        cols: จำนวนคอลัมน์
        spacing_mm: ระยะห่างแนวทแยง (mm)

    Returns:
        np.array: (N, 3) array ของ 3D points
    """
    objp = np.zeros((rows * cols, 3), np.float32)

    for i in range(rows):
        for j in range(cols):
            objp[i * cols + j] = [
                (2 * j + i % 2) * spacing_mm,
                i * spacing_mm,
                0
            ]

    return objp


def create_camera_matrix(focal_length_px, image_width, image_height):
    """
    สร้าง camera intrinsic matrix (K)

    Args:
        focal_length_px: Focal length ในหน่วย pixels
        image_width: ความกว้างภาพ (pixels)
        image_height: ความสูงภาพ (pixels)

    Returns:
        np.array: 3x3 camera matrix
    """
    cx = image_width / 2.0
    cy = image_height / 2.0

    K = np.array([
        [focal_length_px, 0, cx],
        [0, focal_length_px, cy],
        [0, 0, 1]
    ], dtype=np.float64)

    return K


def project_points_to_image(points_3d, K, R, T, distortion=None, noise_level=0.0):
    """
    Project 3D points ไปยัง image plane

    Args:
        points_3d: (N, 3) array ของ 3D points
        K: Camera matrix
        R: Rotation matrix
        T: Translation vector
        distortion: Distortion coefficients (optional)
        noise_level: Gaussian noise std dev ในหน่วย pixels

    Returns:
        np.array: (N, 2) array ของ 2D image points
    """
    # Convert to camera coordinate system
    points_cam = (R @ points_3d.T).T + T.flatten()

    # Project to image plane
    points_2d_homogeneous = (K @ points_cam.T).T
    points_2d = points_2d_homogeneous[:, :2] / points_2d_homogeneous[:, 2:3]

    # Add noise if specified
    if noise_level > 0:
        noise = np.random.normal(0, noise_level, points_2d.shape)
        points_2d += noise

    return points_2d.astype(np.float32)


def generate_random_poses(num_poses, pattern_objp, min_distance=300, max_distance=500):
    """
    สร้าง random poses สำหรับ calibration pattern

    Args:
        num_poses: จำนวน poses ที่ต้องการ
        pattern_objp: Pattern object points
        min_distance: ระยะห่างขั้นต่ำจาก camera (mm)
        max_distance: ระยะห่างสูงสุดจาก camera (mm)

    Returns:
        list: List of (R, T) tuples
    """
    poses = []

    # Calculate pattern center for reference
    pattern_center = np.mean(pattern_objp, axis=0)

    for i in range(num_poses):
        # Random distance
        distance = np.random.uniform(min_distance, max_distance)

        # Random rotation angles (in radians)
        # Limit rotation to realistic ranges
        rx = np.random.uniform(-30, 30) * np.pi / 180  # Pitch
        ry = np.random.uniform(-30, 30) * np.pi / 180  # Yaw
        rz = np.random.uniform(-15, 15) * np.pi / 180  # Roll

        # Create rotation matrix from euler angles
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])

        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])

        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])

        R = Rz @ Ry @ Rx

        # Position pattern at specified distance
        # Add some random offset in X and Y
        offset_x = np.random.uniform(-100, 100)
        offset_y = np.random.uniform(-100, 100)

        T = np.array([[offset_x, offset_y, distance]], dtype=np.float64)

        poses.append((R, T))

    return poses


def test_baseline_estimation(
    true_baseline_mm=60.0,
    pattern_rows=5,
    pattern_cols=6,
    pattern_spacing_mm=18.0,
    num_poses=10,
    noise_level=0.0,
    image_width=1280,
    image_height=720,
    focal_length_px=700.0,
    output_dir='test_synthetic_baseline'
):
    """
    ทดสอบการคำนวณ baseline ด้วยข้อมูล synthetic

    Args:
        true_baseline_mm: Baseline ที่แท้จริง (mm)
        pattern_rows: จำนวนแถวของ pattern
        pattern_cols: จำนวนคอลัมน์ของ pattern
        pattern_spacing_mm: ระยะห่างของ pattern (mm)
        num_poses: จำนวน poses
        noise_level: Noise level (pixels), 0 = perfect
        image_width: ความกว้างภาพ (pixels)
        image_height: ความสูงภาพ (pixels)
        focal_length_px: Focal length (pixels)
        output_dir: โฟลเดอร์สำหรับเก็บผลลัพธ์

    Returns:
        dict: ผลลัพธ์การทดสอบ
    """
    print("=" * 80)
    print("SYNTHETIC BASELINE TEST")
    print("=" * 80)
    print(f"\nTest Parameters:")
    print(f"  True Baseline:      {true_baseline_mm:.2f} mm")
    print(f"  Pattern:            Asymmetric Circles {pattern_rows}x{pattern_cols}")
    print(f"  Pattern Spacing:    {pattern_spacing_mm:.2f} mm")
    print(f"  Number of Poses:    {num_poses}")
    print(f"  Noise Level:        {noise_level:.2f} pixels ({'Perfect' if noise_level == 0 else 'Noisy'})")
    print(f"  Image Size:         {image_width}x{image_height}")
    print(f"  Focal Length:       {focal_length_px:.1f} pixels")
    print()

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # 1. Create pattern
    pattern_objp = create_asymmetric_circles_pattern(pattern_rows, pattern_cols, pattern_spacing_mm)
    print(f"✓ Pattern created: {len(pattern_objp)} points")

    # 2. Create camera matrices (identical for both cameras for simplicity)
    K_left = create_camera_matrix(focal_length_px, image_width, image_height)
    K_right = create_camera_matrix(focal_length_px, image_width, image_height)
    print(f"✓ Camera matrices created")

    # 3. Define stereo geometry
    # Left camera at origin
    R_left_stereo = np.eye(3)
    T_left_stereo = np.zeros((1, 3))

    # Right camera offset by baseline (along X-axis)
    R_right_stereo = np.eye(3)
    T_right_stereo = np.array([[true_baseline_mm, 0, 0]])

    print(f"✓ Stereo geometry defined:")
    print(f"  Left camera:  T = {T_left_stereo.flatten()}")
    print(f"  Right camera: T = {T_right_stereo.flatten()}")
    print(f"  Baseline:     {np.linalg.norm(T_right_stereo - T_left_stereo):.2f} mm")

    # 4. Generate random poses
    print(f"\n✓ Generating {num_poses} random poses...")
    poses = generate_random_poses(num_poses, pattern_objp)

    # 5. Generate synthetic image points
    print(f"✓ Projecting pattern to image planes...")
    imgpoints_left = []
    imgpoints_right = []
    objpoints = []

    valid_poses = 0

    for i, (R_pattern, T_pattern) in enumerate(poses):
        # Project to left camera
        points_2d_left = project_points_to_image(
            pattern_objp, K_left, R_pattern, T_pattern, noise_level=noise_level
        )

        # Transform pattern to right camera coordinate system
        # Pattern pose relative to right camera
        T_pattern_right = T_pattern - T_right_stereo

        # Project to right camera
        points_2d_right = project_points_to_image(
            pattern_objp, K_right, R_pattern, T_pattern_right, noise_level=noise_level
        )

        # Check if all points are within image bounds
        left_valid = np.all((points_2d_left[:, 0] >= 0) & (points_2d_left[:, 0] < image_width) &
                           (points_2d_left[:, 1] >= 0) & (points_2d_left[:, 1] < image_height))

        right_valid = np.all((points_2d_right[:, 0] >= 0) & (points_2d_right[:, 0] < image_width) &
                            (points_2d_right[:, 1] >= 0) & (points_2d_right[:, 1] < image_height))

        if left_valid and right_valid:
            # Reshape to match OpenCV format: (N, 1, 2)
            imgpoints_left.append(points_2d_left.reshape(-1, 1, 2))
            imgpoints_right.append(points_2d_right.reshape(-1, 1, 2))
            objpoints.append(pattern_objp)
            valid_poses += 1
            print(f"  Pose {i+1:2d}: ✓ Valid (distance: {T_pattern[0, 2]:.1f} mm)")
        else:
            print(f"  Pose {i+1:2d}: ✗ Out of bounds (skipped)")

    print(f"\n✓ Valid poses: {valid_poses}/{num_poses}")

    if valid_poses < 3:
        print("\n✗ ERROR: Not enough valid poses for calibration!")
        return None

    # 6. Run stereo calibration
    print("\n" + "=" * 80)
    print("RUNNING STEREO CALIBRATION")
    print("=" * 80)

    # No distortion for synthetic data
    dist_left = np.zeros((5, 1))
    dist_right = np.zeros((5, 1))

    # Step 1: Calibrate left camera
    print("\nStep 1: Calibrating left camera...")
    ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
        objpoints, imgpoints_left, (image_width, image_height), None, None
    )
    print(f"  RMS error: {ret_left:.6f} pixels")

    # Step 2: Calibrate right camera
    print("\nStep 2: Calibrating right camera...")
    ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
        objpoints, imgpoints_right, (image_width, image_height), None, None
    )
    print(f"  RMS error: {ret_right:.6f} pixels")

    # Step 3: Stereo calibration
    print("\nStep 3: Stereo calibration...")
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    ret_stereo, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtx_left, dist_left,
        mtx_right, dist_right,
        (image_width, image_height),
        criteria=criteria,
        flags=flags
    )
    print(f"  RMS error: {ret_stereo:.6f} pixels")

    # 7. Calculate estimated baseline
    estimated_baseline_mm = np.linalg.norm(T)

    print("\n" + "=" * 80)
    print("RESULTS")
    print("=" * 80)
    print(f"\nBaseline Comparison:")
    print(f"  True Baseline:       {true_baseline_mm:.4f} mm")
    print(f"  Estimated Baseline:  {estimated_baseline_mm:.4f} mm")
    print(f"  Error:               {estimated_baseline_mm - true_baseline_mm:.4f} mm")
    print(f"  Error %:             {abs(estimated_baseline_mm - true_baseline_mm) / true_baseline_mm * 100:.2f}%")

    # Translation vector comparison
    print(f"\nTranslation Vector (T):")
    print(f"  True:      [{T_right_stereo[0, 0]:.4f}, {T_right_stereo[0, 1]:.4f}, {T_right_stereo[0, 2]:.4f}]")
    print(f"  Estimated: [{T[0, 0]:.4f}, {T[1, 0]:.4f}, {T[2, 0]:.4f}]")

    # Rotation matrix (should be close to identity for parallel cameras)
    print(f"\nRotation Matrix (R):")
    print(f"  (Should be close to identity for parallel cameras)")
    print(R)
    print(f"  Frobenius norm of (R - I): {np.linalg.norm(R - np.eye(3)):.6f}")

    # Calibration quality
    print(f"\nCalibration Quality:")
    print(f"  Stereo RMS error: {ret_stereo:.6f} pixels")
    if ret_stereo < 0.01:
        quality = "EXCELLENT (synthetic data)"
    elif ret_stereo < 0.1:
        quality = "Very Good"
    elif ret_stereo < 0.5:
        quality = "Good"
    else:
        quality = "Poor"
    print(f"  Quality: {quality}")

    # Interpretation
    print("\n" + "=" * 80)
    print("INTERPRETATION")
    print("=" * 80)

    error_pct = abs(estimated_baseline_mm - true_baseline_mm) / true_baseline_mm * 100

    if error_pct < 1.0:
        print("\n✓ EXCELLENT: Algorithm คำนวณ baseline ได้ถูกต้อง!")
        print("\n  ➜ สรุป: ปัญหา baseline = 436mm (แทน ~60mm) น่าจะมาจาก:")
        print("     1. Pattern ถูกพิมพ์ในขนาดที่ผิด (scale ผิด)")
        print("     2. Pattern spacing จริงๆ ไม่ใช่ 18mm")
        print("     3. Physical baseline ของกล้องไม่ใช่ 60mm (วัดระยะจริงดู)")
        print("\n  ➜ แนะนำ:")
        print("     - วัดขนาดจริงของ printed pattern ด้วยไม้บรรทัด")
        print("     - วัด physical baseline (ระยะห่างระหว่างกล้อง) ด้วยเวอร์เนีย")
        print("     - ตรวจสอบการตั้งค่าเครื่องพิมพ์ (fit to page, scale to fit, etc.)")
    elif error_pct < 5.0:
        print("\n⚠ GOOD: Algorithm ทำงานได้ดี แต่มี error เล็กน้อย")
        print(f"  Error: {error_pct:.2f}% อาจมาจาก numerical precision")
    else:
        print("\n✗ WARNING: Algorithm มีปัญหา หรือ parameters ไม่ถูกต้อง!")
        print(f"  Error: {error_pct:.2f}% สูงเกินไป")
        print("\n  ➜ ตรวจสอบ:")
        print("     - Pattern spacing ที่ใช้ในการ calibrate")
        print("     - Flags และ criteria ใน stereoCalibrate()")
        print("     - การสร้าง object points")

    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_file = os.path.join(output_dir, f'baseline_test_{timestamp}.txt')

    with open(result_file, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("SYNTHETIC BASELINE TEST RESULTS\n")
        f.write("=" * 80 + "\n\n")
        f.write(f"Test Parameters:\n")
        f.write(f"  True Baseline:      {true_baseline_mm:.2f} mm\n")
        f.write(f"  Pattern:            Asymmetric Circles {pattern_rows}x{pattern_cols}\n")
        f.write(f"  Pattern Spacing:    {pattern_spacing_mm:.2f} mm\n")
        f.write(f"  Number of Poses:    {num_poses} (valid: {valid_poses})\n")
        f.write(f"  Noise Level:        {noise_level:.2f} pixels\n")
        f.write(f"  Image Size:         {image_width}x{image_height}\n")
        f.write(f"  Focal Length:       {focal_length_px:.1f} pixels\n\n")
        f.write(f"Results:\n")
        f.write(f"  True Baseline:       {true_baseline_mm:.4f} mm\n")
        f.write(f"  Estimated Baseline:  {estimated_baseline_mm:.4f} mm\n")
        f.write(f"  Error:               {estimated_baseline_mm - true_baseline_mm:.4f} mm\n")
        f.write(f"  Error %:             {error_pct:.2f}%\n\n")
        f.write(f"Translation Vector:\n")
        f.write(f"  True:      [{T_right_stereo[0, 0]:.4f}, {T_right_stereo[0, 1]:.4f}, {T_right_stereo[0, 2]:.4f}]\n")
        f.write(f"  Estimated: [{T[0, 0]:.4f}, {T[1, 0]:.4f}, {T[2, 0]:.4f}]\n\n")
        f.write(f"Calibration Quality:\n")
        f.write(f"  Stereo RMS error: {ret_stereo:.6f} pixels\n")
        f.write(f"  Quality: {quality}\n")

    print(f"\n✓ Results saved to: {result_file}")
    print("=" * 80)

    return {
        'true_baseline_mm': true_baseline_mm,
        'estimated_baseline_mm': estimated_baseline_mm,
        'error_mm': estimated_baseline_mm - true_baseline_mm,
        'error_pct': error_pct,
        'stereo_rms_error': ret_stereo,
        'translation_vector': T.flatten(),
        'rotation_matrix': R,
        'num_poses': valid_poses
    }


def main():
    """Run baseline test with specified parameters"""

    # Test parameters
    TRUE_BASELINE = 60.0  # mm - ค่าที่ต้องการทดสอบ
    NOISE_LEVEL = 0.0     # pixels - perfect (no noise)
    NUM_POSES = 10        # จำนวน poses

    # Pattern parameters (ตาม spec ที่ใช้อยู่)
    PATTERN_ROWS = 5
    PATTERN_COLS = 6
    PATTERN_SPACING = 18.0  # mm

    # Camera parameters (ประมาณค่าของ IMX219)
    IMAGE_WIDTH = 1280
    IMAGE_HEIGHT = 720
    FOCAL_LENGTH = 700.0  # pixels (approximate for IMX219)

    result = test_baseline_estimation(
        true_baseline_mm=TRUE_BASELINE,
        pattern_rows=PATTERN_ROWS,
        pattern_cols=PATTERN_COLS,
        pattern_spacing_mm=PATTERN_SPACING,
        num_poses=NUM_POSES,
        noise_level=NOISE_LEVEL,
        image_width=IMAGE_WIDTH,
        image_height=IMAGE_HEIGHT,
        focal_length_px=FOCAL_LENGTH,
        output_dir='test_synthetic_baseline'
    )

    if result is None:
        print("\n✗ Test failed!")
        return

    print("\n✓ Test completed successfully!")


if __name__ == '__main__':
    main()
