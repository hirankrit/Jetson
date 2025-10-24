#!/usr/bin/env python3
"""
Stereo Camera Calibration using Asymmetric Circles Pattern
Computes intrinsic and extrinsic parameters for stereo vision

Pattern specifications:
- Rows: 5
- Columns: 6
- Diagonal Spacing: 18mm (measured from printed pattern - CONFIRMED)
- Circle Diameter: 14mm
- Total circles: 33
"""

import cv2
import numpy as np
import glob
import yaml
import os


def calibrate_stereo_asymmetric_circles(
    images_left_path,
    images_right_path,
    pattern_rows=5,
    pattern_cols=6,
    spacing_mm=18.0,
    output_file='stereo_calib.yaml'
):
    """
    Perform stereo calibration using asymmetric circles pattern

    Args:
        images_left_path: Path to left camera images (e.g., 'calib_images/left/*.jpg')
        images_right_path: Path to right camera images
        pattern_rows: Number of rows in pattern
        pattern_cols: Number of columns in pattern
        spacing_mm: Diagonal spacing between circles in mm
        output_file: Output YAML file path

    Returns:
        dict: Calibration results
    """

    print("=" * 70)
    print("Stereo Camera Calibration - Asymmetric Circles Pattern")
    print("=" * 70)
    print(f"\nPattern: {pattern_rows} rows × {pattern_cols} columns")
    print(f"Spacing: {spacing_mm}mm (diagonal)")
    print()

    # Prepare object points (3D points in real world space)
    # For asymmetric circles grid, points are at (0,0), (1,0), (1,1), (2,1), etc.
    objp = np.zeros((pattern_rows * pattern_cols, 3), np.float32)

    # Asymmetric grid pattern positions
    for i in range(pattern_rows):
        for j in range(pattern_cols):
            objp[i * pattern_cols + j] = [
                (2 * j + i % 2) * spacing_mm,
                i * spacing_mm,
                0
            ]

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints_left = []  # 2D points in left image plane
    imgpoints_right = []  # 2D points in right image plane

    # Load images
    images_left = sorted(glob.glob(images_left_path))
    images_right = sorted(glob.glob(images_right_path))

    print(f"Found {len(images_left)} left images")
    print(f"Found {len(images_right)} right images")

    if len(images_left) != len(images_right):
        print("ERROR: Number of left and right images don't match!")
        return None

    if len(images_left) < 10:
        print("ERROR: Need at least 10 image pairs for calibration!")
        return None

    # Setup blob detector for circles
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 0
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 5000
    params.filterByCircularity = True
    params.minCircularity = 0.8
    params.filterByConvexity = True
    params.minConvexity = 0.87
    params.filterByInertia = True
    params.minInertiaRatio = 0.6

    detector = cv2.SimpleBlobDetector_create(params)

    print("\nProcessing images...")
    valid_pairs = 0

    for img_left_path, img_right_path in zip(images_left, images_right):
        img_left = cv2.imread(img_left_path)
        img_right = cv2.imread(img_right_path)

        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        # Find circles in both images
        ret_left, corners_left = cv2.findCirclesGrid(
            gray_left,
            (pattern_cols, pattern_rows),
            flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
            blobDetector=detector
        )

        ret_right, corners_right = cv2.findCirclesGrid(
            gray_right,
            (pattern_cols, pattern_rows),
            flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
            blobDetector=detector
        )

        if ret_left and ret_right:
            objpoints.append(objp)
            imgpoints_left.append(corners_left)
            imgpoints_right.append(corners_right)
            valid_pairs += 1
            print(f"  ✓ {os.path.basename(img_left_path)}")
        else:
            print(f"  ✗ {os.path.basename(img_left_path)} - Pattern not found")

    print(f"\nValid image pairs: {valid_pairs}/{len(images_left)}")

    if valid_pairs < 10:
        print("ERROR: Not enough valid pairs for calibration!")
        return None

    # Get image size
    img_shape = gray_left.shape[::-1]

    print("\n" + "=" * 70)
    print("Step 1: Calibrating left camera...")
    print("=" * 70)

    ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
        objpoints, imgpoints_left, img_shape, None, None
    )

    print(f"Left camera RMS re-projection error: {ret_left:.4f} pixels")

    print("\n" + "=" * 70)
    print("Step 2: Calibrating right camera...")
    print("=" * 70)

    ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
        objpoints, imgpoints_right, img_shape, None, None
    )

    print(f"Right camera RMS re-projection error: {ret_right:.4f} pixels")

    print("\n" + "=" * 70)
    print("Step 3: Stereo calibration...")
    print("=" * 70)

    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    ret_stereo, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtx_left, dist_left,
        mtx_right, dist_right,
        img_shape,
        criteria=criteria,
        flags=flags
    )

    print(f"Stereo calibration RMS error: {ret_stereo:.4f} pixels")

    print("\n" + "=" * 70)
    print("Step 4: Stereo rectification...")
    print("=" * 70)

    R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
        mtx_left, dist_left,
        mtx_right, dist_right,
        img_shape, R, T,
        alpha=0
    )

    # Compute rectification maps
    map_left_x, map_left_y = cv2.initUndistortRectifyMap(
        mtx_left, dist_left, R1, P1, img_shape, cv2.CV_32FC1
    )

    map_right_x, map_right_y = cv2.initUndistortRectifyMap(
        mtx_right, dist_right, R2, P2, img_shape, cv2.CV_32FC1
    )

    print("Rectification complete!")

    # Baseline distance
    baseline = np.linalg.norm(T)
    print(f"\nBaseline distance: {baseline:.2f} mm ({baseline/10:.2f} cm)")

    # Prepare calibration data
    calib_data = {
        'image_width': int(img_shape[0]),
        'image_height': int(img_shape[1]),
        'pattern_type': 'asymmetric_circles',
        'pattern_rows': int(pattern_rows),
        'pattern_cols': int(pattern_cols),
        'pattern_spacing_mm': float(spacing_mm),
        'baseline_mm': float(baseline),
        'stereo_rms_error': float(ret_stereo),
        'left_camera': {
            'camera_matrix': mtx_left.tolist(),
            'distortion_coefficients': dist_left.tolist(),
            'rectification_matrix': R1.tolist(),
            'projection_matrix': P1.tolist(),
            'rms_error': float(ret_left)
        },
        'right_camera': {
            'camera_matrix': mtx_right.tolist(),
            'distortion_coefficients': dist_right.tolist(),
            'rectification_matrix': R2.tolist(),
            'projection_matrix': P2.tolist(),
            'rms_error': float(ret_right)
        },
        'stereo': {
            'rotation_matrix': R.tolist(),
            'translation_vector': T.tolist(),
            'essential_matrix': E.tolist(),
            'fundamental_matrix': F.tolist(),
            'Q_matrix': Q.tolist()
        }
    }

    # Save to YAML
    print(f"\nSaving calibration to: {output_file}")
    with open(output_file, 'w') as f:
        yaml.dump(calib_data, f, default_flow_style=False)

    # Save rectification maps (binary format for faster loading)
    np.savez_compressed(
        'rectification_maps.npz',
        map_left_x=map_left_x,
        map_left_y=map_left_y,
        map_right_x=map_right_x,
        map_right_y=map_right_y
    )
    print("Rectification maps saved to: rectification_maps.npz")

    print("\n" + "=" * 70)
    print("Calibration Quality Assessment:")
    print("=" * 70)
    print(f"Stereo RMS error: {ret_stereo:.4f} pixels")
    if ret_stereo < 0.5:
        print("  → Excellent! (< 0.5)")
    elif ret_stereo < 1.0:
        print("  → Good (0.5 - 1.0)")
    elif ret_stereo < 2.0:
        print("  → Acceptable (1.0 - 2.0)")
    else:
        print("  → Poor (> 2.0) - Consider recalibrating")

    print(f"\nBaseline: {baseline:.2f} mm")
    if 50 <= baseline <= 80:
        print("  → Good for 40-100cm working distance")
    elif baseline < 50:
        print("  → Short baseline - better for close objects")
    else:
        print("  → Long baseline - better for distant objects")

    print("\n" + "=" * 70)
    print("Calibration complete!")
    print("=" * 70)
    print("\nNext step: Run test_depth_map.py to verify depth accuracy")

    return calib_data


def main():
    # Run calibration
    calib_data = calibrate_stereo_asymmetric_circles(
        images_left_path='calib_images/left/*.jpg',
        images_right_path='calib_images/right/*.jpg',
        pattern_rows=5,
        pattern_cols=6,
        spacing_mm=18.0,  # CONFIRMED: 18mm measured from printed pattern
        output_file='stereo_calib.yaml'
    )

    if calib_data is None:
        print("\nCalibration failed!")
        return

    print("\n✓ Calibration files created:")
    print("  - stereo_calib.yaml (calibration parameters)")
    print("  - rectification_maps.npz (for fast rectification)")


if __name__ == '__main__':
    main()
