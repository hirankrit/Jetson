#!/usr/bin/env python3
"""
Test Stereo Depth Estimation with IMX219 Camera
Uses OpenCV StereoSGBM (classical method) to understand depth pipeline
Before testing Isaac ROS ESS (DNN-based)
"""

import cv2
import numpy as np
import yaml
import sys
from pathlib import Path

def load_calibration(calib_file: str = "stereo_calib.yaml"):
    """Load stereo calibration data"""
    print(f"\n{'='*60}")
    print("📁 Loading Calibration Data")
    print(f"{'='*60}")

    if not Path(calib_file).exists():
        print(f"❌ Calibration file not found: {calib_file}")
        print("   Please run stereo calibration first!")
        return None

    with open(calib_file, 'r') as f:
        calib = yaml.safe_load(f)

    # Extract important parameters
    baseline = calib['baseline_mm'] / 1000.0  # Convert to meters
    fx = calib['left_camera']['camera_matrix'][0][0]  # Focal length in pixels

    # Rectification matrices
    R1 = np.array(calib['left_camera']['rectification_matrix'])
    R2 = np.array(calib['right_camera']['rectification_matrix'])

    # Projection matrices
    P1 = np.array(calib['left_camera']['projection_matrix'])
    P2 = np.array(calib['right_camera']['projection_matrix'])

    # Camera matrices
    K1 = np.array(calib['left_camera']['camera_matrix'])
    K2 = np.array(calib['right_camera']['camera_matrix'])

    # Distortion coefficients
    D1 = np.array(calib['left_camera']['distortion_coefficients'][0])
    D2 = np.array(calib['right_camera']['distortion_coefficients'][0])

    # Q matrix for 3D reprojection
    Q = np.array(calib['stereo']['Q_matrix'])

    print(f"✅ Calibration loaded successfully!")
    print(f"   Baseline: {baseline*1000:.2f} mm")
    print(f"   Focal length: {fx:.2f} pixels")
    print(f"   Image size: {calib['image_width']}x{calib['image_height']}")
    print(f"   RMS error: {calib['stereo_rms_error']:.3f}")

    return {
        'baseline': baseline,
        'focal_length': fx,
        'K1': K1, 'K2': K2,
        'D1': D1, 'D2': D2,
        'R1': R1, 'R2': R2,
        'P1': P1, 'P2': P2,
        'Q': Q,
        'image_size': (calib['image_width'], calib['image_height'])
    }


def rectify_images(img_left, img_right, calib):
    """Rectify stereo images using calibration data"""
    print(f"\n{'='*60}")
    print("🔧 Rectifying Stereo Images")
    print(f"{'='*60}")

    h, w = img_left.shape[:2]

    # Compute rectification maps
    map1_left, map2_left = cv2.initUndistortRectifyMap(
        calib['K1'], calib['D1'], calib['R1'], calib['P1'],
        (w, h), cv2.CV_32FC1
    )

    map1_right, map2_right = cv2.initUndistortRectifyMap(
        calib['K2'], calib['D2'], calib['R2'], calib['P2'],
        (w, h), cv2.CV_32FC1
    )

    # Apply rectification
    rect_left = cv2.remap(img_left, map1_left, map2_left, cv2.INTER_LINEAR)
    rect_right = cv2.remap(img_right, map1_right, map2_right, cv2.INTER_LINEAR)

    print("✅ Images rectified successfully!")

    return rect_left, rect_right


def compute_disparity(rect_left, rect_right):
    """Compute disparity map using StereoSGBM"""
    print(f"\n{'='*60}")
    print("🔬 Computing Disparity Map (StereoSGBM)")
    print(f"{'='*60}")

    # Convert to grayscale if needed
    if len(rect_left.shape) == 3:
        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
    else:
        gray_left = rect_left
        gray_right = rect_right

    # StereoSGBM parameters (tuned for good results)
    window_size = 5
    min_disp = 0
    num_disp = 128  # Must be divisible by 16

    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=8 * 3 * window_size ** 2,
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    print(f"   Parameters:")
    print(f"   - Block size: {window_size}")
    print(f"   - Disparity range: {min_disp} to {min_disp + num_disp}")
    print(f"   - Mode: SGBM_3WAY")

    # Compute disparity
    disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

    print(f"✅ Disparity computed!")
    print(f"   Shape: {disparity.shape}")
    print(f"   Range: {disparity.min():.1f} to {disparity.max():.1f} pixels")
    print(f"   Valid pixels: {(disparity > 0).sum():,} / {disparity.size:,}")

    return disparity


def disparity_to_depth(disparity, calib):
    """Convert disparity to depth in meters"""
    print(f"\n{'='*60}")
    print("📏 Converting Disparity to Depth")
    print(f"{'='*60}")

    baseline = calib['baseline']  # meters
    focal_length = calib['focal_length']  # pixels

    # Avoid division by zero
    depth = np.zeros_like(disparity)
    valid_mask = disparity > 0

    # Depth = (baseline * focal_length) / disparity
    depth[valid_mask] = (baseline * focal_length) / disparity[valid_mask]

    # Filter unrealistic depths (too close or too far)
    min_depth = 0.1  # 10 cm
    max_depth = 10.0  # 10 meters
    depth[depth < min_depth] = 0
    depth[depth > max_depth] = 0

    valid_depths = depth[depth > 0]

    print(f"✅ Depth computed!")
    print(f"   Formula: depth = ({baseline*1000:.2f}mm × {focal_length:.2f}px) / disparity")
    print(f"   Valid range: {min_depth}m to {max_depth}m")
    if len(valid_depths) > 0:
        print(f"   Depth range: {valid_depths.min():.2f}m to {valid_depths.max():.2f}m")
        print(f"   Mean depth: {valid_depths.mean():.2f}m")
        print(f"   Median depth: {np.median(valid_depths):.2f}m")
    else:
        print(f"   ⚠️  No valid depth values!")

    return depth


def visualize_results(img_left, img_right, rect_left, rect_right,
                     disparity, depth, output_prefix="stereo_depth"):
    """Visualize and save all results"""
    print(f"\n{'='*60}")
    print("🎨 Visualizing Results")
    print(f"{'='*60}")

    # 1. Original images side-by-side
    original = np.hstack((img_left, img_right))
    cv2.imwrite(f"{output_prefix}_01_original.jpg", original)
    print(f"✅ Saved: {output_prefix}_01_original.jpg")

    # 2. Rectified images side-by-side with epipolar lines
    rect_vis = rect_left.copy()
    rect_vis_right = rect_right.copy()
    h = rect_vis.shape[0]
    for y in range(0, h, 40):
        cv2.line(rect_vis, (0, y), (rect_vis.shape[1], y), (0, 255, 0), 1)
        cv2.line(rect_vis_right, (0, y), (rect_vis_right.shape[1], y), (0, 255, 0), 1)
    rectified = np.hstack((rect_vis, rect_vis_right))
    cv2.imwrite(f"{output_prefix}_02_rectified.jpg", rectified)
    print(f"✅ Saved: {output_prefix}_02_rectified.jpg (with epipolar lines)")

    # 3. Disparity map (colorized)
    disparity_vis = disparity.copy()
    disparity_vis[disparity_vis < 0] = 0
    disparity_vis = (disparity_vis / disparity_vis.max() * 255).astype(np.uint8)
    disparity_color = cv2.applyColorMap(disparity_vis, cv2.COLORMAP_JET)
    cv2.imwrite(f"{output_prefix}_03_disparity.jpg", disparity_color)
    print(f"✅ Saved: {output_prefix}_03_disparity.jpg")

    # 4. Depth map (colorized)
    depth_vis = depth.copy()
    valid_mask = depth_vis > 0
    if valid_mask.any():
        depth_vis[~valid_mask] = 0
        # Normalize for visualization
        depth_min = depth_vis[valid_mask].min()
        depth_max = depth_vis[valid_mask].max()
        depth_norm = np.zeros_like(depth_vis)
        depth_norm[valid_mask] = (depth_vis[valid_mask] - depth_min) / (depth_max - depth_min)
        depth_color = (depth_norm * 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_color, cv2.COLORMAP_TURBO)
        cv2.imwrite(f"{output_prefix}_04_depth.jpg", depth_color)
        print(f"✅ Saved: {output_prefix}_04_depth.jpg")
    else:
        print(f"⚠️  No valid depth for visualization")

    # 5. Side-by-side comparison
    comparison = np.hstack((rect_left, depth_color))
    cv2.imwrite(f"{output_prefix}_05_comparison.jpg", comparison)
    print(f"✅ Saved: {output_prefix}_05_comparison.jpg (image + depth)")

    print(f"\n📊 Results Summary:")
    print(f"   - Original images: {output_prefix}_01_original.jpg")
    print(f"   - Rectified (with epipolar lines): {output_prefix}_02_rectified.jpg")
    print(f"   - Disparity map: {output_prefix}_03_disparity.jpg")
    print(f"   - Depth map: {output_prefix}_04_depth.jpg")
    print(f"   - Comparison: {output_prefix}_05_comparison.jpg")


def main():
    print("\n" + "="*60)
    print("🔬 Stereo Depth Estimation Test")
    print("="*60)
    print("Method: OpenCV StereoSGBM (Classical)")
    print("Purpose: Understand depth pipeline before testing ESS")
    print("="*60)

    # Parse arguments
    if len(sys.argv) < 3:
        print("\nUsage: python3 test_stereo_depth.py <left_image> <right_image> [calib_file]")
        print("\nExample:")
        print("  python3 test_stereo_depth.py test_left_20251106_174522.jpg test_right_20251106_174522.jpg")
        sys.exit(1)

    left_path = sys.argv[1]
    right_path = sys.argv[2]
    calib_file = sys.argv[3] if len(sys.argv) > 3 else "stereo_calib.yaml"

    # Load images
    print(f"\n📷 Loading Images")
    print(f"{'='*60}")
    img_left = cv2.imread(left_path)
    img_right = cv2.imread(right_path)

    if img_left is None or img_right is None:
        print("❌ Failed to load images!")
        print(f"   Left: {left_path}")
        print(f"   Right: {right_path}")
        sys.exit(1)

    print(f"✅ Images loaded!")
    print(f"   Left: {left_path} ({img_left.shape[1]}x{img_left.shape[0]})")
    print(f"   Right: {right_path} ({img_right.shape[1]}x{img_right.shape[0]})")

    # Load calibration
    calib = load_calibration(calib_file)
    if calib is None:
        sys.exit(1)

    # Rectify images
    rect_left, rect_right = rectify_images(img_left, img_right, calib)

    # Compute disparity
    disparity = compute_disparity(rect_left, rect_right)

    # Convert to depth
    depth = disparity_to_depth(disparity, calib)

    # Visualize
    output_prefix = "stereo_depth_test"
    visualize_results(img_left, img_right, rect_left, rect_right,
                     disparity, depth, output_prefix)

    print(f"\n{'='*60}")
    print("✅ Stereo Depth Test Complete!")
    print(f"{'='*60}")
    print("\n📝 Key Findings:")
    print("   1. Check rectified images - epipolar lines should be horizontal")
    print("   2. Check disparity map - should show depth structure")
    print("   3. Check depth map - colors indicate distance (blue=far, red=close)")
    print("\n💡 Next Steps:")
    print("   1. Tomorrow: Re-calibrate camera")
    print("   2. Tomorrow: Test Isaac ROS ESS (DNN-based)")
    print("   3. Compare: StereoSGBM vs ESS performance")
    print("   4. Integrate: YOLO11n detection + depth → 3D coordinates")
    print("")


if __name__ == "__main__":
    main()
