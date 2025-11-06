#!/usr/bin/env python3
"""
Demo: Auto Preprocessing for Stereo Depth
Shows how to use auto_preprocess in real pipeline
"""

import cv2
import sys
from stereo_preprocess import auto_preprocess, auto_preprocess_stereo, get_preset


def demo_simple():
    """Simple usage example"""
    print("\n" + "="*60)
    print("📝 Demo 1: Simple Usage")
    print("="*60)

    left_path = "test_left_20251106_174522.jpg"
    right_path = "test_right_20251106_174522.jpg"

    # Load images
    left = cv2.imread(left_path)
    right = cv2.imread(right_path)

    if left is None or right is None:
        print("❌ Images not found!")
        return

    print(f"✅ Loaded images: {left.shape}")

    # Auto preprocessing (simple - best for cheap cameras)
    print("\n▶ Method 1: Simple auto_preprocess")
    left_enhanced = auto_preprocess(left, method="denoise")
    right_enhanced = auto_preprocess(right, method="denoise")
    print(f"   Result: {left_enhanced.shape}")

    # Save results
    cv2.imwrite("demo_left_enhanced.jpg", left_enhanced)
    cv2.imwrite("demo_right_enhanced.jpg", right_enhanced)
    print("   Saved: demo_left_enhanced.jpg, demo_right_enhanced.jpg")


def demo_stereo_pair():
    """Stereo pair usage"""
    print("\n" + "="*60)
    print("📝 Demo 2: Stereo Pair Helper")
    print("="*60)

    left = cv2.imread("test_left_20251106_174522.jpg")
    right = cv2.imread("test_right_20251106_174522.jpg")

    if left is None or right is None:
        return

    # Process stereo pair in one function
    print("\n▶ Method 2: auto_preprocess_stereo")
    left_enh, right_enh = auto_preprocess_stereo(left, right, method="denoise")
    print(f"   Result: {left_enh.shape}, {right_enh.shape}")


def demo_presets():
    """Using presets"""
    print("\n" + "="*60)
    print("📝 Demo 3: Using Presets")
    print("="*60)

    left = cv2.imread("test_left_20251106_174522.jpg")
    if left is None:
        return

    print("\n▶ Method 3: Using presets")

    # For cheap camera (IMX219 1800 THB)
    print("   Testing preset: 'cheap_camera'")
    settings = get_preset("cheap_camera")
    print(f"   Settings: {settings}")
    enhanced = auto_preprocess(left, **settings)
    print(f"   Result: {enhanced.shape}")


def demo_comparison():
    """Compare before/after"""
    print("\n" + "="*60)
    print("📝 Demo 4: Before/After Comparison")
    print("="*60)

    left = cv2.imread("test_left_20251106_174522.jpg")
    if left is None:
        return

    # Before
    if len(left.shape) == 3:
        before = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    else:
        before = left

    # After
    after = auto_preprocess(left, method="denoise")

    # Create side-by-side comparison
    comparison = cv2.hconcat([before, after])

    # Add labels
    cv2.putText(comparison, "BEFORE", (10, 50),
               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
    cv2.putText(comparison, "AFTER (denoise)", (before.shape[1] + 10, 50),
               cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)

    cv2.imwrite("demo_before_after.jpg", comparison)
    print("✅ Saved: demo_before_after.jpg")


def demo_pipeline():
    """Complete stereo depth pipeline with preprocessing"""
    print("\n" + "="*60)
    print("📝 Demo 5: Complete Pipeline")
    print("="*60)

    import yaml
    import numpy as np

    # Load images
    left = cv2.imread("test_left_20251106_174522.jpg")
    right = cv2.imread("test_right_20251106_174522.jpg")

    if left is None or right is None:
        return

    print("Step 1: Auto preprocessing")
    left_enh, right_enh = auto_preprocess_stereo(left, right, method="denoise")
    print(f"   ✅ Enhanced: {left_enh.shape}")

    print("\nStep 2: Load calibration")
    try:
        with open("stereo_calib.yaml") as f:
            calib = yaml.safe_load(f)
        print(f"   ✅ Baseline: {calib['baseline_mm']:.2f}mm")
    except:
        print("   ⚠️  Calibration file not found, skipping...")
        return

    print("\nStep 3: Rectify images")
    h, w = left.shape[:2]
    K1 = np.array(calib['left_camera']['camera_matrix'])
    K2 = np.array(calib['right_camera']['camera_matrix'])
    D1 = np.array(calib['left_camera']['distortion_coefficients'][0])
    D2 = np.array(calib['right_camera']['distortion_coefficients'][0])
    R1 = np.array(calib['left_camera']['rectification_matrix'])
    R2 = np.array(calib['right_camera']['rectification_matrix'])
    P1 = np.array(calib['left_camera']['projection_matrix'])
    P2 = np.array(calib['right_camera']['projection_matrix'])

    map1_left, map2_left = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    map1_right, map2_right = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)

    rect_left = cv2.remap(left_enh, map1_left, map2_left, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_enh, map1_right, map2_right, cv2.INTER_LINEAR)
    print(f"   ✅ Rectified: {rect_left.shape}")

    print("\nStep 4: Compute disparity")
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,
        blockSize=7,
        P1=8 * 3 * 7 ** 2,
        P2=32 * 3 * 7 ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=15,
        speckleWindowSize=150,
        speckleRange=2,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )
    disparity = stereo.compute(rect_left, rect_right).astype(np.float32) / 16.0
    valid_pct = (disparity > 0).sum() / disparity.size * 100
    print(f"   ✅ Disparity: valid pixels = {valid_pct:.1f}%")

    # Visualize
    disp_vis = disparity.copy()
    disp_vis[disp_vis < 0] = 0
    if disp_vis.max() > 0:
        disp_vis = (disp_vis / disp_vis.max() * 255).astype(np.uint8)
    disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

    # Save
    result = cv2.hconcat([cv2.cvtColor(rect_left, cv2.COLOR_GRAY2BGR), disp_color])
    cv2.imwrite("demo_pipeline_result.jpg", result)
    print(f"\n✅ Saved: demo_pipeline_result.jpg")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("🎬 Auto Preprocessing Demo")
    print("="*60)
    print("Optimized for cheap cameras (IMX219 1800 THB)")
    print("="*60)

    demo_simple()
    demo_stereo_pair()
    demo_presets()
    demo_comparison()
    demo_pipeline()

    print("\n" + "="*60)
    print("✅ All Demos Complete!")
    print("="*60)
    print("\n📊 Results:")
    print("   - demo_left_enhanced.jpg")
    print("   - demo_right_enhanced.jpg")
    print("   - demo_before_after.jpg")
    print("   - demo_pipeline_result.jpg")
    print("\n💡 To use in your code:")
    print("   from stereo_preprocess import auto_preprocess")
    print("   enhanced = auto_preprocess(image, method='denoise')")
    print("="*60 + "\n")
