#!/usr/bin/env python3
"""
Debug depth accuracy issues
Helps identify if problem is:
1. Pattern spacing
2. Stereo parameters
3. Measurement method
"""

import yaml
import numpy as np

def analyze_calibration():
    """Analyze calibration parameters"""
    print("=" * 70)
    print("Depth Accuracy Debug Tool")
    print("=" * 70)

    # Load calibration
    try:
        with open('stereo_calib.yaml', 'r') as f:
            calib = yaml.safe_load(f)
    except FileNotFoundError:
        print("ERROR: stereo_calib.yaml not found!")
        return

    baseline = calib['baseline_mm']
    Q = np.array(calib['stereo']['Q_matrix'])

    # Extract focal length from Q matrix
    focal_length = abs(Q[2, 3])

    print(f"\n📐 Calibration Parameters:")
    print(f"   Baseline: {baseline:.2f} mm ({baseline/10:.1f} cm)")
    print(f"   Focal length: {focal_length:.2f} pixels")
    print(f"   Image size: {calib['image_width']}x{calib['image_height']}")

    # Camera matrix
    K_left = np.array(calib['left_camera']['camera_matrix'])
    fx = K_left[0, 0]
    fy = K_left[1, 1]

    print(f"\n📷 Left Camera Intrinsics:")
    print(f"   fx: {fx:.2f} pixels")
    print(f"   fy: {fy:.2f} pixels")

    # Theoretical depth calculation
    print("\n" + "=" * 70)
    print("Depth Formula: depth = (baseline * focal_length) / disparity")
    print("=" * 70)

    # Simulate different real distances
    real_distances_cm = [30, 32, 40, 50, 60]

    print("\n🔍 Expected Disparity for Different Distances:")
    print(f"   (Using: baseline={baseline:.2f}mm, focal={focal_length:.2f}px)")
    print()

    for real_dist_cm in real_distances_cm:
        real_dist_mm = real_dist_cm * 10

        # Calculate expected disparity
        expected_disparity = (baseline * focal_length) / real_dist_mm

        print(f"   {real_dist_cm} cm → disparity = {expected_disparity:.2f} pixels")

    # Analyze the reported problem
    print("\n" + "=" * 70)
    print("Problem Analysis:")
    print("=" * 70)
    print(f"\n📊 Your Test Results:")
    print(f"   Real distance: 32 cm")
    print(f"   Measured distance: 60 cm (avg)")
    print(f"   Error: +28 cm (+87.5%)")
    print(f"   Error ratio: 60/32 = 1.875x")

    # Calculate what disparity was used
    measured_dist_mm = 600  # 60 cm
    real_dist_mm = 320  # 32 cm

    # Disparity that gave 60cm reading
    actual_disparity = (baseline * focal_length) / measured_dist_mm

    # Disparity needed for 32cm
    needed_disparity = (baseline * focal_length) / real_dist_mm

    print(f"\n🔍 Disparity Analysis:")
    print(f"   Disparity measured: {actual_disparity:.2f} pixels")
    print(f"   Disparity needed for 32cm: {needed_disparity:.2f} pixels")
    print(f"   Ratio: {needed_disparity / actual_disparity:.3f}x")

    # Possible causes
    print("\n" + "=" * 70)
    print("🤔 Possible Causes:")
    print("=" * 70)

    print("\n1️⃣  PATTERN SPACING ERROR (Most Likely)")
    print("   If you measured HORIZONTAL instead of DIAGONAL:")

    # For asymmetric circles: horizontal ≈ 1.44 × diagonal
    diagonal_18mm = 18.0
    horizontal_26mm = 26.0
    ratio_h_to_d = horizontal_26mm / diagonal_18mm

    print(f"   - Diagonal spacing: {diagonal_18mm} mm (should use this)")
    print(f"   - Horizontal spacing: {horizontal_26mm} mm")
    print(f"   - Ratio: {ratio_h_to_d:.3f}x")
    print(f"   - But error is 1.875x ❌ (doesn't match)")

    # Check if spacing is actually different
    print(f"\n   If true spacing was different:")
    true_spacing_if_error = diagonal_18mm * 1.875
    print(f"   - To get 1.875x error, true spacing = {true_spacing_if_error:.1f} mm")
    print(f"   - This is too large! ❌")

    # Inverse calculation
    true_spacing_inverse = diagonal_18mm / 1.875
    print(f"\n   OR if baseline calculation used wrong spacing:")
    print(f"   - True spacing might be: {true_spacing_inverse:.1f} mm")
    print(f"   - This is too small! ❌")

    print("\n2️⃣  DISTANCE MEASUREMENT ERROR")
    print("   How did you measure 32cm?")
    print("   - From camera LENS to pattern board? ✅ (Correct)")
    print("   - From camera MOUNT/BODY? ❌ (Wrong!)")
    print("   - From camera SENSOR? ❌ (Wrong!)")
    print()
    print("   📏 Correct method:")
    print("      Measure from FRONT of LENS to PATTERN SURFACE")

    print("\n3️⃣  RECTIFICATION OR SGBM ISSUE")
    print("   - Baseline looks correct: {:.2f}mm".format(baseline))
    print("   - Focal length: {:.2f}px".format(focal_length))
    print("   - But disparity might be wrong")

    # Recommendations
    print("\n" + "=" * 70)
    print("🔧 Recommended Actions:")
    print("=" * 70)

    print("\n1. RE-MEASURE pattern spacing with caliper:")
    print("   - Measure DIAGONAL between circle centers")
    print("   - Example: Row 0 Col 0 → Row 0 Col 1 (diagonal)")
    print("   - NOT horizontal distance!")
    print()
    print("   Pattern layout:")
    print("   Row 0: •       •       •")
    print("              ╲")
    print("   Row 1:   •   ╲   •       •")
    print("               ╲")
    print("            diagonal")
    print()

    print("2. RE-MEASURE 32cm distance:")
    print("   - Use ruler or tape measure")
    print("   - From camera LENS FRONT to pattern board")
    print("   - Make sure board is perpendicular to camera")

    print("\n3. Try different test distances:")
    print("   - 25cm, 30cm, 35cm, 40cm, 50cm")
    print("   - Record actual vs measured for each")
    print("   - Look for consistent error ratio")

    print("\n4. Check pattern flatness:")
    print("   - Is pattern board perfectly flat?")
    print("   - Any warping or bending?")

    print("\n" + "=" * 70)
    print("Next step: Answer these questions, then we can fix it!")
    print("=" * 70)

if __name__ == '__main__':
    analyze_calibration()
