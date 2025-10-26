#!/usr/bin/env python3
"""
Stereo Calibration Script for SYNTHETIC DATA
Wrapper around stereo_calibration.py that uses synthetic images

Usage:
    python3 stereo_calibration_synthetic.py

This script runs stereo_calibration.py on synthetic data and compares
the results with ground_truth.yaml
"""

import yaml
import numpy as np
from stereo_calibration import calibrate_stereo_asymmetric_circles


def load_ground_truth(file_path='ground_truth.yaml'):
    """Load ground truth parameters from YAML"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def compare_results(calib_results, ground_truth):
    """
    Compare calibration results with ground truth

    Args:
        calib_results: Results from stereo_calibration.py
        ground_truth: Ground truth parameters from YAML

    Returns:
        dict: Comparison metrics
    """
    print("\n" + "=" * 70)
    print("COMPARISON: Calibration Results vs Ground Truth")
    print("=" * 70)

    # Baseline comparison
    gt_baseline = ground_truth['baseline_mm']
    cal_baseline = calib_results['baseline_mm']
    baseline_error = abs(cal_baseline - gt_baseline)
    baseline_error_pct = (baseline_error / gt_baseline) * 100

    print(f"\n1. BASELINE:")
    print(f"   Ground Truth:  {gt_baseline:.4f} mm")
    print(f"   Calibrated:    {cal_baseline:.4f} mm")
    print(f"   Error:         {baseline_error:.4f} mm ({baseline_error_pct:.2f}%)")

    if baseline_error_pct < 1.0:
        print(f"   Status:        ✓ EXCELLENT (< 1%)")
    elif baseline_error_pct < 5.0:
        print(f"   Status:        ✓ GOOD (< 5%)")
    elif baseline_error_pct < 10.0:
        print(f"   Status:        ⚠ ACCEPTABLE (< 10%)")
    else:
        print(f"   Status:        ✗ POOR (≥ 10%)")

    # Focal length comparison (average of left and right)
    gt_focal = ground_truth['camera_matrix'][0][0]
    cal_focal_left = calib_results['left_camera']['camera_matrix'][0][0]
    cal_focal_right = calib_results['right_camera']['camera_matrix'][0][0]
    cal_focal_avg = (cal_focal_left + cal_focal_right) / 2

    focal_error = abs(cal_focal_avg - gt_focal)
    focal_error_pct = (focal_error / gt_focal) * 100

    print(f"\n2. FOCAL LENGTH:")
    print(f"   Ground Truth:  {gt_focal:.2f} px")
    print(f"   Calibrated:    {cal_focal_avg:.2f} px (avg)")
    print(f"   Error:         {focal_error:.2f} px ({focal_error_pct:.2f}%)")

    if focal_error_pct < 1.0:
        print(f"   Status:        ✓ EXCELLENT (< 1%)")
    elif focal_error_pct < 5.0:
        print(f"   Status:        ✓ GOOD (< 5%)")
    else:
        print(f"   Status:        ⚠ ACCEPTABLE (≥ 5%)")

    # Principal point comparison
    gt_cx = ground_truth['camera_matrix'][0][2]
    gt_cy = ground_truth['camera_matrix'][1][2]
    cal_cx_left = calib_results['left_camera']['camera_matrix'][0][2]
    cal_cy_left = calib_results['left_camera']['camera_matrix'][1][2]

    cx_error = abs(cal_cx_left - gt_cx)
    cy_error = abs(cal_cy_left - gt_cy)

    print(f"\n3. PRINCIPAL POINT (Left Camera):")
    print(f"   Ground Truth:  ({gt_cx:.2f}, {gt_cy:.2f})")
    print(f"   Calibrated:    ({cal_cx_left:.2f}, {cal_cy_left:.2f})")
    print(f"   Error:         ({cx_error:.2f}, {cy_error:.2f}) px")

    # RMS error
    rms_error = calib_results['stereo_rms_error']
    print(f"\n4. STEREO RMS ERROR:")
    print(f"   Value:         {rms_error:.4f} pixels")

    if rms_error < 0.5:
        print(f"   Status:        ✓ EXCELLENT (< 0.5) - Expected for perfect data!")
    elif rms_error < 1.0:
        print(f"   Status:        ✓ GOOD (< 1.0)")
    elif rms_error < 2.0:
        print(f"   Status:        ⚠ ACCEPTABLE (< 2.0)")
    else:
        print(f"   Status:        ✗ POOR (≥ 2.0)")

    print("\n" + "=" * 70)
    print("OVERALL ASSESSMENT:")
    print("=" * 70)

    # Overall pass/fail
    all_good = (
        baseline_error_pct < 5.0 and
        focal_error_pct < 5.0 and
        rms_error < 1.0
    )

    if all_good:
        print("✓ PASS - stereo_calibration.py works correctly!")
        print("  → Problem with real data is likely:")
        print("    - Pattern spacing incorrect (16mm vs 18mm)")
        print("    - Focus issues")
        print("    - Lens distortion")
        print("    - Pattern print quality")
    else:
        print("✗ FAIL - stereo_calibration.py may have issues!")
        print("  → Possible problems:")
        print("    - Object points generation incorrect")
        print("    - Pattern spacing formula wrong")
        print("    - Calibration algorithm parameters")

    print("=" * 70)

    # Return comparison metrics
    return {
        'baseline_error_mm': baseline_error,
        'baseline_error_pct': baseline_error_pct,
        'focal_error_px': focal_error,
        'focal_error_pct': focal_error_pct,
        'cx_error_px': cx_error,
        'cy_error_px': cy_error,
        'rms_error': rms_error,
        'pass': all_good
    }


def main():
    """Run calibration on synthetic data and compare with ground truth"""

    print("=" * 70)
    print("Stereo Calibration - SYNTHETIC DATA TEST")
    print("=" * 70)
    print()

    # Run calibration on synthetic images
    print("Running stereo calibration on synthetic data...")
    print("(This will take a moment...)")
    print()

    calib_results = calibrate_stereo_asymmetric_circles(
        images_left_path='calib_images/synthetic/left/*.jpg',
        images_right_path='calib_images/synthetic/right/*.jpg',
        pattern_rows=5,
        pattern_cols=6,
        spacing_mm=18.0,
        output_file='stereo_calib_synthetic.yaml'
    )

    if calib_results is None:
        print("\n✗ Calibration failed!")
        return

    print("\n✓ Calibration completed!")
    print(f"   Results saved to: stereo_calib_synthetic.yaml")

    # Load ground truth
    print("\nLoading ground truth...")
    ground_truth = load_ground_truth('ground_truth.yaml')

    # Compare results
    comparison = compare_results(calib_results, ground_truth)

    # Save comparison report
    comparison_report = {
        'test_date': calib_results.get('test_date', 'unknown'),
        'ground_truth': {
            'baseline_mm': ground_truth['baseline_mm'],
            'focal_length_px': ground_truth['camera_matrix'][0][0],
            'principal_point': [
                ground_truth['camera_matrix'][0][2],
                ground_truth['camera_matrix'][1][2]
            ]
        },
        'calibrated': {
            'baseline_mm': calib_results['baseline_mm'],
            'focal_length_px': (
                calib_results['left_camera']['camera_matrix'][0][0] +
                calib_results['right_camera']['camera_matrix'][0][0]
            ) / 2,
            'stereo_rms_error': calib_results['stereo_rms_error']
        },
        'comparison': comparison
    }

    with open('calibration_comparison_report.yaml', 'w') as f:
        yaml.dump(comparison_report, f, default_flow_style=False, sort_keys=False)

    print(f"\n✓ Comparison report saved to: calibration_comparison_report.yaml")

    print("\n" + "=" * 70)
    print("TEST COMPLETE")
    print("=" * 70)


if __name__ == '__main__':
    main()
