#!/usr/bin/env python3
"""
🌶️ Pepper Depth Testing Tool - Adaptive Percentile Method
- Automatically adjusts percentile based on coverage
- Low coverage (shiny) → aggressive (5th percentile)
- High coverage (normal) → conservative (median)
- Best for mixed pepper types (shiny + normal)
"""

import cv2
import numpy as np
import yaml
import time


def build_gstreamer_pipeline(sensor_id, width=1280, height=720, framerate=15):
    """Use same resolution as calibration for correct depth computation"""
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, '
        f'format=(string)NV12, framerate=(fraction){framerate}/1 ! '
        f'nvvidconv flip-method=0 ! '
        f'video/x-raw, format=(string)BGRx ! '
        f'videoconvert ! '
        f'video/x-raw, format=(string)BGR ! '
        f'appsink'
    )


def load_calibration():
    with open('stereo_calib.yaml', 'r') as f:
        calib = yaml.safe_load(f)
    maps = np.load('rectification_maps.npz')
    return calib, maps


def compute_depth(left_gray, right_gray, baseline, focal):
    """Fast depth computation without heavy filtering"""
    window_size = 5
    min_disp = 0
    num_disp = 512  # 16 * 32

    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=8 * 3 * window_size ** 2,
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=2,
        uniquenessRatio=12,
        speckleWindowSize=120,
        speckleRange=16,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    # Compute disparity
    disparity = stereo.compute(left_gray, right_gray)
    disparity_float = disparity.astype(np.float32) / 16.0

    # Valid mask
    valid_mask = (disparity_float > min_disp) & (disparity_float < num_disp)

    # Calculate depth
    depth_map = np.zeros_like(disparity_float)
    depth_map[valid_mask] = (baseline * focal) / disparity_float[valid_mask]

    # Filter realistic depths (15-120cm)
    realistic_mask = (depth_map >= 150) & (depth_map <= 1200)
    final_mask = valid_mask & realistic_mask

    return depth_map, final_mask


def adaptive_depth_estimation(valid_depths, coverage):
    """
    Adaptive depth estimation based on coverage

    Strategy:
    - Low coverage (shiny surface) → aggressive percentile (5th)
    - Medium coverage → standard (10th percentile)
    - High coverage → conservative (median)

    Args:
        valid_depths: Array of valid depth values
        coverage: Coverage percentage (0-100)

    Returns:
        Dictionary with adaptive depth and reasoning
    """
    # Calculate all candidate methods
    methods = {
        'min': np.min(valid_depths),
        'percentile_05': np.percentile(valid_depths, 5),
        'percentile_10': np.percentile(valid_depths, 10),
        'percentile_15': np.percentile(valid_depths, 15),
        'median': np.median(valid_depths),
        'mean': np.mean(valid_depths),
    }

    # Adaptive selection based on coverage
    if coverage < 15:
        # Very low coverage - very shiny surface
        selected_depth = methods['percentile_05']
        method_name = "5th percentile"
        reason = "Very low coverage (<15%) - shiny surface detected"
        confidence = "low"

    elif coverage < 25:
        # Low coverage - shiny surface
        selected_depth = methods['percentile_05']
        method_name = "5th percentile"
        reason = "Low coverage (<25%) - shiny surface likely"
        confidence = "medium"

    elif coverage < 35:
        # Medium-low coverage - slightly shiny
        selected_depth = methods['percentile_10']
        method_name = "10th percentile"
        reason = "Medium-low coverage (25-35%) - balanced approach"
        confidence = "good"

    elif coverage < 50:
        # Medium coverage - normal surface
        selected_depth = methods['percentile_10']
        method_name = "10th percentile"
        reason = "Medium coverage (35-50%) - standard approach"
        confidence = "good"

    else:
        # High coverage - good texture
        selected_depth = methods['percentile_15']
        method_name = "15th percentile"
        reason = "High coverage (>50%) - good texture, conservative"
        confidence = "high"

    return {
        'depth': selected_depth,
        'method': method_name,
        'reason': reason,
        'confidence': confidence,
        'coverage': coverage,
        'all_methods': methods
    }


def click_handler(event, x, y, flags, param):
    """Interactive depth measurement"""
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_map, final_mask, clicks, img_offset = param
        h, w = depth_map.shape

        # Adjust x for combined display
        if x >= img_offset:
            x = x - img_offset

        if 0 <= y < h and 0 <= x < w:
            if final_mask[y, x]:
                depth = depth_map[y, x]
                clicks.append(depth)
                print(f"\n📍 Click {len(clicks)}: ({x}, {y})")
                print(f"   Depth: {depth:.1f} mm ({depth/10:.1f} cm)")

                if len(clicks) >= 2:
                    values = np.array(clicks)
                    print(f"\n   Last {len(clicks)} measurements:")
                    print(f"   Average: {values.mean():.1f} mm ({values.mean()/10:.1f} cm)")
                    print(f"   Std Dev: {values.std():.2f} mm ({values.std()/10:.2f} cm)")
                    print(f"   Range: {values.min():.1f} - {values.max():.1f} mm")
                    print(f"   Median: {np.median(values):.1f} mm")
            else:
                print(f"\n⚠️  No valid depth at ({x}, {y})")


def main():
    print("=" * 80)
    print("🌶️  PEPPER DEPTH TESTING TOOL - ADAPTIVE METHOD")
    print("=" * 80)
    print("\n✨ Features:")
    print("  • Adaptive percentile based on coverage")
    print("  • Automatic surface type detection (shiny vs normal)")
    print("  • Best for mixed pepper types")
    print("\n🎯 Strategy:")
    print("  Coverage < 25%:  Use 5th percentile (shiny surface)")
    print("  Coverage 25-35%: Use 10th percentile (balanced)")
    print("  Coverage > 35%:  Use 10-15th percentile (conservative)")
    print("=" * 80)

    # Load calibration
    print("\n📐 Loading calibration...")
    calib, maps = load_calibration()
    baseline = calib['baseline_mm']
    Q = np.array(calib['stereo']['Q_matrix'])
    focal_length = abs(Q[2, 3])

    print(f"  Baseline: {baseline:.2f} mm")
    print(f"  Focal: {focal_length:.2f} px")

    width = calib['image_width']
    height = calib['image_height']
    print(f"  Working resolution: {width}x{height} ✓")

    # Use original calibration maps
    map_left_x = maps['map_left_x']
    map_left_y = maps['map_left_y']
    map_right_x = maps['map_right_x']
    map_right_y = maps['map_right_y']

    # Open cameras
    print("\n📷 Opening cameras...")
    left_pipeline = build_gstreamer_pipeline(0, width, height)
    right_pipeline = build_gstreamer_pipeline(1, width, height)

    left_cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
    if not left_cap.isOpened():
        print("❌ ERROR: Left camera failed")
        return

    time.sleep(2)

    right_cap = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)
    if not right_cap.isOpened():
        print("❌ ERROR: Right camera failed")
        left_cap.release()
        return

    print("  ✓ Cameras ready")

    # CLAHE
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))

    print("\n" + "=" * 80)
    print("⌨️  CONTROLS:")
    print("  SPACE  : Capture and compute depth")
    print("  Click  : Measure depth at point")
    print("  'r'    : Reset measurements")
    print("  's'    : Save current result")
    print("  'q'    : Quit")
    print("=" * 80)
    print("\n👉 Press SPACE to start...")

    cv2.namedWindow('View', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('View', 1280, 480)

    depth_map = None
    final_mask = None
    clicks = []
    capture_count = 0

    while True:
        # Live preview
        ret_left, frame_left = left_cap.read()
        ret_right, frame_right = right_cap.read()

        if not ret_left or not ret_right:
            break

        # Show preview
        preview = frame_left.copy()
        cv2.putText(preview, "Press SPACE to capture", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(preview, f"Captures: {capture_count}", (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow('View', preview)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):  # SPACE - capture
            print("\n" + "=" * 80)
            print(f"📸 CAPTURE #{capture_count + 1}")
            print("=" * 80)

            # Rectify
            t0 = time.time()
            rectified_left = cv2.remap(frame_left, map_left_x, map_left_y,
                                       cv2.INTER_LINEAR)
            rectified_right = cv2.remap(frame_right, map_right_x, map_right_y,
                                        cv2.INTER_LINEAR)
            t1 = time.time()
            print(f"  Rectification: {(t1-t0)*1000:.0f} ms")

            # Grayscale + CLAHE
            gray_left = cv2.cvtColor(rectified_left, cv2.COLOR_BGR2GRAY)
            gray_right = cv2.cvtColor(rectified_right, cv2.COLOR_BGR2GRAY)
            enhanced_left = clahe.apply(gray_left)
            enhanced_right = clahe.apply(gray_right)

            # Compute depth
            t2 = time.time()
            depth_map, final_mask = compute_depth(
                enhanced_left, enhanced_right, baseline, focal_length
            )
            t3 = time.time()
            print(f"  Depth computation: {(t3-t2)*1000:.0f} ms")
            print(f"  Total: {(t3-t0)*1000:.0f} ms")

            # Coverage stats
            coverage = np.sum(final_mask) / (width * height) * 100
            left_cov = np.sum(final_mask[:, :width//2]) / (height * width // 2) * 100
            right_cov = np.sum(final_mask[:, width//2:]) / (height * width // 2) * 100

            print(f"\n📊 Coverage:")
            print(f"  Overall: {coverage:.1f}%")
            print(f"  Left half: {left_cov:.1f}%")
            print(f"  Right half: {right_cov:.1f}%")

            # Adaptive depth estimation
            if final_mask.any():
                valid_depths = depth_map[final_mask]
                result = adaptive_depth_estimation(valid_depths, coverage)

                print(f"\n🤖 ADAPTIVE DEPTH ESTIMATION:")
                print(f"  Selected Method: {result['method']}")
                print(f"  📏 Depth: {result['depth']:.1f} mm ({result['depth']/10:.1f} cm) ⭐")
                print(f"  Reason: {result['reason']}")
                print(f"  Confidence: {result['confidence']}")

                print(f"\n📊 All Methods (for comparison):")
                all_methods = result['all_methods']
                print(f"  Min:              {all_methods['min']:.1f} mm ({all_methods['min']/10:.1f} cm)")
                print(f"  5th percentile:   {all_methods['percentile_05']:.1f} mm ({all_methods['percentile_05']/10:.1f} cm)")
                print(f"  10th percentile:  {all_methods['percentile_10']:.1f} mm ({all_methods['percentile_10']/10:.1f} cm)")
                print(f"  15th percentile:  {all_methods['percentile_15']:.1f} mm ({all_methods['percentile_15']/10:.1f} cm)")
                print(f"  Median:           {all_methods['median']:.1f} mm ({all_methods['median']/10:.1f} cm)")
                print(f"  Mean:             {all_methods['mean']:.1f} mm ({all_methods['mean']/10:.1f} cm)")

                print(f"\n💡 Comparison:")
                diff_median = all_methods['median'] - result['depth']
                diff_mean = all_methods['mean'] - result['depth']
                print(f"  Median - Adaptive: {diff_median:.1f} mm ({diff_median/10:.1f} cm)")
                print(f"  Mean - Adaptive:   {diff_mean:.1f} mm ({diff_mean/10:.1f} cm)")

                # Visualize depth
                depth_vis = depth_map.copy()
                depth_vis[~final_mask] = 0
                depth_vis = np.clip(depth_vis, 150, 1200)
                depth_vis = ((depth_vis - 150) / 1050 * 255).astype(np.uint8)
                depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                depth_colored[~final_mask] = [0, 0, 0]

                # Add overlay
                cv2.putText(depth_colored, f"Coverage: {coverage:.1f}%",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(depth_colored, f"{result['method']}: {result['depth']/10:.1f}cm",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(depth_colored, f"Confidence: {result['confidence']}",
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

                # Combine
                combined = np.hstack([rectified_left, depth_colored])
                cv2.imshow('View', combined)

                # Reset clicks
                clicks = []

                # Set mouse callback
                cv2.setMouseCallback('View', click_handler,
                                   (depth_map, final_mask, clicks, width))

                capture_count += 1

                print("\n👆 Click on the image to measure depth")

        elif key == ord('r'):  # Reset clicks
            clicks = []
            print("\n🔄 Measurements reset")

        elif key == ord('s') and depth_map is not None:  # Save
            timestamp = int(time.time())
            cv2.imwrite(f"adaptive_{timestamp}_left.jpg", rectified_left)
            cv2.imwrite(f"adaptive_{timestamp}_depth.jpg", depth_colored)
            print(f"\n💾 Saved: adaptive_{timestamp}_*.jpg")

    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 80)
    print("✓ Session complete")
    print("=" * 80)


if __name__ == '__main__':
    main()
