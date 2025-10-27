#!/usr/bin/env python3
"""
Balanced depth map - compromise between accuracy and coverage
For real-world pepper sorting use case
"""

import cv2
import numpy as np
import yaml


def build_gstreamer_pipeline(sensor_id, width=1280, height=720, framerate=30):
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


def click_distance(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        depth_map, baseline, focal = param
        if 0 <= y < depth_map.shape[0] and 0 <= x < depth_map.shape[1]:
            depth = depth_map[y, x]
            if depth > 0:
                print(f"\n📍 Point ({x}, {y}):")
                print(f"   Depth: {depth:.1f} mm ({depth/10:.1f} cm)")
            else:
                print(f"\n⚠️  No depth data at ({x}, {y})")


def main():
    print("=" * 70)
    print("BALANCED Stereo Depth Map")
    print("Balance: Accuracy vs Coverage for Real-World Use")
    print("=" * 70)

    # Load calibration
    calib, maps = load_calibration()
    baseline = calib['baseline_mm']
    Q = np.array(calib['stereo']['Q_matrix'])
    focal_length = abs(Q[2, 3])

    print(f"\nCalibration:")
    print(f"  Baseline: {baseline:.2f} mm")
    print(f"  Focal: {focal_length:.2f} px")

    map_left_x = maps['map_left_x']
    map_left_y = maps['map_left_y']
    map_right_x = maps['map_right_x']
    map_right_y = maps['map_right_y']

    width = calib['image_width']
    height = calib['image_height']

    # Cameras
    left_pipeline = build_gstreamer_pipeline(0, width, height)
    right_pipeline = build_gstreamer_pipeline(1, width, height)

    left_cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
    if not left_cap.isOpened():
        print("ERROR: Left camera")
        return

    import time
    time.sleep(2)

    right_cap = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)
    if not right_cap.isOpened():
        print("ERROR: Right camera")
        left_cap.release()
        return

    print("✓ Cameras ready")

    # CLAHE
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))

    # BALANCED StereoSGBM parameters
    print("\nBALANCED StereoSGBM Parameters:")
    print("  → Moderate strictness for real-world objects")

    window_size = 5
    min_disp = 0
    num_disp = 16 * 32  # 512

    left_matcher = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=8 * 3 * window_size ** 2,
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=2,           # Relaxed from 1
        uniquenessRatio=12,        # Balanced (was 15)
        speckleWindowSize=120,     # Moderate filtering
        speckleRange=16,           # Relaxed from 2
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    # WLS filter - moderate
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(9000)     # Moderate
    wls_filter.setSigmaColor(1.3)  # Moderate

    print("  uniquenessRatio: 12 (balanced)")
    print("  speckleRange: 16 (relaxed)")
    print("  WLS lambda: 9000")

    print("\nControls:")
    print("  Click: Measure depth")
    print("  'q': Quit")
    print("  's': Save frame")
    print("=" * 70)

    cv2.namedWindow('Depth Map', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Coverage', cv2.WINDOW_NORMAL)

    frame_count = 0

    while True:
        ret_left, frame_left = left_cap.read()
        ret_right, frame_right = right_cap.read()

        if not ret_left or not ret_right:
            break

        # Rectify
        rectified_left = cv2.remap(frame_left, map_left_x, map_left_y,
                                   cv2.INTER_LINEAR)
        rectified_right = cv2.remap(frame_right, map_right_x, map_right_y,
                                    cv2.INTER_LINEAR)

        # Grayscale + CLAHE
        gray_left = cv2.cvtColor(rectified_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rectified_right, cv2.COLOR_BGR2GRAY)

        enhanced_left = clahe.apply(gray_left)
        enhanced_right = clahe.apply(gray_right)

        # Compute disparity
        disparity_left = left_matcher.compute(enhanced_left, enhanced_right)
        disparity_right = right_matcher.compute(enhanced_right, enhanced_left)

        # WLS filter
        disparity_filtered = wls_filter.filter(disparity_left, enhanced_left,
                                               None, disparity_right)

        # Convert to float
        disparity_float = disparity_filtered.astype(np.float32) / 16.0

        # Valid mask
        valid_mask = (disparity_float > min_disp) & (disparity_float < num_disp)

        # Calculate depth
        depth_map = np.zeros_like(disparity_float)
        depth_map[valid_mask] = (baseline * focal_length) / disparity_float[valid_mask]

        # Filter realistic depths (15-120cm)
        realistic_mask = (depth_map >= 150) & (depth_map <= 1200)
        final_mask = valid_mask & realistic_mask

        # Stats
        coverage = np.sum(final_mask) / (width * height) * 100

        # Visualize depth
        depth_vis = depth_map.copy()
        depth_vis[~final_mask] = 0
        depth_vis = np.clip(depth_vis, 150, 1200)
        depth_vis = ((depth_vis - 150) / 1050 * 255).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        depth_colored[~final_mask] = [0, 0, 0]

        # Add info
        cv2.putText(depth_colored, f"Coverage: {coverage:.1f}%",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        if final_mask.any():
            avg_depth = depth_map[final_mask].mean()
            cv2.putText(depth_colored, f"Avg: {avg_depth:.1f}mm",
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Coverage visualization
        mask_vis = np.zeros((height, width, 3), dtype=np.uint8)
        mask_vis[final_mask] = [0, 255, 0]

        # Grid
        for i in range(0, width, width // 10):
            cv2.line(mask_vis, (i, 0), (i, height), (128, 128, 128), 1)
        for i in range(0, height, height // 6):
            cv2.line(mask_vis, (0, i), (width, i), (128, 128, 128), 1)

        # Display
        cv2.imshow('Depth Map', depth_colored)
        cv2.imshow('Coverage', mask_vis)

        # Mouse callback
        cv2.setMouseCallback('Depth Map', click_distance,
                           (depth_map, baseline, focal_length))

        # Stats every 30 frames
        if frame_count % 30 == 0:
            left_cov = np.sum(final_mask[:, :width//2]) / (height * width // 2) * 100
            right_cov = np.sum(final_mask[:, width//2:]) / (height * width // 2) * 100

            print(f"\n[Frame {frame_count}]")
            print(f"  Overall: {coverage:.1f}%")
            print(f"  Left: {left_cov:.1f}%  Right: {right_cov:.1f}%")

            if final_mask.any():
                print(f"  Depth: {depth_map[final_mask].min():.0f} - "
                      f"{depth_map[final_mask].max():.0f} mm")

        frame_count += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite(f"balanced_{frame_count:04d}_depth.jpg", depth_colored)
            cv2.imwrite(f"balanced_{frame_count:04d}_coverage.jpg", mask_vis)
            print(f"✓ Saved balanced_{frame_count:04d}_*.jpg")

    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
