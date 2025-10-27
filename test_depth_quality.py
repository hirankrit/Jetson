#!/usr/bin/env python3
"""
Test depth map quality and coverage
Shows:
1. Valid depth coverage (%)
2. Confidence map
3. Error distribution
"""

import cv2
import numpy as np
import yaml


def build_gstreamer_pipeline(sensor_id, width=1280, height=720, framerate=30):
    """Build GStreamer pipeline"""
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
    """Load stereo calibration"""
    with open('stereo_calib.yaml', 'r') as f:
        calib = yaml.safe_load(f)
    maps = np.load('rectification_maps.npz')
    return calib, maps


def main():
    print("=" * 70)
    print("Depth Map Quality Analysis")
    print("=" * 70)

    # Load calibration
    calib, maps = load_calibration()
    baseline = calib['baseline_mm']
    Q = np.array(calib['stereo']['Q_matrix'])
    focal_length = abs(Q[2, 3])

    map_left_x = maps['map_left_x']
    map_left_y = maps['map_left_y']
    map_right_x = maps['map_right_x']
    map_right_y = maps['map_right_y']

    width = calib['image_width']
    height = calib['image_height']

    # Setup cameras
    left_pipeline = build_gstreamer_pipeline(0, width, height)
    right_pipeline = build_gstreamer_pipeline(1, width, height)

    left_cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
    if not left_cap.isOpened():
        print("ERROR: Failed to open left camera")
        return

    import time
    time.sleep(2)

    right_cap = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)
    if not right_cap.isOpened():
        print("ERROR: Failed to open right camera")
        left_cap.release()
        return

    print("✓ Cameras opened")

    # Setup CLAHE
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    # Setup StereoSGBM with improved parameters
    print("\nSetting up improved StereoSGBM...")
    window_size = 5
    min_disp = 0
    num_disp = 16 * 32  # 512

    left_matcher = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=8 * 3 * window_size ** 2,
        P2=32 * 3 * window_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=15,  # Increased from 10 → stricter matching
        speckleWindowSize=150,  # Increased from 100 → better filtering
        speckleRange=2,  # Decreased from 32 → stricter
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    # WLS filter with stronger parameters
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
    wls_filter.setLambda(10000)  # Increased from 8000
    wls_filter.setSigmaColor(1.5)  # Increased from 1.2

    print("✓ Enhanced matcher ready")
    print("  - uniquenessRatio: 15 (stricter)")
    print("  - speckleWindowSize: 150 (better filtering)")
    print("  - speckleRange: 2 (stricter)")
    print("  - WLS lambda: 10000 (stronger)")

    print("\n" + "=" * 70)
    print("Controls:")
    print("  'q' - Quit")
    print("  's' - Save current frame analysis")
    print("=" * 70)

    cv2.namedWindow('Left', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Depth Map', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Valid Mask', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Confidence', cv2.WINDOW_NORMAL)

    frame_count = 0

    while True:
        ret_left, frame_left = left_cap.read()
        ret_right, frame_right = right_cap.read()

        if not ret_left or not ret_right:
            print("ERROR: Failed to read frames")
            break

        # Rectify
        rectified_left = cv2.remap(frame_left, map_left_x, map_left_y,
                                   cv2.INTER_LINEAR)
        rectified_right = cv2.remap(frame_right, map_right_x, map_right_y,
                                    cv2.INTER_LINEAR)

        # Convert to grayscale and apply CLAHE
        gray_left = cv2.cvtColor(rectified_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rectified_right, cv2.COLOR_BGR2GRAY)

        enhanced_left = clahe.apply(gray_left)
        enhanced_right = clahe.apply(gray_right)

        # Compute disparity
        disparity_left = left_matcher.compute(enhanced_left, enhanced_right)
        disparity_right = right_matcher.compute(enhanced_right, enhanced_left)

        # Apply WLS filter
        disparity_filtered = wls_filter.filter(disparity_left, enhanced_left,
                                               None, disparity_right)

        # Get confidence map
        confidence = wls_filter.getConfidenceMap()

        # Convert disparity to depth
        disparity_float = disparity_filtered.astype(np.float32) / 16.0

        # Create valid mask
        valid_mask = (disparity_float > min_disp) & (disparity_float < num_disp)
        valid_mask = valid_mask & (confidence > 0)  # Only confident regions

        # Calculate depth
        depth_map = np.zeros_like(disparity_float)
        depth_map[valid_mask] = (baseline * focal_length) / disparity_float[valid_mask]

        # Filter unrealistic depths (20-100cm range)
        realistic_mask = (depth_map >= 200) & (depth_map <= 1000)  # 20-100cm in mm
        final_mask = valid_mask & realistic_mask

        # Calculate statistics
        total_pixels = width * height
        valid_pixels = np.sum(final_mask)
        coverage = (valid_pixels / total_pixels) * 100

        # Visualization
        # 1. Left image
        left_display = frame_left.copy()
        cv2.putText(left_display, f"Coverage: {coverage:.1f}%",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 2. Depth map (colored)
        depth_vis = depth_map.copy()
        depth_vis[~final_mask] = 0
        depth_vis = np.clip(depth_vis, 200, 1000)  # 20-100cm
        depth_vis = ((depth_vis - 200) / 800 * 255).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        depth_colored[~final_mask] = [0, 0, 0]  # Black for invalid

        # Add text
        cv2.putText(depth_colored, f"Valid: {coverage:.1f}%",
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # 3. Valid mask (green = valid, black = invalid)
        mask_vis = np.zeros((height, width, 3), dtype=np.uint8)
        mask_vis[final_mask] = [0, 255, 0]  # Green

        # Add grid to show coverage by region
        grid_rows = 6
        grid_cols = 10
        cell_h = height // grid_rows
        cell_w = width // grid_cols

        for i in range(grid_rows):
            for j in range(grid_cols):
                y1, y2 = i * cell_h, (i + 1) * cell_h
                x1, x2 = j * cell_w, (j + 1) * cell_w

                cell_mask = final_mask[y1:y2, x1:x2]
                cell_coverage = np.sum(cell_mask) / (cell_h * cell_w) * 100

                # Draw cell border
                color = (0, 255, 0) if cell_coverage > 50 else (0, 0, 255)
                cv2.rectangle(mask_vis, (x1, y1), (x2, y2), color, 2)

                # Draw coverage percentage
                text = f"{cell_coverage:.0f}%"
                cv2.putText(mask_vis, text,
                           (x1 + 5, y1 + cell_h // 2),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 4. Confidence map
        confidence_vis = (confidence / 255.0 * 255).astype(np.uint8)
        confidence_colored = cv2.applyColorMap(confidence_vis, cv2.COLORMAP_HOT)

        # Display
        cv2.imshow('Left', left_display)
        cv2.imshow('Depth Map', depth_colored)
        cv2.imshow('Valid Mask', mask_vis)
        cv2.imshow('Confidence', confidence_colored)

        # Statistics (print every 30 frames)
        if frame_count % 30 == 0:
            print(f"\n[Frame {frame_count}]")
            print(f"  Coverage: {coverage:.1f}% ({valid_pixels}/{total_pixels} pixels)")
            print(f"  Depth range: {depth_map[final_mask].min():.1f} - "
                  f"{depth_map[final_mask].max():.1f} mm")

            # Coverage by region (left vs right)
            left_half = final_mask[:, :width//2]
            right_half = final_mask[:, width//2:]
            left_coverage = np.sum(left_half) / left_half.size * 100
            right_coverage = np.sum(right_half) / right_half.size * 100

            print(f"  Left half: {left_coverage:.1f}%")
            print(f"  Right half: {right_coverage:.1f}%")

            if left_coverage < 30:
                print("  ⚠️  WARNING: Low coverage on left side!")

        frame_count += 1

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            timestamp = f"quality_{frame_count:04d}"
            cv2.imwrite(f"{timestamp}_left.jpg", left_display)
            cv2.imwrite(f"{timestamp}_depth.jpg", depth_colored)
            cv2.imwrite(f"{timestamp}_mask.jpg", mask_vis)
            cv2.imwrite(f"{timestamp}_confidence.jpg", confidence_colored)
            print(f"✓ Saved {timestamp}_*.jpg")

    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
