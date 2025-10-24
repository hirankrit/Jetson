#!/usr/bin/env python3
"""
Test depth map accuracy using stereo calibration
Displays real-time depth map and allows distance measurement
"""

import cv2
import numpy as np
import yaml


def build_gstreamer_pipeline(sensor_id, width=1280, height=720, framerate=30, flip_method=0):
    """Build GStreamer pipeline for nvarguscamerasrc"""
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, '
        f'format=(string)NV12, framerate=(fraction){framerate}/1 ! '
        f'nvvidconv flip-method={flip_method} ! '
        f'video/x-raw, format=(string)BGRx ! '
        f'videoconvert ! '
        f'video/x-raw, format=(string)BGR ! '
        f'appsink'
    )


def load_calibration(calib_file='stereo_calib.yaml'):
    """Load stereo calibration parameters"""
    with open(calib_file, 'r') as f:
        calib_data = yaml.safe_load(f)

    # Load rectification maps
    maps = np.load('rectification_maps.npz')

    return calib_data, maps


def click_distance(event, x, y, flags, param):
    """Mouse callback to measure distance at clicked point"""
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
    print("Stereo Depth Map Testing")
    print("=" * 70)

    # Load calibration
    print("\nLoading calibration data...")
    try:
        calib_data, maps = load_calibration()
    except FileNotFoundError:
        print("ERROR: Calibration file not found!")
        print("Run stereo_calibration.py first")
        return

    baseline = calib_data['baseline_mm']
    Q = np.array(calib_data['stereo']['Q_matrix'])

    # Get focal length from Q matrix
    focal_length = abs(Q[2, 3])

    print(f"✓ Calibration loaded")
    print(f"  Baseline: {baseline:.2f} mm ({baseline/10:.1f} cm)")
    print(f"  Focal length: {focal_length:.2f} pixels")
    print(f"  Image size: {calib_data['image_width']}x{calib_data['image_height']}")
    print(f"  Stereo RMS error: {calib_data['stereo_rms_error']:.4f} pixels")

    # Load rectification maps
    map_left_x = maps['map_left_x']
    map_left_y = maps['map_left_y']
    map_right_x = maps['map_right_x']
    map_right_y = maps['map_right_y']

    # Setup cameras
    width = calib_data['image_width']
    height = calib_data['image_height']

    left_pipeline = build_gstreamer_pipeline(0, width, height)
    right_pipeline = build_gstreamer_pipeline(1, width, height)

    print("\nOpening cameras...")
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

    # Setup stereo matcher
    print("\nSetting up stereo matcher...")

    # StereoBM (faster)
    stereo = cv2.StereoBM_create(numDisparities=16*10, blockSize=15)
    stereo.setMinDisparity(0)
    stereo.setSpeckleWindowSize(100)
    stereo.setSpeckleRange(32)
    stereo.setDisp12MaxDiff(1)

    print("✓ Stereo matcher ready")
    print("\n" + "=" * 70)
    print("Controls:")
    print("  - Click on image to measure distance")
    print("  - 'q' or ESC: Quit")
    print("  - 's': Save current depth map")
    print("=" * 70)
    print("\nTips:")
    print("  - Point camera at objects 30-80cm away")
    print("  - Ensure good lighting")
    print("  - Look for textured surfaces (easier to match)")
    print("=" * 70)

    # Create window and set mouse callback
    depth_map = None
    cv2.namedWindow('Depth Map')

    frame_count = 0

    while True:
        ret_left, frame_left = left_cap.read()
        ret_right, frame_right = right_cap.read()

        if not ret_left or not ret_right:
            print("Failed to capture frames")
            break

        # Rectify images
        rect_left = cv2.remap(frame_left, map_left_x, map_left_y, cv2.INTER_LINEAR)
        rect_right = cv2.remap(frame_right, map_right_x, map_right_y, cv2.INTER_LINEAR)

        # Convert to grayscale
        gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

        # Compute disparity
        disparity = stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0

        # Compute depth (Z = focal * baseline / disparity)
        depth_map = np.zeros_like(disparity)
        mask = disparity > 0
        depth_map[mask] = (focal_length * baseline) / disparity[mask]

        # Set mouse callback with current depth map
        cv2.setMouseCallback('Depth Map', click_distance,
                            (depth_map, baseline, focal_length))

        # Normalize disparity for visualization
        disparity_vis = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
        disparity_vis = disparity_vis.astype(np.uint8)
        disparity_color = cv2.applyColorMap(disparity_vis, cv2.COLORMAP_JET)

        # Create depth visualization (limit to 0-1000mm range)
        depth_vis = np.clip(depth_map, 0, 1000)
        depth_vis = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = depth_vis.astype(np.uint8)
        depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # Add text overlay
        cv2.putText(depth_color, f"Baseline: {baseline:.1f}mm", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(depth_color, "Click to measure distance", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(depth_color, "Range: 0-100cm", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # Show images
        combined = np.hstack((rect_left, depth_color))
        cv2.imshow('Depth Map', combined)

        # Show disparity separately
        cv2.imshow('Disparity', disparity_color)

        frame_count += 1

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # Save depth map
            timestamp = cv2.getTickCount()
            cv2.imwrite(f'depth_map_{timestamp}.png', depth_color)
            np.save(f'depth_data_{timestamp}.npy', depth_map)
            print(f"\n💾 Saved depth map: depth_map_{timestamp}.png")

        elif key == ord('q') or key == 27:
            break

    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 70)
    print("Depth testing complete")
    print("=" * 70)


if __name__ == '__main__':
    main()
