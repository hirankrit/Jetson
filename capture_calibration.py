#!/usr/bin/env python3
"""
Capture calibration images for IMX219 Stereo Camera
Using ASYMMETRIC CIRCLES pattern (recommended for agricultural applications)

Pattern specifications:
- Rows: 5
- Columns: 6
- Diagonal Spacing: 18mm
- Circle Diameter: 14mm
- Total circles: 33

Instructions:
1. Print the asymmetric circles pattern from:
   https://calib.io/pages/camera-calibration-pattern-generator
   Settings: Asymmetric Circles Grid, 5x6, 18mm spacing, 14mm diameter

2. Mount pattern on rigid board (foam board or acrylic)

3. Run this script and move the pattern around:
   - Different angles (tilt, rotate)
   - Different distances (40cm to 80cm)
   - Cover the entire field of view
   - At least 20-30 good images

4. Press 'c' to capture, 'q' to quit
"""

import cv2
import numpy as np
import os
from datetime import datetime


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


def main():
    # Pattern parameters (ASYMMETRIC CIRCLES)
    PATTERN_ROWS = 5
    PATTERN_COLS = 6
    PATTERN_TYPE = cv2.CALIB_CB_ASYMMETRIC_GRID

    # Create output directories
    os.makedirs('calib_images/left', exist_ok=True)
    os.makedirs('calib_images/right', exist_ok=True)

    print("=" * 70)
    print("IMX219 Stereo Camera Calibration - Image Capture")
    print("=" * 70)
    print(f"\nPattern: Asymmetric Circles Grid")
    print(f"Rows: {PATTERN_ROWS}")
    print(f"Columns: {PATTERN_COLS}")
    print(f"Total Circles: 33")
    print(f"Diagonal Spacing: 18mm")
    print(f"Circle Diameter: 14mm")
    print("\nControls:")
    print("  'c' : Capture image pair (only when pattern detected)")
    print("  'q' : Quit")
    print("=" * 70)
    print("\nTips for good calibration:")
    print("  - Hold pattern steady when capturing")
    print("  - Cover different angles (tilt, rotate)")
    print("  - Cover different distances (40-80cm)")
    print("  - Make sure pattern is fully visible in both cameras")
    print("  - Aim for 20-30 good images")
    print("=" * 70)

    # Setup cameras
    width, height = 1280, 720
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

    print("Cameras opened successfully!")
    print("\nReady to capture. Move the pattern around and press 'c' to capture.\n")

    # Calibration parameters for circle detection
    params = cv2.SimpleBlobDetector_Params()

    # Filter by color (black circles on white background)
    params.filterByColor = True
    params.blobColor = 0  # 0 for dark blobs

    # Filter by area
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 5000

    # Filter by circularity
    params.filterByCircularity = True
    params.minCircularity = 0.8

    # Filter by convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87

    # Filter by inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.6

    detector = cv2.SimpleBlobDetector_create(params)

    capture_count = 0

    while True:
        ret_left, frame_left = left_cap.read()
        ret_right, frame_right = right_cap.read()

        if not ret_left or not ret_right:
            print("Failed to capture frames")
            break

        # Convert to grayscale
        gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

        # Find circles
        ret_left_detect, corners_left = cv2.findCirclesGrid(
            gray_left,
            (PATTERN_COLS, PATTERN_ROWS),
            flags=PATTERN_TYPE,
            blobDetector=detector
        )

        ret_right_detect, corners_right = cv2.findCirclesGrid(
            gray_right,
            (PATTERN_COLS, PATTERN_ROWS),
            flags=PATTERN_TYPE,
            blobDetector=detector
        )

        # Draw detected circles
        display_left = frame_left.copy()
        display_right = frame_right.copy()

        if ret_left_detect:
            cv2.drawChessboardCorners(
                display_left,
                (PATTERN_COLS, PATTERN_ROWS),
                corners_left,
                ret_left_detect
            )
            cv2.putText(display_left, "DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(display_left, "NOT DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if ret_right_detect:
            cv2.drawChessboardCorners(
                display_right,
                (PATTERN_COLS, PATTERN_ROWS),
                corners_right,
                ret_right_detect
            )
            cv2.putText(display_right, "DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            cv2.putText(display_right, "NOT DETECTED", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Add capture count
        cv2.putText(display_left, f"Captured: {capture_count}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(display_right, f"Captured: {capture_count}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # Combine for display
        combined = np.hstack((display_left, display_right))
        cv2.line(combined, (width, 0), (width, height), (255, 255, 255), 2)

        cv2.imshow('Stereo Calibration Capture (Left | Right)', combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            # Capture only if both patterns detected
            if ret_left_detect and ret_right_detect:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

                left_filename = f"calib_images/left/img_{capture_count:03d}_{timestamp}.jpg"
                right_filename = f"calib_images/right/img_{capture_count:03d}_{timestamp}.jpg"

                cv2.imwrite(left_filename, frame_left)
                cv2.imwrite(right_filename, frame_right)

                capture_count += 1
                print(f"✓ Captured pair #{capture_count}")

                if capture_count >= 30:
                    print("\n✓ Recommended number of images (30) reached!")
                    print("  You can continue capturing or press 'q' to quit.")
            else:
                print("✗ Cannot capture: Pattern not detected in both cameras")

        elif key == ord('q') or key == 27:  # 'q' or ESC
            break

    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 70)
    print(f"Capture complete! Total images: {capture_count}")
    print("\nImages saved to:")
    print(f"  - calib_images/left/")
    print(f"  - calib_images/right/")
    print("\nNext step: Run stereo_calibration.py to compute calibration parameters")
    print("=" * 70)


if __name__ == '__main__':
    main()
