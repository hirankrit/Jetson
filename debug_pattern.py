#!/usr/bin/env python3
"""
Debug script to detect calibration pattern and find correct parameters
"""

import cv2
import numpy as np


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


def test_pattern_detection(frame, pattern_sizes, pattern_types):
    """Test different pattern configurations"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Setup blob detector
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 0  # dark blobs
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 5000
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.filterByConvexity = True
    params.minConvexity = 0.8
    params.filterByInertia = True
    params.minInertiaRatio = 0.5

    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs first
    keypoints = detector.detect(gray)

    print(f"\n=== Blob Detection ===")
    print(f"Found {len(keypoints)} blobs")

    # Draw all detected blobs
    img_with_blobs = frame.copy()
    img_with_blobs = cv2.drawKeypoints(
        img_with_blobs, keypoints, np.array([]),
        (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
    )

    results = []

    # Try different configurations
    for pattern_type_name, pattern_type_flag in pattern_types.items():
        for rows, cols in pattern_sizes:
            ret, corners = cv2.findCirclesGrid(
                gray,
                (cols, rows),
                flags=pattern_type_flag,
                blobDetector=detector
            )

            if ret:
                result = {
                    'type': pattern_type_name,
                    'rows': rows,
                    'cols': cols,
                    'corners': corners,
                    'success': True
                }
                results.append(result)
                print(f"✓ FOUND: {pattern_type_name}, {rows}×{cols}")
            else:
                print(f"✗ Failed: {pattern_type_name}, {rows}×{cols}")

    return results, img_with_blobs, keypoints


def main():
    print("=" * 70)
    print("Calibration Pattern Debug Tool")
    print("=" * 70)
    print("\nThis tool will:")
    print("1. Show all detected blobs (green circles)")
    print("2. Try different pattern configurations")
    print("3. Report which patterns are detected")
    print("\nPress 'q' to quit")
    print("=" * 70)

    # Setup camera (left only for testing)
    width, height = 1280, 720
    left_pipeline = build_gstreamer_pipeline(0, width, height)

    print("\nOpening left camera...")
    cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("ERROR: Failed to open camera")
        return

    print("Camera opened successfully!\n")

    # Pattern configurations to try
    pattern_sizes = [
        # Common sizes
        (4, 11),  # Standard asymmetric
        (5, 13),  # Our configured size
        (3, 7),   # Smaller
        (4, 7),   # Smaller
        (5, 7),   # Medium
        (6, 8),   # Medium
        (7, 9),   # Larger
        # Try symmetric too
        (4, 7),
        (5, 8),
        (6, 9),
    ]

    pattern_types = {
        'ASYMMETRIC': cv2.CALIB_CB_ASYMMETRIC_GRID,
        'SYMMETRIC': cv2.CALIB_CB_SYMMETRIC_GRID,
    }

    detection_done = False

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to capture frame")
            break

        # Run detection once when pattern is visible
        key = cv2.waitKey(1) & 0xFF

        if key == ord('d') or not detection_done:
            print("\n" + "=" * 70)
            print("Detecting pattern...")
            print("=" * 70)

            results, img_with_blobs, keypoints = test_pattern_detection(
                frame, pattern_sizes, pattern_types
            )

            detection_done = True

            if results:
                print(f"\n✓ SUCCESS! Found {len(results)} matching pattern(s):")
                for r in results:
                    print(f"  - Type: {r['type']}, Size: {r['rows']}×{r['cols']}")

                    # Draw the best match
                    img_with_pattern = frame.copy()
                    cv2.drawChessboardCorners(
                        img_with_pattern,
                        (r['cols'], r['rows']),
                        r['corners'],
                        True
                    )
                    cv2.putText(
                        img_with_pattern,
                        f"{r['type']} {r['rows']}x{r['cols']}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2
                    )
                    cv2.imshow('Detected Pattern', img_with_pattern)
            else:
                print("\n✗ No pattern detected")
                print("\nPossible issues:")
                print("  1. Pattern not in view")
                print("  2. Pattern size not in our test list")
                print("  3. Lighting issues")
                print("  4. Circles too small/large")
                print(f"\nDetected {len(keypoints)} blobs - check if they look correct")

            # Show detected blobs
            cv2.imshow('Detected Blobs (Green)', img_with_blobs)

            print("\nPress 'd' to detect again, 'q' to quit")

        # Show live view
        display = frame.copy()
        cv2.putText(
            display,
            "Press 'd' to detect pattern",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2
        )
        cv2.imshow('Live View', display)

        if key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 70)
    print("Debug session complete")
    print("=" * 70)


if __name__ == '__main__':
    main()
