#!/usr/bin/env python3
"""
Test different configurations for 33-circle asymmetric grid
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


def main():
    print("=" * 70)
    print("Testing 33-Circle Asymmetric Grid Detection")
    print("=" * 70)
    print("\nPattern: 5 rows × 7 columns (33 circles total)")
    print("Row 1: 7 circles")
    print("Row 2: 6 circles (offset)")
    print("Row 3: 7 circles")
    print("Row 4: 6 circles (offset)")
    print("Row 5: 7 circles")
    print("=" * 70)

    # Setup camera
    width, height = 1280, 720
    left_pipeline = build_gstreamer_pipeline(0, width, height)

    print("\nOpening left camera...")
    cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("ERROR: Failed to open camera")
        return

    print("Camera opened successfully!")
    print("\nPress 'd' to detect, 'q' to quit\n")

    # Setup blob detector (relaxed parameters)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 0
    params.filterByArea = True
    params.minArea = 30
    params.maxArea = 7000
    params.filterByCircularity = True
    params.minCircularity = 0.6
    params.filterByConvexity = True
    params.minConvexity = 0.7
    params.filterByInertia = True
    params.minInertiaRatio = 0.4

    detector = cv2.SimpleBlobDetector_create(params)

    # Configurations to try for 33 circles
    configs = [
        # (rows, cols, flags, description)
        (5, 7, cv2.CALIB_CB_ASYMMETRIC_GRID, "ASYMMETRIC 5×7"),
        (7, 5, cv2.CALIB_CB_ASYMMETRIC_GRID, "ASYMMETRIC 7×5 (flipped)"),
        (5, 6, cv2.CALIB_CB_ASYMMETRIC_GRID, "ASYMMETRIC 5×6"),
        (6, 5, cv2.CALIB_CB_ASYMMETRIC_GRID, "ASYMMETRIC 6×5"),
        (11, 4, cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING, "ASYMMETRIC 11×4 + CLUSTERING"),
        (5, 7, cv2.CALIB_CB_ASYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING, "ASYMMETRIC 5×7 + CLUSTERING"),
        (5, 7, cv2.CALIB_CB_SYMMETRIC_GRID, "SYMMETRIC 5×7 (test)"),
    ]

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Detect blobs
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints = detector.detect(gray)

        # Draw blobs
        display = frame.copy()
        for kp in keypoints:
            x, y = int(kp.pt[0]), int(kp.pt[1])
            radius = int(kp.size / 2)
            cv2.circle(display, (x, y), radius, (0, 255, 0), 3)
            cv2.circle(display, (x, y), 3, (0, 255, 0), -1)

        blob_count = len(keypoints)
        color = (0, 255, 0) if blob_count == 33 else (0, 165, 255)
        cv2.putText(display, f"Blobs: {blob_count} / 33", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        cv2.putText(display, "Press 'd' to detect pattern", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow('33-Circle Pattern Test', display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('d'):
            print("\n" + "=" * 70)
            print(f"Detecting pattern... (Found {blob_count} blobs)")
            print("=" * 70)

            found_any = False

            for rows, cols, flags, desc in configs:
                ret_detect, corners = cv2.findCirclesGrid(
                    gray, (cols, rows), flags=flags, blobDetector=detector
                )

                if ret_detect:
                    print(f"✓ FOUND: {desc} ({rows} rows × {cols} cols)")
                    found_any = True

                    # Draw detected pattern
                    result = frame.copy()
                    cv2.drawChessboardCorners(result, (cols, rows), corners, ret_detect)
                    cv2.putText(result, f"FOUND: {desc}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow('Pattern Detected!', result)
                else:
                    print(f"✗ Failed: {desc}")

            if not found_any:
                print("\n⚠️  No pattern detected with any configuration!")
                print("\nPossible solutions:")
                print("1. Try different blob detector parameters (run tune_blob_detector.py)")
                print("2. Improve lighting (uniform, no shadows)")
                print("3. Ensure pattern is flat and fully visible")
                print("4. Try different pattern (checkerboard might work better)")

            print("=" * 70)

        elif key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
