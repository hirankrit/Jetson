#!/usr/bin/env python3
"""
Interactive blob detector tuning for calibration pattern
Allows real-time parameter adjustment to find all circles
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


# Global parameters (will be updated by trackbars)
params = {
    'minArea': 50,
    'maxArea': 5000,
    'minCircularity': 70,  # 0-100 (will divide by 100)
    'minConvexity': 80,    # 0-100
    'minInertia': 50,      # 0-100
}


def nothing(x):
    """Dummy callback for trackbars"""
    pass


def create_blob_detector(params):
    """Create blob detector from parameters"""
    detector_params = cv2.SimpleBlobDetector_Params()

    detector_params.filterByColor = True
    detector_params.blobColor = 0  # dark blobs

    detector_params.filterByArea = True
    detector_params.minArea = params['minArea']
    detector_params.maxArea = params['maxArea']

    detector_params.filterByCircularity = True
    detector_params.minCircularity = params['minCircularity'] / 100.0

    detector_params.filterByConvexity = True
    detector_params.minConvexity = params['minConvexity'] / 100.0

    detector_params.filterByInertia = True
    detector_params.minInertiaRatio = params['minInertia'] / 100.0

    return cv2.SimpleBlobDetector_create(detector_params)


def main():
    print("=" * 70)
    print("Blob Detector Parameter Tuning")
    print("=" * 70)
    print("\nGoal: Find all 35 circles (5 rows × 7 columns)")
    print("\nControls:")
    print("  - Use trackbars to adjust parameters")
    print("  - Watch the blob count change")
    print("  - Press 's' to save current parameters")
    print("  - Press 'q' to quit")
    print("=" * 70)

    # Setup camera (left only)
    width, height = 1280, 720
    left_pipeline = build_gstreamer_pipeline(0, width, height)

    print("\nOpening left camera...")
    cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("ERROR: Failed to open camera")
        return

    print("Camera opened successfully!\n")

    # Create window and trackbars
    window_name = 'Blob Detector Tuning'
    cv2.namedWindow(window_name)

    cv2.createTrackbar('Min Area', window_name, params['minArea'], 500, nothing)
    cv2.createTrackbar('Max Area', window_name, params['maxArea'], 10000, nothing)
    cv2.createTrackbar('Min Circularity (%)', window_name, params['minCircularity'], 100, nothing)
    cv2.createTrackbar('Min Convexity (%)', window_name, params['minConvexity'], 100, nothing)
    cv2.createTrackbar('Min Inertia (%)', window_name, params['minInertia'], 100, nothing)

    # Pattern to test
    PATTERN_ROWS = 5
    PATTERN_COLS = 7
    PATTERN_TYPE = cv2.CALIB_CB_ASYMMETRIC_GRID

    print("\nAdjust parameters until you see 35 blobs detected")
    print("Green circles = detected blobs")
    print("If pattern is recognized, you'll see it highlighted\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Get current trackbar values
        params['minArea'] = cv2.getTrackbarPos('Min Area', window_name)
        params['maxArea'] = max(cv2.getTrackbarPos('Max Area', window_name), params['minArea'] + 1)
        params['minCircularity'] = cv2.getTrackbarPos('Min Circularity (%)', window_name)
        params['minConvexity'] = cv2.getTrackbarPos('Min Convexity (%)', window_name)
        params['minInertia'] = cv2.getTrackbarPos('Min Inertia (%)', window_name)

        # Create detector with current parameters
        detector = create_blob_detector(params)

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect blobs
        keypoints = detector.detect(gray)

        # Draw detected blobs (GREEN CIRCLES with THICK lines)
        display = frame.copy()
        for kp in keypoints:
            x, y = int(kp.pt[0]), int(kp.pt[1])
            radius = int(kp.size / 2)
            # Draw green circle with thick line
            cv2.circle(display, (x, y), radius, (0, 255, 0), 3)
            # Draw center point
            cv2.circle(display, (x, y), 3, (0, 255, 0), -1)

        # Try to find pattern
        ret_pattern, corners = cv2.findCirclesGrid(
            gray,
            (PATTERN_COLS, PATTERN_ROWS),
            flags=PATTERN_TYPE,
            blobDetector=detector
        )

        # Display info
        blob_count = len(keypoints)
        color = (0, 255, 0) if blob_count == 35 else (0, 165, 255)

        cv2.putText(display, f"Blobs detected: {blob_count} / 35", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        if ret_pattern:
            cv2.drawChessboardCorners(display, (PATTERN_COLS, PATTERN_ROWS), corners, ret_pattern)
            cv2.putText(display, "PATTERN FOUND!", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            status = "Pattern not found" if blob_count >= 35 else "Need more blobs"
            cv2.putText(display, status, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # Parameter display
        cv2.putText(display, f"Area: {params['minArea']}-{params['maxArea']}", (10, height - 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display, f"Circ: {params['minCircularity']}% Conv: {params['minConvexity']}% Inert: {params['minInertia']}%",
                   (10, height - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(display, "Press 's' to save, 'q' to quit", (10, height - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        cv2.imshow(window_name, display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # Save current parameters
            print("\n" + "=" * 70)
            print("CURRENT PARAMETERS:")
            print("=" * 70)
            print(f"minArea = {params['minArea']}")
            print(f"maxArea = {params['maxArea']}")
            print(f"minCircularity = {params['minCircularity'] / 100.0}")
            print(f"minConvexity = {params['minConvexity'] / 100.0}")
            print(f"minInertiaRatio = {params['minInertia'] / 100.0}")
            print(f"\nBlobs detected: {blob_count}")
            print(f"Pattern found: {'YES' if ret_pattern else 'NO'}")
            print("=" * 70)
            print("\nParameters saved to terminal output")
            print("Copy these values to update capture_calibration.py")

        elif key == ord('q') or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 70)
    print("Final Parameters:")
    print("=" * 70)
    print(f"minArea = {params['minArea']}")
    print(f"maxArea = {params['maxArea']}")
    print(f"minCircularity = {params['minCircularity'] / 100.0}")
    print(f"minConvexity = {params['minConvexity'] / 100.0}")
    print(f"minInertiaRatio = {params['minInertia'] / 100.0}")
    print("=" * 70)


if __name__ == '__main__':
    main()
