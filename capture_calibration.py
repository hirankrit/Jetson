#!/usr/bin/env python3
"""
Capture calibration images for IMX219 Stereo Camera
Using ASYMMETRIC CIRCLES pattern (recommended for agricultural applications)

Pattern specifications:
- Rows: 5
- Columns: 6
- Diagonal Spacing: 18mm (CONFIRMED - measured from printed pattern)
- Circle Diameter: 14mm
- Total circles: 33

Features:
- Real-time focus monitoring (Laplacian variance)
- Automatic pattern detection
- Focus status indicator (Green=Good, Yellow=OK, Red=Check!)
- Warns if focus changes during capture

Instructions:
1. Print the asymmetric circles pattern from:
   https://calib.io/pages/camera-calibration-pattern-generator
   Settings: Asymmetric Circles Grid, 5x6, 18mm spacing, 14mm diameter

2. Mount pattern on rigid board (foam board or acrylic)

3. IMPORTANT: Check focus before starting!
   - Optimal values: Left ~176.5, Right ~171.0, Diff < 10
   - If different, re-adjust focus (see CAMERA_SETUP_GUIDE.md)

4. Run this script and move the pattern around:
   - Different angles (tilt, rotate)
   - Different distances (30cm to 50cm recommended)
   - Cover the entire field of view
   - At least 20-30 good images
   - Monitor focus values on screen!

5. Press 'c' to capture, 'q' to quit
"""

import cv2
import numpy as np
import os
from datetime import datetime


def calculate_focus(image):
    """
    Calculate focus measure using Laplacian variance
    Higher value = sharper image

    Args:
        image: Grayscale image

    Returns:
        float: Focus measure (Laplacian variance)
    """
    laplacian = cv2.Laplacian(image, cv2.CV_64F)
    return laplacian.var()


def calculate_brightness(image):
    """
    Calculate mean brightness of image

    Args:
        image: Grayscale image

    Returns:
        float: Mean brightness (0-255)
    """
    return np.mean(image)


def calculate_contrast(image):
    """
    Calculate contrast using standard deviation

    Args:
        image: Grayscale image

    Returns:
        float: Standard deviation (higher = better contrast)
    """
    return np.std(image)


def calculate_exposure_stats(image):
    """
    Calculate over/under exposure statistics

    Args:
        image: Grayscale image

    Returns:
        dict: {'overexposed_pct': float, 'underexposed_pct': float}
    """
    total_pixels = image.size
    overexposed = np.sum(image > 250)
    underexposed = np.sum(image < 10)

    return {
        'overexposed_pct': (overexposed / total_pixels) * 100,
        'underexposed_pct': (underexposed / total_pixels) * 100
    }


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

    # Optimal focus values (from CAMERA_SETUP_GUIDE.md)
    OPTIMAL_FOCUS_LEFT = 176.5
    OPTIMAL_FOCUS_RIGHT = 171.0
    FOCUS_TOLERANCE = 15.0  # Warning if focus differs by more than this

    # Optimal lighting parameters
    BRIGHTNESS_MIN = 50    # Too dark if < 50
    BRIGHTNESS_MAX = 200   # Too bright if > 200
    BRIGHTNESS_DIFF_MAX = 20  # Max acceptable difference between cameras
    CONTRAST_MIN = 30      # Minimum acceptable contrast (std dev)
    EXPOSURE_THRESHOLD = 5.0  # Max acceptable % of over/under exposed pixels

    print("=" * 70)
    print("IMX219 Stereo Camera Calibration - Image Capture")
    print("=" * 70)
    print(f"\nPattern: Asymmetric Circles Grid")
    print(f"Rows: {PATTERN_ROWS}")
    print(f"Columns: {PATTERN_COLS}")
    print(f"Total Circles: 33")
    print(f"Diagonal Spacing: 18mm")
    print(f"Circle Diameter: 14mm")
    print(f"\nOptimal Parameters:")
    print(f"  Focus:")
    print(f"    Left:  {OPTIMAL_FOCUS_LEFT:.1f}")
    print(f"    Right: {OPTIMAL_FOCUS_RIGHT:.1f}")
    print(f"    Diff:  < 10.0 (acceptable)")
    print(f"  Lighting:")
    print(f"    Brightness: {BRIGHTNESS_MIN}-{BRIGHTNESS_MAX}")
    print(f"    Contrast (Std Dev): > {CONTRAST_MIN}")
    print(f"    Over/Under Exposure: < {EXPOSURE_THRESHOLD}%")
    print(f"    Brightness Diff (L-R): < {BRIGHTNESS_DIFF_MAX}")
    print("\nStatus Indicators:")
    print("  🟢 Green (GOOD):  All parameters optimal")
    print("  🟡 Yellow (OK):   Acceptable, minor issues")
    print("  🔴 Red (CHECK!): Needs adjustment")
    print("\nControls:")
    print("  'c' : Capture image pair (only when pattern detected)")
    print("  'q' : Quit")
    print("=" * 70)
    print("\nTips for good calibration:")
    print("  - Hold pattern steady when capturing")
    print("  - Cover different angles (tilt, rotate)")
    print("  - Cover different distances (30-50cm)")
    print("  - Make sure pattern is fully visible in both cameras")
    print("  - Aim for 20-30 good images")
    print("  - Monitor focus AND lighting values (shown on screen)")
    print("  - Capture only when both Focus and Lighting are GOOD/OK")
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

        # Calculate focus values
        focus_left = calculate_focus(gray_left)
        focus_right = calculate_focus(gray_right)
        focus_diff = abs(focus_left - focus_right)

        # Determine focus status
        left_diff = abs(focus_left - OPTIMAL_FOCUS_LEFT)
        right_diff = abs(focus_right - OPTIMAL_FOCUS_RIGHT)

        # Color coding: Green = good, Yellow = warning, Red = bad
        if focus_diff < 10.0 and left_diff < FOCUS_TOLERANCE and right_diff < FOCUS_TOLERANCE:
            focus_color = (0, 255, 0)  # Green - excellent
            focus_status = "GOOD"
        elif focus_diff < 20.0:
            focus_color = (0, 255, 255)  # Yellow - acceptable
            focus_status = "OK"
        else:
            focus_color = (0, 0, 255)  # Red - needs adjustment
            focus_status = "CHECK!"

        # Calculate lighting parameters
        brightness_left = calculate_brightness(gray_left)
        brightness_right = calculate_brightness(gray_right)
        brightness_diff = abs(brightness_left - brightness_right)

        contrast_left = calculate_contrast(gray_left)
        contrast_right = calculate_contrast(gray_right)

        exposure_left = calculate_exposure_stats(gray_left)
        exposure_right = calculate_exposure_stats(gray_right)

        # Determine lighting status
        lighting_issues = []

        # Check brightness range
        if brightness_left < BRIGHTNESS_MIN or brightness_right < BRIGHTNESS_MIN:
            lighting_issues.append("DARK")
        elif brightness_left > BRIGHTNESS_MAX or brightness_right > BRIGHTNESS_MAX:
            lighting_issues.append("BRIGHT")

        # Check brightness difference
        if brightness_diff > BRIGHTNESS_DIFF_MAX:
            lighting_issues.append("UNEVEN")

        # Check contrast
        if contrast_left < CONTRAST_MIN or contrast_right < CONTRAST_MIN:
            lighting_issues.append("LOW_CONTRAST")

        # Check exposure
        max_overexposed = max(exposure_left['overexposed_pct'], exposure_right['overexposed_pct'])
        max_underexposed = max(exposure_left['underexposed_pct'], exposure_right['underexposed_pct'])

        if max_overexposed > EXPOSURE_THRESHOLD:
            lighting_issues.append("OVEREXP")
        if max_underexposed > EXPOSURE_THRESHOLD:
            lighting_issues.append("UNDEREXP")

        # Overall lighting status
        if len(lighting_issues) == 0:
            lighting_color = (0, 255, 0)  # Green
            lighting_status = "GOOD"
        elif len(lighting_issues) <= 2:
            lighting_color = (0, 255, 255)  # Yellow
            lighting_status = "OK"
        else:
            lighting_color = (0, 0, 255)  # Red
            lighting_status = "CHECK!"

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

        # Font settings
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        line_height = 35

        # Left camera info
        y_pos = 70
        cv2.putText(display_left, f"Focus: {focus_left:.1f}", (10, y_pos),
                   font, font_scale, focus_color, thickness)
        y_pos += line_height
        cv2.putText(display_left, f"Bright: {brightness_left:.1f}", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_left, f"Contrast: {contrast_left:.1f}", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_left, f"Over: {exposure_left['overexposed_pct']:.2f}%", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_left, f"Under: {exposure_left['underexposed_pct']:.2f}%", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_left, f"Status: {lighting_status}", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_left, f"Captured: {capture_count}", (10, y_pos),
                   font, font_scale, (255, 255, 0), thickness)

        # Right camera info
        y_pos = 70
        cv2.putText(display_right, f"Focus: {focus_right:.1f}", (10, y_pos),
                   font, font_scale, focus_color, thickness)
        y_pos += line_height
        cv2.putText(display_right, f"Bright: {brightness_right:.1f}", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_right, f"Contrast: {contrast_right:.1f}", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_right, f"Over: {exposure_right['overexposed_pct']:.2f}%", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_right, f"Under: {exposure_right['underexposed_pct']:.2f}%", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_right, f"Status: {lighting_status}", (10, y_pos),
                   font, font_scale, lighting_color, thickness)
        y_pos += line_height
        cv2.putText(display_right, f"Captured: {capture_count}", (10, y_pos),
                   font, font_scale, (255, 255, 0), thickness)

        # Add summary info at bottom
        summary_y = height - 40
        cv2.putText(display_left, f"F.Diff: {focus_diff:.1f} | B.Diff: {brightness_diff:.1f}",
                   (10, summary_y), font, 0.5, (255, 255, 255), 1)
        cv2.putText(display_right, f"F.Diff: {focus_diff:.1f} | B.Diff: {brightness_diff:.1f}",
                   (10, summary_y), font, 0.5, (255, 255, 255), 1)

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

                # Print detailed info with capture
                print(f"\n✓ Captured pair #{capture_count}")
                print(f"  {'='*60}")

                # Focus info
                print(f"  Focus:")
                print(f"    Left: {focus_left:.1f}, Right: {focus_right:.1f}, Diff: {focus_diff:.1f}")
                if focus_status == "CHECK!":
                    print(f"    ⚠️  WARNING: Focus may have changed! (Status: {focus_status})")
                    print(f"        Optimal - Left: {OPTIMAL_FOCUS_LEFT:.1f}, Right: {OPTIMAL_FOCUS_RIGHT:.1f}")
                elif focus_status == "GOOD":
                    print(f"    ✓  Focus is excellent!")
                else:
                    print(f"    Status: {focus_status}")

                # Lighting info
                print(f"  Lighting:")
                print(f"    Brightness - Left: {brightness_left:.1f}, Right: {brightness_right:.1f}, Diff: {brightness_diff:.1f}")
                print(f"    Contrast - Left: {contrast_left:.1f}, Right: {contrast_right:.1f}")
                print(f"    Overexposed - Left: {exposure_left['overexposed_pct']:.2f}%, Right: {exposure_right['overexposed_pct']:.2f}%")
                print(f"    Underexposed - Left: {exposure_left['underexposed_pct']:.2f}%, Right: {exposure_right['underexposed_pct']:.2f}%")
                print(f"    Status: {lighting_status}")

                # Warnings
                if len(lighting_issues) > 0:
                    print(f"    ⚠️  Issues: {', '.join(lighting_issues)}")
                else:
                    print(f"    ✓  Lighting is excellent!")

                print(f"  {'='*60}")

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
