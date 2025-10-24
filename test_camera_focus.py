#!/usr/bin/env python3
"""
Test camera focus and sharpness
"""
import cv2
import numpy as np

def build_gstreamer_pipeline(sensor_id, width=1280, height=720, framerate=30):
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, '
        f'format=(string)NV12, framerate=(fraction){framerate}/1 ! '
        f'nvvidconv ! video/x-raw, format=(string)BGRx ! '
        f'videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    )

def calculate_sharpness(image):
    """Calculate image sharpness using Laplacian variance"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    variance = laplacian.var()
    return variance

def main():
    print("=" * 70)
    print("Camera Focus Test")
    print("=" * 70)
    print("\nCapturing from both cameras...")

    # Open cameras
    left_pipeline = build_gstreamer_pipeline(0)
    right_pipeline = build_gstreamer_pipeline(1)

    cap_left = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
    cap_right = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)

    if not cap_left.isOpened() or not cap_right.isOpened():
        print("❌ Cannot open cameras!")
        return

    print("✓ Cameras opened")
    print("\nInstructions:")
    print("  - Point cameras at pattern or textured surface")
    print("  - Press SPACE to measure sharpness")
    print("  - Press 'q' to quit")
    print()

    sharpness_history_left = []
    sharpness_history_right = []

    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        if not ret_left or not ret_right:
            print("❌ Failed to capture frames")
            break

        # Calculate sharpness
        sharp_left = calculate_sharpness(frame_left)
        sharp_right = calculate_sharpness(frame_right)

        # Draw info
        cv2.putText(frame_left, f"Left Camera", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame_left, f"Sharpness: {sharp_left:.1f}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        cv2.putText(frame_right, f"Right Camera", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame_right, f"Sharpness: {sharp_right:.1f}", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Show images
        combined = np.hstack((frame_left, frame_right))
        cv2.imshow('Camera Focus Test', combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            # Measure and record
            sharpness_history_left.append(sharp_left)
            sharpness_history_right.append(sharp_right)

            print(f"\n📸 Measurement #{len(sharpness_history_left)}:")
            print(f"   Left:  {sharp_left:.1f}")
            print(f"   Right: {sharp_right:.1f}")
            print(f"   Diff:  {abs(sharp_left - sharp_right):.1f}")

        elif key == ord('q'):
            break

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

    if sharpness_history_left:
        print("\n" + "=" * 70)
        print("Focus Test Summary")
        print("=" * 70)

        avg_left = np.mean(sharpness_history_left)
        avg_right = np.mean(sharpness_history_right)

        print(f"\nAverage Sharpness:")
        print(f"  Left:  {avg_left:.1f}")
        print(f"  Right: {avg_right:.1f}")
        print(f"  Diff:  {abs(avg_left - avg_right):.1f}")

        print(f"\nAssessment:")
        if abs(avg_left - avg_right) < 50:
            print("  ✅ Focus balanced (difference < 50)")
        elif abs(avg_left - avg_right) < 100:
            print("  ⚠️  Slight focus imbalance (50-100)")
        else:
            print("  ❌ Significant focus imbalance (> 100)")

        if avg_left < 100 or avg_right < 100:
            print("  ⚠️  Overall sharpness low (< 100)")
            print("      → Cameras may need better focus")
        elif avg_left < 200 or avg_right < 200:
            print("  ✓  Moderate sharpness (100-200)")
        else:
            print("  ✅ Good sharpness (> 200)")

    print("\n" + "=" * 70)

if __name__ == '__main__':
    main()
