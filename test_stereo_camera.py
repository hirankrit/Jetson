#!/usr/bin/env python3
"""
Test script for IMX219 Stereo Camera on Jetson Orin Nano
Tests both cameras and displays live feed
"""

import cv2
import sys

def test_stereo_camera():
    """Test stereo camera by opening both video devices."""
    print("=" * 60)
    print("IMX219 Stereo Camera Test")
    print("=" * 60)
    print()

    # Camera device IDs
    LEFT_CAM = 0
    RIGHT_CAM = 1

    # Camera settings for IMX219
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30

    print(f"Attempting to open cameras...")
    print(f"  Left camera: /dev/video{LEFT_CAM}")
    print(f"  Right camera: /dev/video{RIGHT_CAM}")
    print()

    # Open left camera
    cap_left = cv2.VideoCapture(LEFT_CAM, cv2.CAP_V4L2)
    if not cap_left.isOpened():
        print(f"❌ ERROR: Cannot open left camera (/dev/video{LEFT_CAM})")
        print("   Check if camera is connected and driver is loaded.")
        sys.exit(1)

    # Open right camera
    cap_right = cv2.VideoCapture(RIGHT_CAM, cv2.CAP_V4L2)
    if not cap_right.isOpened():
        print(f"❌ ERROR: Cannot open right camera (/dev/video{RIGHT_CAM})")
        cap_left.release()
        sys.exit(1)

    # Set camera properties
    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap_left.set(cv2.CAP_PROP_FPS, FPS)

    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap_right.set(cv2.CAP_PROP_FPS, FPS)

    print("✅ Both cameras opened successfully!")
    print()
    print(f"Camera settings:")
    print(f"  Resolution: {WIDTH}x{HEIGHT}")
    print(f"  FPS: {FPS}")
    print()
    print("Reading test frames...")
    print()

    # Test reading frames
    ret_left, frame_left = cap_left.read()
    ret_right, frame_right = cap_right.read()

    if not ret_left or not ret_right:
        print("❌ ERROR: Failed to read frames from cameras")
        cap_left.release()
        cap_right.release()
        sys.exit(1)

    print(f"✅ Successfully captured frames!")
    print(f"  Left frame shape: {frame_left.shape}")
    print(f"  Right frame shape: {frame_right.shape}")
    print()

    # Display info
    print("=" * 60)
    print("LIVE PREVIEW")
    print("=" * 60)
    print("Press 'q' to quit")
    print("Press 's' to save snapshot")
    print()

    frame_count = 0
    snapshot_count = 0

    try:
        while True:
            # Read frames
            ret_left, frame_left = cap_left.read()
            ret_right, frame_right = cap_right.read()

            if not ret_left or not ret_right:
                print("⚠️  Warning: Dropped frame")
                continue

            frame_count += 1

            # Add labels
            cv2.putText(frame_left, "Left Camera", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame_right, "Right Camera", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Add frame counter
            cv2.putText(frame_left, f"Frame: {frame_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame_right, f"Frame: {frame_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Combine frames side by side
            combined = cv2.hconcat([frame_left, frame_right])

            # Resize for display (if needed)
            display_width = 1280
            aspect_ratio = combined.shape[0] / combined.shape[1]
            display_height = int(display_width * aspect_ratio)
            combined_resized = cv2.resize(combined, (display_width, display_height))

            # Display
            cv2.imshow('Stereo Camera Test', combined_resized)

            # Handle key press
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\nExiting...")
                break
            elif key == ord('s'):
                # Save snapshot
                snapshot_count += 1
                cv2.imwrite(f'snapshot_left_{snapshot_count}.jpg', frame_left)
                cv2.imwrite(f'snapshot_right_{snapshot_count}.jpg', frame_right)
                print(f"📸 Saved snapshot {snapshot_count}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    finally:
        # Cleanup
        print("\nCleaning up...")
        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()

        print()
        print("=" * 60)
        print("Test Summary")
        print("=" * 60)
        print(f"Total frames captured: {frame_count}")
        print(f"Snapshots saved: {snapshot_count}")
        print()
        print("✅ Stereo camera test completed successfully!")
        print()

if __name__ == "__main__":
    test_stereo_camera()
