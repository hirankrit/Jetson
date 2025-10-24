#!/usr/bin/env python3
"""
Simple viewer for IMX219 Stereo Camera
Displays both left and right camera feeds side-by-side
Press 'q' to quit, 's' to save snapshot
"""

import cv2
import numpy as np
import argparse
from datetime import datetime


def build_gstreamer_pipeline(sensor_id, width=1280, height=720, framerate=30, flip_method=0):
    """
    Build GStreamer pipeline for nvarguscamerasrc

    Args:
        sensor_id: Camera sensor ID (0 or 1)
        width: Image width
        height: Image height
        framerate: Frames per second
        flip_method: Flip method (0=none, 2=rotate-180)

    Returns:
        GStreamer pipeline string
    """
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
    parser = argparse.ArgumentParser(description='View IMX219 Stereo Camera')
    parser.add_argument('--width', type=int, default=1280, help='Image width (default: 1280)')
    parser.add_argument('--height', type=int, default=720, help='Image height (default: 720)')
    parser.add_argument('--fps', type=int, default=30, help='Framerate (default: 30)')
    parser.add_argument('--flip', type=int, default=0, help='Flip method (0=none, 2=rotate-180)')
    parser.add_argument('--single', type=int, help='View single camera only (0 or 1)')

    args = parser.parse_args()

    print("=" * 60)
    print("IMX219 Stereo Camera Viewer")
    print("=" * 60)
    print(f"Resolution: {args.width}x{args.height} @ {args.fps} fps")
    print(f"Flip method: {args.flip}")
    print("\nControls:")
    print("  'q' or ESC : Quit")
    print("  's'        : Save snapshot")
    print("=" * 60)

    # Build pipelines
    if args.single is not None:
        # Single camera mode
        pipeline = build_gstreamer_pipeline(args.single, args.width, args.height, args.fps, args.flip)
        print(f"\nOpening camera {args.single}...")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print(f"ERROR: Failed to open camera {args.single}")
            return

        print("Camera opened successfully!")
        print("\nStarting capture... (press 'q' to quit)")

        frame_count = 0
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Failed to capture frame")
                break

            frame_count += 1

            # Add info overlay
            cv2.putText(frame, f"Camera {args.single}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Frame: {frame_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow(f'Camera {args.single}', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            elif key == ord('s'):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"snapshot_cam{args.single}_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved: {filename}")

        cap.release()
        cv2.destroyAllWindows()

    else:
        # Stereo mode
        left_pipeline = build_gstreamer_pipeline(0, args.width, args.height, args.fps, args.flip)
        right_pipeline = build_gstreamer_pipeline(1, args.width, args.height, args.fps, args.flip)

        print("\nOpening left camera...")
        left_cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)

        if not left_cap.isOpened():
            print("ERROR: Failed to open left camera")
            return

        import time
        time.sleep(2)  # Wait for left camera to initialize

        print("Opening right camera...")
        right_cap = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)

        if not right_cap.isOpened():
            print("ERROR: Failed to open right camera")
            left_cap.release()
            return

        print("Both cameras opened successfully!")
        print("\nStarting capture... (press 'q' to quit)")

        frame_count = 0
        while True:
            ret_left, frame_left = left_cap.read()
            ret_right, frame_right = right_cap.read()

            if not ret_left or not ret_right:
                print("Failed to capture frames")
                break

            frame_count += 1

            # Add info overlays
            cv2.putText(frame_left, "LEFT Camera", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame_left, f"Frame: {frame_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(frame_right, "RIGHT Camera", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame_right, f"Frame: {frame_count}", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Combine side-by-side
            combined = np.hstack((frame_left, frame_right))

            # Add separator line
            height = combined.shape[0]
            cv2.line(combined, (args.width, 0), (args.width, height), (255, 255, 255), 2)

            cv2.imshow('Stereo Camera (Left | Right)', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            elif key == ord('s'):
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                left_filename = f"snapshot_left_{timestamp}.jpg"
                right_filename = f"snapshot_right_{timestamp}.jpg"
                stereo_filename = f"snapshot_stereo_{timestamp}.jpg"
                cv2.imwrite(left_filename, frame_left)
                cv2.imwrite(right_filename, frame_right)
                cv2.imwrite(stereo_filename, combined)
                print(f"Saved: {left_filename}, {right_filename}, {stereo_filename}")

        left_cap.release()
        right_cap.release()
        cv2.destroyAllWindows()

    print(f"\nTotal frames captured: {frame_count}")
    print("Viewer closed.")


if __name__ == '__main__':
    main()
