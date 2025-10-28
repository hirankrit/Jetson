#!/usr/bin/env python3
"""
⚖️ Brightness Balance Tool - Match brightness between cameras

Helps tune exposure/gain to match brightness between left and right cameras
"""
import cv2
import numpy as np
import time

def build_gstreamer_pipeline(sensor_id, exposure_ns=33000000, gain=4, width=1280, height=720, framerate=15):
    """
    Build pipeline with adjustable exposure and gain

    Args:
        sensor_id: 0 (left) or 1 (right)
        exposure_ns: Exposure time in nanoseconds (default 33ms = 33000000)
        gain: Analog gain 1-16 (default 4)
    """
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} '
        f'wbmode=0 '
        f'exposuretimerange="{exposure_ns} {exposure_ns}" '
        f'gainrange="{gain} {gain}" '
        f'! '
        f'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, '
        f'format=(string)NV12, framerate=(fraction){framerate}/1 ! '
        f'nvvidconv flip-method=0 ! '
        f'video/x-raw, format=(string)BGRx ! '
        f'videoconvert ! '
        f'video/x-raw, format=(string)BGR ! '
        f'appsink'
    )

def calculate_brightness_metrics(frame):
    """Calculate detailed brightness metrics"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    return {
        'mean': gray.mean(),
        'median': np.median(gray),
        'std': gray.std(),
        'min': gray.min(),
        'max': gray.max(),
        'over_exp': (gray > 250).sum() / gray.size * 100,
        'under_exp': (gray < 5).sum() / gray.size * 100
    }

def main():
    print("=" * 80)
    print("⚖️  BRIGHTNESS BALANCE TOOL")
    print("=" * 80)
    print("\nThis tool helps you match brightness between left and right cameras")
    print("by adjusting exposure and gain for each camera independently.")
    print()

    # Initial settings (same for both)
    print("Current settings (from LED test):")
    print("  Exposure: 33ms (33000000 ns)")
    print("  Gain: 4")
    print()

    # Ask for custom settings
    print("Enter camera settings:")
    print("(Press Enter to use defaults)")
    print()

    # Left camera
    left_exp_input = input("Left camera exposure (ms) [33]: ").strip()
    left_exp = int(float(left_exp_input if left_exp_input else "33") * 1000000)

    left_gain_input = input("Left camera gain (1-16) [4]: ").strip()
    left_gain = float(left_gain_input if left_gain_input else "4")

    # Right camera
    right_exp_input = input("Right camera exposure (ms) [33]: ").strip()
    right_exp = int(float(right_exp_input if right_exp_input else "33") * 1000000)

    right_gain_input = input("Right camera gain (1-16) [4]: ").strip()
    right_gain = float(right_gain_input if right_gain_input else "4")

    print()
    print("=" * 80)
    print(f"Opening cameras with settings:")
    print(f"  Left:  Exposure={left_exp/1000000:.1f}ms  Gain={left_gain}")
    print(f"  Right: Exposure={right_exp/1000000:.1f}ms  Gain={right_gain}")
    print("=" * 80)

    # Build pipelines
    left_pipeline = build_gstreamer_pipeline(0, left_exp, left_gain)
    right_pipeline = build_gstreamer_pipeline(1, right_exp, right_gain)

    # Open cameras
    cap_left = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
    time.sleep(2)
    cap_right = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)

    if not cap_left.isOpened() or not cap_right.isOpened():
        print("❌ Failed to open cameras!")
        return

    print("\n✓ Cameras opened")
    print("\nControls:")
    print("  SPACE: Print brightness metrics")
    print("  's':   Save images")
    print("  'q':   Quit")
    print()
    print("Guide for adjustment:")
    print("  • Target brightness: 80-120 (both cameras)")
    print("  • Difference < 10 is excellent")
    print("  • Difference < 20 is good")
    print("  • Over-exposure < 5% is good")
    print()
    print("If left is darker:")
    print("  → Increase left exposure (e.g., 33 → 40ms)")
    print("  → OR increase left gain (e.g., 4 → 5)")
    print()
    print("If right is darker:")
    print("  → Increase right exposure")
    print("  → OR increase right gain")
    print()
    print("To restart with new settings: Close and run again")
    print("=" * 80)

    cv2.namedWindow('Balance', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Balance', 1280, 480)

    save_count = 0

    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()

        if not ret_left or not ret_right:
            break

        # Calculate metrics
        metrics_left = calculate_brightness_metrics(frame_left)
        metrics_right = calculate_brightness_metrics(frame_right)
        diff = abs(metrics_left['mean'] - metrics_right['mean'])

        # Determine status color
        if diff < 10:
            status_color = (0, 255, 0)  # Green
            status_text = "EXCELLENT"
        elif diff < 20:
            status_color = (0, 255, 255)  # Yellow
            status_text = "GOOD"
        else:
            status_color = (0, 0, 255)  # Red
            status_text = "ADJUST"

        # Draw on images
        cv2.putText(frame_left, f"LEFT Camera", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame_left, f"Bright: {metrics_left['mean']:.1f}", (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame_left, f"Exp: {left_exp/1000000:.1f}ms", (10, 95),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame_left, f"Gain: {left_gain}", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.putText(frame_right, f"RIGHT Camera", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame_right, f"Bright: {metrics_right['mean']:.1f}", (10, 65),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame_right, f"Exp: {right_exp/1000000:.1f}ms", (10, 95),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(frame_right, f"Gain: {right_gain}", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        # Draw status
        h = frame_left.shape[0]
        cv2.putText(frame_left, f"Diff: {diff:.1f}", (10, h-60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(frame_left, status_text, (10, h-25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)

        cv2.putText(frame_right, f"Diff: {diff:.1f}", (10, h-60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(frame_right, status_text, (10, h-25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)

        # Combine and show
        combined = np.hstack([frame_left, frame_right])
        cv2.imshow('Balance', combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        elif key == ord(' '):
            # Print detailed metrics
            print("\n" + "=" * 80)
            print("BRIGHTNESS METRICS")
            print("=" * 80)
            print(f"\nLeft Camera:")
            print(f"  Mean:       {metrics_left['mean']:.1f}")
            print(f"  Median:     {metrics_left['median']:.1f}")
            print(f"  Std Dev:    {metrics_left['std']:.1f}")
            print(f"  Range:      {metrics_left['min']} - {metrics_left['max']}")
            print(f"  Over-exp:   {metrics_left['over_exp']:.1f}%")
            print(f"  Under-exp:  {metrics_left['under_exp']:.1f}%")

            print(f"\nRight Camera:")
            print(f"  Mean:       {metrics_right['mean']:.1f}")
            print(f"  Median:     {metrics_right['median']:.1f}")
            print(f"  Std Dev:    {metrics_right['std']:.1f}")
            print(f"  Range:      {metrics_right['min']} - {metrics_right['max']}")
            print(f"  Over-exp:   {metrics_right['over_exp']:.1f}%")
            print(f"  Under-exp:  {metrics_right['under_exp']:.1f}%")

            print(f"\nDifference:")
            print(f"  Mean diff:  {diff:.1f}")
            print(f"  Status:     {status_text}")

            if diff > 10:
                if metrics_left['mean'] < metrics_right['mean']:
                    print(f"\n💡 Suggestion: Left is darker by {diff:.1f}")
                    print(f"   Option 1: Increase left exposure to {left_exp/1000000*1.2:.1f}ms")
                    print(f"   Option 2: Increase left gain to {min(left_gain+1, 16):.1f}")
                else:
                    print(f"\n💡 Suggestion: Right is darker by {diff:.1f}")
                    print(f"   Option 1: Increase right exposure to {right_exp/1000000*1.2:.1f}ms")
                    print(f"   Option 2: Increase right gain to {min(right_gain+1, 16):.1f}")

            print("=" * 80)

        elif key == ord('s'):
            # Save images
            timestamp = int(time.time())
            cv2.imwrite(f"balance_{timestamp}_left.jpg", frame_left)
            cv2.imwrite(f"balance_{timestamp}_right.jpg", frame_right)
            save_count += 1
            print(f"\n💾 Saved: balance_{timestamp}_*.jpg (#{save_count})")

    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 80)
    print("Session complete")
    print("=" * 80)

if __name__ == '__main__':
    main()
