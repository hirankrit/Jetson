#!/usr/bin/env python3
"""
🔍 Camera Diagnostic Tool - Check Auto-Focus, Auto-Exposure, etc.

This tool helps diagnose flickering/unstable camera issues
"""
import cv2
import numpy as np
import time

def build_gstreamer_pipeline_auto(sensor_id, width=1280, height=720, framerate=15):
    """Current pipeline (may have auto-focus issues)"""
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} ! '
        f'video/x-raw(memory:NVMM), '
        f'width=(int){width}, height=(int){height}, '
        f'format=(string)NV12, framerate=(fraction){framerate}/1 ! '
        f'nvvidconv flip-method=0 ! '
        f'video/x-raw, format=(string)BGRx ! '
        f'videoconvert ! '
        f'video/x-raw, format=(string)BGR ! '
        f'appsink'
    )

def build_gstreamer_pipeline_manual(sensor_id, width=1280, height=720, framerate=15):
    """Fixed pipeline with manual controls"""
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} '
        # Disable auto white balance
        f'wbmode=0 '
        # Fix exposure time (in nanoseconds)
        # 33ms = 33000000 ns (reasonable for indoor)
        f'exposuretimerange="33000000 33000000" '
        # Fix gain (1-16, higher = brighter)
        f'gainrange="4 4" '
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

def calculate_metrics(frame):
    """Calculate image metrics"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Sharpness (Laplacian variance)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    sharpness = laplacian.var()

    # Brightness (mean)
    brightness = gray.mean()

    # Contrast (std dev)
    contrast = gray.std()

    # Over/Under exposure
    over_exp = (gray > 250).sum() / gray.size * 100
    under_exp = (gray < 5).sum() / gray.size * 100

    return {
        'sharpness': sharpness,
        'brightness': brightness,
        'contrast': contrast,
        'over_exp': over_exp,
        'under_exp': under_exp
    }

def main():
    print("=" * 80)
    print("🔍 CAMERA DIAGNOSTIC TOOL")
    print("=" * 80)
    print("\nThis tool helps diagnose:")
    print("  • Auto-focus issues (sharpness flickering)")
    print("  • Auto-exposure issues (brightness flickering)")
    print("  • Auto-white-balance issues")
    print()

    # Ask user which mode to test
    print("Choose mode:")
    print("  1. AUTO mode (current - may have issues)")
    print("  2. MANUAL mode (fixed exposure/gain - recommended)")
    print("  3. COMPARE both (side by side)")
    print()

    choice = input("Enter choice (1/2/3): ").strip()

    if choice == '3':
        test_comparison()
    else:
        use_manual = (choice == '2')
        test_single_mode(use_manual)

def test_single_mode(use_manual=False):
    """Test single camera mode"""
    mode_name = "MANUAL" if use_manual else "AUTO"
    print(f"\n{'='*80}")
    print(f"Testing {mode_name} Mode - Left Camera (sensor-id=0)")
    print("="*80)

    if use_manual:
        pipeline = build_gstreamer_pipeline_manual(0)
        print("\n⚙️  Using MANUAL controls:")
        print("   • White Balance: MANUAL (wbmode=0)")
        print("   • Exposure: FIXED at 33ms")
        print("   • Gain: FIXED at 4")
    else:
        pipeline = build_gstreamer_pipeline_auto(0)
        print("\n⚠️  Using AUTO controls (default)")
        print("   • White Balance: AUTO")
        print("   • Exposure: AUTO")
        print("   • Gain: AUTO")

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("❌ Failed to open camera!")
        return

    print("\n✓ Camera opened")
    print("\nMonitoring metrics (30 samples)...")
    print("Look for flickering values!")
    print()

    metrics_history = []

    for i in range(30):
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to capture frame")
            break

        metrics = calculate_metrics(frame)
        metrics_history.append(metrics)

        # Print every 5 frames
        if (i+1) % 5 == 0:
            print(f"Sample {i+1:2d}: "
                  f"Sharp={metrics['sharpness']:6.1f}  "
                  f"Bright={metrics['brightness']:5.1f}  "
                  f"Contrast={metrics['contrast']:5.1f}  "
                  f"Over={metrics['over_exp']:4.1f}%  "
                  f"Under={metrics['under_exp']:4.1f}%")

        time.sleep(0.1)  # 100ms between samples

    cap.release()

    # Analysis
    print("\n" + "="*80)
    print("ANALYSIS")
    print("="*80)

    if not metrics_history:
        return

    # Convert to arrays
    sharpness = np.array([m['sharpness'] for m in metrics_history])
    brightness = np.array([m['brightness'] for m in metrics_history])
    contrast = np.array([m['contrast'] for m in metrics_history])

    print(f"\nSharpness:")
    print(f"  Mean:  {sharpness.mean():.1f}")
    print(f"  Std:   {sharpness.std():.1f}")
    print(f"  Range: {sharpness.min():.1f} - {sharpness.max():.1f}")
    print(f"  Variation: {(sharpness.max() - sharpness.min()) / sharpness.mean() * 100:.1f}%")

    print(f"\nBrightness:")
    print(f"  Mean:  {brightness.mean():.1f}")
    print(f"  Std:   {brightness.std():.1f}")
    print(f"  Range: {brightness.min():.1f} - {brightness.max():.1f}")
    print(f"  Variation: {(brightness.max() - brightness.min()) / brightness.mean() * 100:.1f}%")

    print(f"\nContrast:")
    print(f"  Mean:  {contrast.mean():.1f}")
    print(f"  Std:   {contrast.std():.1f}")
    print(f"  Range: {contrast.min():.1f} - {contrast.max():.1f}")

    # Diagnosis
    print("\n" + "="*80)
    print("DIAGNOSIS")
    print("="*80)

    sharp_var = (sharpness.max() - sharpness.min()) / sharpness.mean() * 100
    bright_var = (brightness.max() - brightness.min()) / brightness.mean() * 100

    issues = []

    if sharp_var > 30:
        issues.append(f"⚠️  Sharpness UNSTABLE (variation {sharp_var:.1f}% > 30%)")
        issues.append("   → Possible auto-focus or vibration issue")
    else:
        print(f"✅ Sharpness STABLE (variation {sharp_var:.1f}% < 30%)")

    if bright_var > 20:
        issues.append(f"⚠️  Brightness UNSTABLE (variation {bright_var:.1f}% > 20%)")
        issues.append("   → Possible auto-exposure issue")
    else:
        print(f"✅ Brightness STABLE (variation {bright_var:.1f}% < 20%)")

    if sharpness.mean() < 100:
        issues.append(f"⚠️  Overall sharpness LOW ({sharpness.mean():.1f} < 100)")
        issues.append("   → Camera may need manual focus adjustment")
    else:
        print(f"✅ Overall sharpness GOOD ({sharpness.mean():.1f} > 100)")

    if issues:
        print("\n🔴 ISSUES FOUND:")
        for issue in issues:
            print(issue)

        if not use_manual:
            print("\n💡 RECOMMENDATION:")
            print("   Try MANUAL mode (option 2) to fix auto-exposure/auto-focus issues")
    else:
        print("\n✅ No issues detected - camera is stable!")

    print("="*80)

def test_comparison():
    """Compare AUTO vs MANUAL mode side by side"""
    print("\n" + "="*80)
    print("COMPARISON MODE - AUTO vs MANUAL")
    print("="*80)
    print("\nOpening both cameras...")
    print("  Left:  AUTO mode")
    print("  Right: MANUAL mode (fixed exposure/gain)")
    print()

    pipeline_auto = build_gstreamer_pipeline_auto(0)
    pipeline_manual = build_gstreamer_pipeline_manual(1)

    cap_auto = cv2.VideoCapture(pipeline_auto, cv2.CAP_GSTREAMER)
    time.sleep(2)
    cap_manual = cv2.VideoCapture(pipeline_manual, cv2.CAP_GSTREAMER)

    if not cap_auto.isOpened() or not cap_manual.isOpened():
        print("❌ Failed to open cameras!")
        return

    print("✓ Cameras opened")
    print("\nPress 'q' to quit, SPACE to print metrics")
    print()

    cv2.namedWindow('Comparison', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Comparison', 1280, 480)

    while True:
        ret_auto, frame_auto = cap_auto.read()
        ret_manual, frame_manual = cap_manual.read()

        if not ret_auto or not ret_manual:
            break

        metrics_auto = calculate_metrics(frame_auto)
        metrics_manual = calculate_metrics(frame_manual)

        # Draw metrics
        cv2.putText(frame_auto, "AUTO Mode (Left Camera)", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame_auto, f"Sharp: {metrics_auto['sharpness']:.1f}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame_auto, f"Bright: {metrics_auto['brightness']:.1f}", (10, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.putText(frame_manual, "MANUAL Mode (Right Camera)", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame_manual, f"Sharp: {metrics_manual['sharpness']:.1f}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame_manual, f"Bright: {metrics_manual['brightness']:.1f}", (10, 85),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        combined = np.hstack([frame_auto, frame_manual])
        cv2.imshow('Comparison', combined)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord(' '):
            print("\n" + "="*60)
            print(f"AUTO:   Sharp={metrics_auto['sharpness']:6.1f}  Bright={metrics_auto['brightness']:5.1f}")
            print(f"MANUAL: Sharp={metrics_manual['sharpness']:6.1f}  Bright={metrics_manual['brightness']:5.1f}")
            print("="*60)

    cap_auto.release()
    cap_manual.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
