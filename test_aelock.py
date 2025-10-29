#!/usr/bin/env python3
"""
🔒 Test Auto-Exposure Lock (aelock)

ทดสอบว่า aelock=true แก้ปัญหา focus เปลี่ยนเมื่อมือเข้า-ออกหรือไม่
"""
import cv2
import numpy as np
import time

def calculate_sharpness(image):
    """Calculate sharpness using Laplacian variance"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()

def build_pipeline(sensor_id, use_aelock=True, exposure_ms=24, gain=2):
    """Build pipeline with/without aelock"""
    exposure_ns = int(exposure_ms * 1000000)

    aelock_param = "aelock=true " if use_aelock else ""

    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"wbmode=0 "
        f"{aelock_param}"
        f'exposuretimerange="{exposure_ns} {exposure_ns}" '
        f'gainrange="{gain} {gain}" '
        f"! "
        f"video/x-raw(memory:NVMM), "
        f"width=(int)1280, height=(int)720, "
        f"format=(string)NV12, framerate=(fraction)15/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink"
    )

def main():
    print("="*80)
    print("🔒 AUTO-EXPOSURE LOCK TEST")
    print("="*80)
    print("\nทดสอบว่า aelock=true ช่วยแก้ปัญหา focus หรือไม่")
    print("\nขั้นตอน:")
    print("1. วางพริกกลางเฟรม")
    print("2. เอามือเข้า-ออกเฟรมหลายๆ ครั้ง")
    print("3. สังเกตค่า Sharpness")
    print("   - ถ้าค่าคงที่ = aelock ทำงาน ✅")
    print("   - ถ้าค่าเปลี่ยนมาก = aelock ไม่ช่วย ❌")
    print("\nControls:")
    print("  SPACE - Toggle monitoring")
    print("  Q     - Quit")
    print("="*80)

    # Open camera WITH aelock
    print("\n📷 Opening camera with aelock=true...")
    pipeline = build_pipeline(sensor_id=0, use_aelock=True)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print("❌ Error: Cannot open camera")
        print("\n💡 Trying without aelock...")
        pipeline = build_pipeline(sensor_id=0, use_aelock=False)
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("❌ Still failed. Check camera connection.")
            return

    print("✅ Camera opened")
    print("\n⏳ Warming up (3 seconds)...")

    for i in range(3, 0, -1):
        ret, _ = cap.read()
        print(f"   {i}...", end="\r")
        time.sleep(1)

    print("\n✅ Ready!\n")
    print("📊 Instructions:")
    print("   1. Press SPACE to start monitoring")
    print("   2. Move your hand in/out of frame multiple times")
    print("   3. Watch the Sharpness value")
    print("   4. Press SPACE again to see summary\n")

    monitoring = False
    sharpness_history = []
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        sharpness = calculate_sharpness(frame)

        if monitoring:
            sharpness_history.append(sharpness)
            frame_count += 1

            # Real-time stats
            if len(sharpness_history) > 1:
                avg = np.mean(sharpness_history)
                std = np.std(sharpness_history)
                variation = (std / avg * 100) if avg > 0 else 0

                status = "✅ STABLE" if variation < 10 else "⚠️ CHANGING"

                print(f"\r[{frame_count:3d}] Sharp: {sharpness:6.1f} | "
                      f"Avg: {avg:6.1f} | Var: {variation:5.1f}% | {status}  ",
                      end="", flush=True)

        # Display
        display = frame.copy()
        h, w = display.shape[:2]

        cv2.putText(display, f"Sharpness: {sharpness:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        if monitoring:
            cv2.putText(display, "MONITORING - Move hand in/out", (10, h-60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display, "Press SPACE to stop", (10, h-30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Press SPACE to start monitoring", (10, h-30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("AELock Test", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            if not monitoring:
                print("\n🎬 Started monitoring. Move your hand in/out!\n")
                monitoring = True
                sharpness_history = []
                frame_count = 0
            else:
                print("\n\n⏸️  Stopped.\n")
                monitoring = False

                if len(sharpness_history) > 10:
                    avg = np.mean(sharpness_history)
                    std = np.std(sharpness_history)
                    variation = (std / avg * 100)

                    print("="*80)
                    print("📊 SUMMARY")
                    print("="*80)
                    print(f"Frames: {len(sharpness_history)}")
                    print(f"Avg Sharpness: {avg:.1f}")
                    print(f"Std Deviation: {std:.1f}")
                    print(f"Variation: {variation:.1f}%\n")

                    if variation < 5:
                        print("✅ EXCELLENT! Focus is very stable (< 5%)")
                        print("   → aelock=true is working perfectly!")
                    elif variation < 10:
                        print("✅ GOOD! Focus is stable (5-10%)")
                        print("   → aelock=true helps significantly")
                    elif variation < 20:
                        print("⚠️  MODERATE. Some variation (10-20%)")
                        print("   → aelock helps but not perfect")
                    else:
                        print("❌ UNSTABLE. High variation (> 20%)")
                        print("   → aelock may not be working")
                        print("\n💡 Possible issues:")
                        print("   - Camera driver doesn't support aelock")
                        print("   - Need to wait longer after placing object")
                        print("   - Camera has mechanical auto-focus")

                    print("="*80 + "\n")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
