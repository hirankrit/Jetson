#!/usr/bin/env python3
"""
Pepper Dataset Collection Tool
Collect images for YOLO training using stereo camera

Features:
- Use left camera for primary dataset
- Optimized camera settings (exposure=30ms, gain=2)
- Optional: Save right image and depth for future use
- Real-time preview with statistics
- Organized folder structure

Controls:
  SPACE - Capture image (with 3s countdown)
  's' - Toggle save mode (left only / left+right / left+right+depth)
  'q' - Quit and show summary
"""

import cv2
import numpy as np
import os
import yaml
from datetime import datetime
import argparse
import time


def build_gstreamer_pipeline(
    sensor_id, width=1280, height=720, framerate=15, exposure_ms=30, gain=2
):
    """
    Build GStreamer pipeline with optimized settings

    MANUAL mode with fixed exposure/gain (prevents flickering)
    """
    exposure_ns = int(exposure_ms * 1000000)

    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"wbmode=0 "
        f"aelock=true "
        f'exposuretimerange="{exposure_ns} {exposure_ns}" '
        f'gainrange="{gain} {gain}" '
        f"! "
        f"video/x-raw(memory:NVMM), "
        f"width=(int){width}, height=(int){height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink"
    )


def compute_depth_map(left_gray, right_gray, baseline, focal):
    """
    Compute depth map (optional for dataset metadata)
    """
    window_size = 5
    min_disp = 0
    num_disp = 512

    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=8 * 3 * window_size**2,
        P2=32 * 3 * window_size**2,
        disp12MaxDiff=2,
        uniquenessRatio=12,
        speckleWindowSize=120,
        speckleRange=16,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )

    disparity = stereo.compute(left_gray, right_gray)
    disparity_float = disparity.astype(np.float32) / 16.0

    valid_mask = (disparity_float > min_disp) & (disparity_float < num_disp)

    depth_map = np.zeros_like(disparity_float)
    depth_map[valid_mask] = (baseline * focal) / disparity_float[valid_mask]

    realistic_mask = (depth_map >= 150) & (depth_map <= 1200)
    final_mask = valid_mask & realistic_mask

    return depth_map, final_mask


def main():
    parser = argparse.ArgumentParser(description="Collect pepper dataset")
    parser.add_argument(
        "--output", type=str, default="pepper_dataset", help="Output directory"
    )
    parser.add_argument(
        "--width", type=int, default=1280, help="Image width (default: 1280)"
    )
    parser.add_argument(
        "--height", type=int, default=720, help="Image height (default: 720)"
    )
    parser.add_argument(
        "--exposure", type=int, default=30, help="Exposure in ms (default: 30)"
    )
    parser.add_argument("--gain", type=float, default=2, help="Gain 1-16 (default: 2)")

    args = parser.parse_args()

    # Create output directories
    output_dir = args.output
    os.makedirs(f"{output_dir}/raw/left", exist_ok=True)
    os.makedirs(f"{output_dir}/raw/right", exist_ok=True)
    os.makedirs(f"{output_dir}/raw/depth", exist_ok=True)
    os.makedirs(f"{output_dir}/metadata", exist_ok=True)

    print("=" * 80)
    print("🌶️  PEPPER DATASET COLLECTION TOOL")
    print("=" * 80)
    print(f"\nOutput directory: {output_dir}")
    print(f"Resolution: {args.width}x{args.height}")
    print("\n⚙️  Camera Settings (MANUAL mode - Optimized):")
    print(f"  Exposure: {args.exposure}ms (fixed)")
    print(f"  Gain: {args.gain} (fixed)")
    print("  White Balance: Manual")
    print("\n📊 Save Modes:")
    print("  Mode 1: Left only (for YOLO training) - Default")
    print("  Mode 2: Left + Right (stereo pair)")
    print("  Mode 3: Left + Right + Depth (full data)")
    print("\n⌨️  Controls:")
    print("  SPACE - Capture image (3s countdown for stable focus)")
    print("  's'   - Toggle save mode (1/2/3)")
    print("  'q'   - Quit and show summary")
    print("=" * 80)

    # Load calibration for depth (optional)
    calib_available = False
    try:
        with open("stereo_calib.yaml", "r") as f:
            calib = yaml.safe_load(f)
        _ = np.load("rectification_maps.npz")  # Not used but validates file exists
        baseline = calib["baseline_mm"]
        Q = np.array(calib["stereo"]["Q_matrix"])
        focal_length = abs(Q[2, 3])
        calib_available = True
        print("\n✓ Calibration loaded (depth computation available)")
    except Exception:
        print("\n⚠️  Calibration not found (depth computation disabled)")
        print("   Only left/right images will be saved")

    # Setup cameras
    left_pipeline = build_gstreamer_pipeline(
        0, args.width, args.height, exposure_ms=args.exposure, gain=args.gain
    )
    right_pipeline = build_gstreamer_pipeline(
        1, args.width, args.height, exposure_ms=args.exposure, gain=args.gain
    )

    print("\n📷 Opening cameras...")
    left_cap = cv2.VideoCapture(left_pipeline, cv2.CAP_GSTREAMER)
    if not left_cap.isOpened():
        print("❌ ERROR: Left camera failed")
        return

    import time

    time.sleep(2)

    right_cap = cv2.VideoCapture(right_pipeline, cv2.CAP_GSTREAMER)
    if not right_cap.isOpened():
        print("❌ ERROR: Right camera failed")
        left_cap.release()
        return

    print("✓ Cameras ready")
    print("\n👉 Press 'c' to start capturing...")

    # State variables
    count = 0
    save_mode = 1  # 1=left only, 2=left+right, 3=left+right+depth
    metadata_log = []

    cv2.namedWindow("Dataset Collection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Dataset Collection", 1280, 720)

    while True:
        ret_left, frame_left = left_cap.read()
        ret_right, frame_right = right_cap.read()

        if not ret_left or not ret_right:
            print("⚠️  Failed to read frames")
            break

        # Display preview
        display = frame_left.copy()
        cv2.putText(
            display,
            f"Dataset Collection - Mode {save_mode}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            display,
            f"Captured: {count}",
            (10, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )

        mode_text = {
            1: "Left only",
            2: "Left + Right",
            3: "Left + Right + Depth",
        }
        cv2.putText(
            display,
            f"Mode: {mode_text[save_mode]}",
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
        )

        cv2.putText(
            display,
            "Press SPACE to capture (3s countdown), 's' to change mode, 'q' to quit",
            (10, args.height - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        cv2.imshow("Dataset Collection", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("s"):
            # Toggle save mode
            if not calib_available and save_mode == 2:
                save_mode = 1
            elif not calib_available:
                save_mode = (save_mode % 2) + 1
            else:
                save_mode = (save_mode % 3) + 1
            print(f"\n🔄 Save mode changed to: {mode_text[save_mode]}")

        elif key == ord(" "):  # SPACE key
            # Countdown before capture (allows focus to stabilize)
            print(f"\n⏱️  Countdown: ", end="", flush=True)
            for i in range(3, 0, -1):
                print(f"{i}... ", end="", flush=True)
                time.sleep(1)
            print("📸")

            # Now capture
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            filename_base = f"pepper_{count:04d}_{timestamp}"

            print(f"\n📸 Capturing #{count + 1}...")

            # Save left image (always)
            left_path = f"{output_dir}/raw/left/{filename_base}.jpg"
            cv2.imwrite(left_path, frame_left)
            print(f"  ✓ Left: {left_path}")

            # Metadata
            meta = {
                "id": count,
                "timestamp": timestamp,
                "mode": save_mode,
                "resolution": f"{args.width}x{args.height}",
                "exposure_ms": args.exposure,
                "gain": args.gain,
                "files": {"left": f"raw/left/{filename_base}.jpg"},
            }

            # Save right image (mode 2, 3)
            if save_mode >= 2:
                right_path = f"{output_dir}/raw/right/{filename_base}.jpg"
                cv2.imwrite(right_path, frame_right)
                print(f"  ✓ Right: {right_path}")
                meta["files"]["right"] = f"raw/right/{filename_base}.jpg"

            # Compute and save depth (mode 3)
            if save_mode == 3 and calib_available:
                gray_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2GRAY)
                gray_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2GRAY)

                depth_map, mask = compute_depth_map(
                    gray_left, gray_right, baseline, focal_length
                )

                # Visualize depth
                depth_vis = depth_map.copy()
                depth_vis[~mask] = 0
                depth_vis = np.clip(depth_vis, 150, 1200)
                depth_vis = ((depth_vis - 150) / 1050 * 255).astype(np.uint8)
                depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                depth_colored[~mask] = [0, 0, 0]

                depth_path = f"{output_dir}/raw/depth/{filename_base}.jpg"
                cv2.imwrite(depth_path, depth_colored)
                print(f"  ✓ Depth: {depth_path}")

                coverage = np.sum(mask) / (args.width * args.height) * 100
                meta["depth_coverage"] = f"{coverage:.1f}%"
                meta["files"]["depth"] = f"raw/depth/{filename_base}.jpg"

            metadata_log.append(meta)
            count += 1

            print(f"  ✓ Total captured: {count}")

    # Cleanup
    left_cap.release()
    right_cap.release()
    cv2.destroyAllWindows()

    # Save metadata
    if count > 0:
        metadata_path = f"{output_dir}/metadata/collection_log.yaml"
        with open(metadata_path, "w") as f:
            yaml.dump(
                {
                    "total_images": count,
                    "collection_date": datetime.now().isoformat(),
                    "camera_settings": {
                        "resolution": f"{args.width}x{args.height}",
                        "exposure_ms": args.exposure,
                        "gain": args.gain,
                    },
                    "images": metadata_log,
                },
                f,
            )
        print(f"\n💾 Metadata saved: {metadata_path}")

    # Summary
    print("\n" + "=" * 80)
    print("📊 COLLECTION SUMMARY")
    print("=" * 80)
    print(f"Total images collected: {count}")
    print(f"Output directory: {output_dir}")
    print("\nFiles saved:")
    print(f"  - Left images:  {count}")
    if any(m["mode"] >= 2 for m in metadata_log):
        right_count = sum(1 for m in metadata_log if m["mode"] >= 2)
        print(f"  - Right images: {right_count}")
    if any(m["mode"] == 3 for m in metadata_log):
        depth_count = sum(1 for m in metadata_log if m["mode"] == 3)
        print(f"  - Depth maps:   {depth_count}")

    print("\n🎯 Next steps:")
    print("  1. Review collected images")
    print("  2. Collect more if needed (target: 500-1000 images)")
    print("  3. Annotate using Roboflow or LabelImg")
    print("  4. Prepare for YOLO training (Week 3)")
    print("=" * 80)


if __name__ == "__main__":
    main()
