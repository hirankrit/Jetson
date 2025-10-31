#!/usr/bin/env python3
"""
Pepper Dataset Collection Tool
Collect FULL stereo dataset (Left + Right + Depth) for YOLO training

Features:
- Stereo camera capture (left + right)
- Depth map computation (StereoSGBM)
- Optimized camera settings (exposure=30ms, gain=2, aelock=true)
- Real-time preview with statistics
- Organized folder structure

Controls:
  SPACE - Capture image (with 3s countdown)
  'q' - Quit and show summary
"""

import cv2
import numpy as np
import os
import yaml
from datetime import datetime
import argparse
import time


def create_hardware_config(args, baseline, focal_length):
    """
    Create comprehensive hardware configuration for reproducibility
    Includes all 7 categories of camera parameters
    """
    config = {
        "metadata_version": "1.0",
        "creation_date": datetime.now().isoformat(),
        "description": "Hardware configuration for pepper dataset collection",
        # ========== 1. Exposure & Light Sensitivity ==========
        "exposure_and_light": {
            "exposure_time_ms": args.exposure,
            "exposure_time_ns": args.exposure * 1000000,
            "gain_iso": args.gain,
            "auto_exposure": False,
            "ae_lock": True,
            "notes": "MANUAL mode with fixed exposure/gain to prevent flickering",
        },
        # ========== 2. White Balance & Color ==========
        "white_balance_and_color": {
            "white_balance_mode": "manual",  # wbmode=0
            "awb_lock": True,
            "color_temperature_k": None,  # Not set (using manual mode)
            "saturation_percent": None,  # Default
            "hue_shift_deg": 0,  # Default
            "notes": "Manual white balance to ensure color consistency",
        },
        # ========== 3. Image Enhancement ==========
        "image_enhancement": {
            "brightness": None,  # Default (not modified)
            "contrast": None,  # Default (not modified)
            "gamma": None,  # Default (not modified)
            "sharpness": None,  # Default (not modified)
            "notes": "Using GStreamer defaults, no post-processing applied",
        },
        # ========== 4. Noise & Dynamic Range ==========
        "noise_and_dynamic_range": {
            "denoise": None,  # Default
            "temporal_noise_reduction": None,  # Default
            "backlight_compensation": False,
            "hdr": False,
            "notes": "Standard dynamic range, no special noise reduction",
        },
        # ========== 5. Focus & Aperture ==========
        "focus_and_aperture": {
            "focus_mode": "manual",
            "focus_value_left": 176.5,
            "focus_value_right": 171.0,
            "focus_difference": 5.5,
            "aperture": "fixed",  # IMX219 has fixed aperture
            "notes": "Focus manually adjusted and locked before collection",
        },
        # ========== 6. Frame & Timing ==========
        "frame_and_timing": {
            "frame_rate_fps": 15,
            "resolution_width": args.width,
            "resolution_height": args.height,
            "pixel_format": "NV12 -> BGRx -> BGR",
            "ae_antibanding_hz": None,  # Not set
            "notes": "15 FPS for stable capture with 3s countdown",
        },
        # ========== 7. External Lighting ==========
        "external_lighting": {
            "led_count": 3,
            "led_type": "White LED strips",
            "led_positions": [
                {
                    "name": "Top LED",
                    "position": "overhead",
                    "distance_from_target_cm": 10,
                    "angle_deg": 0,
                    "notes": "Mounted above camera, reduces shadows",
                },
                {
                    "name": "Left LED",
                    "position": "left-front diagonal",
                    "distance_from_target_cm": 10,
                    "angle_deg": 45,
                    "notes": "Diagonal lighting from left side",
                },
                {
                    "name": "Right LED",
                    "position": "right-front diagonal",
                    "distance_from_target_cm": 10,
                    "angle_deg": 45,
                    "notes": "Diagonal lighting from right side",
                },
            ],
            "ambient_light": "indoor",
            "notes": "3-point LED setup optimized for even illumination",
        },
        # ========== Hardware Setup ==========
        "hardware_setup": {
            "camera_model": "IMX219-83 Stereo Camera",
            "sensor_resolution": "8MP (3280x2464)",
            "fov_deg": 160,
            "baseline_mm": baseline,
            "focal_length_px": focal_length,
            "camera_height_mm": 320,
            "camera_angle": "top-down view (perpendicular to surface)",
            "mounting": "fixed tripod mount",
        },
        # ========== Environment ==========
        "environment": {
            "background": "gray cloth",
            "background_material": "non-reflective fabric",
            "surface": "flat table with gray cloth",
            "working_distance_cm": "23-35",
            "temperature_c": None,  # Not measured
            "humidity_percent": None,  # Not measured
            "notes": "Controlled indoor environment with gray background",
        },
        # ========== GStreamer Pipeline ==========
        "gstreamer_pipeline": {
            "source": "nvarguscamerasrc",
            "wbmode": 0,
            "aelock": True,
            "flip_method": 0,
            "full_pipeline": (
                f"nvarguscamerasrc sensor-id=<ID> "
                f"wbmode=0 aelock=true "
                f'exposuretimerange="<EXPOSURE> <EXPOSURE>" '
                f'gainrange="<GAIN> <GAIN>" ! '
                f"video/x-raw(memory:NVMM), width=<W>, height=<H>, "
                f"format=NV12, framerate=15/1 ! "
                f"nvvidconv flip-method=0 ! "
                f"video/x-raw, format=BGRx ! videoconvert ! "
                f"video/x-raw, format=BGR ! appsink"
            ),
        },
        # ========== Stereo Calibration ==========
        "stereo_calibration": {
            "calibration_file": "stereo_calib.yaml",
            "calibration_date": None,  # Will be read from file if available
            "pattern_type": "Asymmetric Circles Grid (5x6, 33 circles)",
            "pattern_spacing_mm": 18,
            "baseline_measured_mm": baseline,
            "focal_length_px": focal_length,
        },
        # ========== Dataset Information ==========
        "dataset_info": {
            "session_name": os.path.basename(args.output),
            "output_directory": args.output,
            "save_mode": "full (left + right + depth)",
            "depth_computation": "StereoSGBM",
            "depth_range_mm": "150-1200",
        },
    }
    return config


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
    print("🌶️  PEPPER DATASET COLLECTION TOOL - FULL MODE")
    print("=" * 80)
    print(f"\nOutput directory: {output_dir}")
    print(f"Resolution: {args.width}x{args.height}")
    print("\n⚙️  Camera Settings (MANUAL mode - Optimized):")
    print(f"  Exposure: {args.exposure}ms (fixed)")
    print(f"  Gain: {args.gain} (fixed)")
    print("  White Balance: Manual")
    print("  Auto-Exposure Lock: Enabled")
    print("\n📊 Save Mode:")
    print("  ✅ Left + Right + Depth (Full stereo dataset)")
    print("\n⌨️  Controls:")
    print("  SPACE - Capture image (3s countdown for stable focus)")
    print("  'q'   - Quit and show summary")
    print("=" * 80)

    # Load calibration for depth (required for Mode 3)
    calib_available = False
    try:
        with open("stereo_calib.yaml", "r") as f:
            calib = yaml.safe_load(f)
        _ = np.load("rectification_maps.npz")  # Not used but validates file exists
        baseline = calib["baseline_mm"]
        Q = np.array(calib["stereo"]["Q_matrix"])
        focal_length = abs(Q[2, 3])
        calib_available = True
        print("\n✅ Calibration loaded successfully!")
        print(f"   Baseline: {baseline:.2f}mm, Focal: {focal_length:.2f}px")
    except Exception as e:
        print("\n❌ ERROR: Calibration not found!")
        print("   Depth computation requires stereo_calib.yaml")
        print(f"   Error: {e}")
        return

    # Save hardware configuration
    print("\n💾 Saving hardware configuration...")
    hardware_config = create_hardware_config(args, baseline, focal_length)
    hardware_config_path = f"{output_dir}/metadata/hardware_config.yaml"
    with open(hardware_config_path, "w") as f:
        yaml.dump(hardware_config, f, default_flow_style=False, sort_keys=False)
    print(f"   ✅ Saved: {hardware_config_path}")
    print("   📋 Complete camera parameters logged (7 categories)")

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
    print("\n👉 Press SPACE to start capturing...")

    # State variables
    count = 0
    save_mode = 3  # Fixed: Always save left+right+depth
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
            "Dataset Collection - FULL MODE",
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

        cv2.putText(
            display,
            "Mode: Left + Right + Depth",
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
        )

        cv2.putText(
            display,
            "Press SPACE to capture (3s countdown), 'q' to quit",
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

            # Save left image
            left_path = f"{output_dir}/raw/left/{filename_base}.jpg"
            cv2.imwrite(left_path, frame_left)
            print(f"  ✓ Left: {left_path}")

            # Save right image
            right_path = f"{output_dir}/raw/right/{filename_base}.jpg"
            cv2.imwrite(right_path, frame_right)
            print(f"  ✓ Right: {right_path}")

            # Compute and save depth
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

            # Metadata
            meta = {
                "id": count,
                "timestamp": timestamp,
                "mode": 3,
                "resolution": f"{args.width}x{args.height}",
                "exposure_ms": args.exposure,
                "gain": args.gain,
                "depth_coverage": f"{coverage:.1f}%",
                "files": {
                    "left": f"raw/left/{filename_base}.jpg",
                    "right": f"raw/right/{filename_base}.jpg",
                    "depth": f"raw/depth/{filename_base}.jpg",
                },
            }

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
    print("\nFiles saved (Full stereo dataset):")
    print(f"  - Left images:  {count}")
    print(f"  - Right images: {count}")
    print(f"  - Depth maps:   {count}")
    print(f"  - Total files:  {count * 3}")

    print("\n🎯 Next steps:")
    print("  1. Review collected images")
    print("  2. Collect more if needed (target: 500-1000 images)")
    print("  3. Annotate using Roboflow or LabelImg")
    print("  4. Prepare for YOLO training (Week 3)")
    print("=" * 80)


if __name__ == "__main__":
    main()
