#!/usr/bin/env python3
"""
Generate Synthetic Stereo Calibration Images for Testing

สร้างภาพ stereo calibration จำลอง (perfect synthetic data) เพื่อทดสอบ
stereo_calibration.py algorithm โดยไม่พึ่ง hardware

Features:
- Known ground truth camera parameters
- Perfect asymmetric circles pattern
- No lens distortion (or controllable distortion)
- Multiple poses (distances, angles)
- Saves ground_truth.yaml for comparison

Usage:
    python3 generate_synthetic_calibration.py

Then test:
    python3 stereo_calibration.py (will use synthetic data)
    Compare results with ground_truth.yaml
"""

import numpy as np
import cv2
import yaml
import os
from datetime import datetime


class SyntheticStereoCalibrationGenerator:
    """Generate synthetic stereo calibration images with known ground truth"""

    def __init__(
        self,
        image_width=1280,
        image_height=720,
        focal_length=750.0,
        baseline_mm=60.0,
        pattern_rows=5,
        pattern_cols=6,
        spacing_mm=18.0,
        circle_diameter_mm=14.0,
        distortion_coeffs=None
    ):
        """
        Initialize synthetic calibration generator

        Args:
            image_width: Image width in pixels
            image_height: Image height in pixels
            focal_length: Focal length in pixels (realistic for IMX219)
            baseline_mm: Baseline distance between cameras in mm
            pattern_rows: Number of rows in asymmetric circles pattern
            pattern_cols: Number of columns in pattern
            spacing_mm: Diagonal spacing between circles in mm
            circle_diameter_mm: Circle diameter in mm
            distortion_coeffs: Lens distortion [k1, k2, p1, p2, k3] or None for perfect lens
        """

        self.image_width = image_width
        self.image_height = image_height
        self.image_size = (image_width, image_height)

        # Camera intrinsics (KNOWN GROUND TRUTH)
        self.focal_length = focal_length
        self.cx = image_width / 2.0
        self.cy = image_height / 2.0

        self.camera_matrix = np.array([
            [focal_length, 0, self.cx],
            [0, focal_length, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)

        # Distortion coefficients (KNOWN GROUND TRUTH)
        if distortion_coeffs is None:
            # Perfect lens (no distortion)
            self.dist_coeffs = np.zeros(5, dtype=np.float32)
        else:
            self.dist_coeffs = np.array(distortion_coeffs, dtype=np.float32)

        # Stereo extrinsics (KNOWN GROUND TRUTH)
        self.baseline_mm = baseline_mm

        # Cameras are parallel (no rotation)
        self.R = np.eye(3, dtype=np.float32)

        # Translation: right camera is baseline_mm to the right of left camera
        self.T = np.array([[baseline_mm], [0], [0]], dtype=np.float32)

        # Pattern parameters
        self.pattern_rows = pattern_rows
        self.pattern_cols = pattern_cols
        self.spacing_mm = spacing_mm
        self.circle_diameter_mm = circle_diameter_mm

        # Generate 3D pattern points (KNOWN GROUND TRUTH)
        self.pattern_points_3d = self._generate_pattern_points()

        print("=" * 70)
        print("Synthetic Stereo Calibration Generator")
        print("=" * 70)
        print(f"\nCamera Parameters (GROUND TRUTH):")
        print(f"  Image size: {image_width} × {image_height}")
        print(f"  Focal length: {focal_length:.2f} pixels")
        print(f"  Principal point: ({self.cx:.2f}, {self.cy:.2f})")
        print(f"  Baseline: {baseline_mm:.2f} mm")
        print(f"  Distortion: {self.dist_coeffs.tolist()}")
        print(f"\nPattern Parameters:")
        print(f"  Type: Asymmetric Circles Grid")
        print(f"  Size: {pattern_rows} × {pattern_cols}")
        print(f"  Spacing: {spacing_mm:.2f} mm")
        print(f"  Circle diameter: {circle_diameter_mm:.2f} mm")
        print("=" * 70)

    def _generate_pattern_points(self):
        """
        Generate 3D coordinates of asymmetric circles pattern
        Same formula as stereo_calibration.py:51-62

        Returns:
            np.array: (N, 3) array of 3D points in pattern coordinate system
        """
        objp = np.zeros((self.pattern_rows * self.pattern_cols, 3), np.float32)

        # Asymmetric grid pattern positions
        for i in range(self.pattern_rows):
            for j in range(self.pattern_cols):
                objp[i * self.pattern_cols + j] = [
                    (2 * j + i % 2) * self.spacing_mm,
                    i * self.spacing_mm,
                    0
                ]

        return objp

    def _pose_to_transform(self, distance_mm, tilt_x_deg, tilt_y_deg, tilt_z_deg=0):
        """
        Convert pose parameters to rotation and translation vectors

        Args:
            distance_mm: Distance from camera to pattern center in mm
            tilt_x_deg: Tilt around X-axis in degrees (pitch)
            tilt_y_deg: Tilt around Y-axis in degrees (yaw)
            tilt_z_deg: Tilt around Z-axis in degrees (roll)

        Returns:
            rvec, tvec: Rotation and translation vectors for cv2.projectPoints
        """
        # Rotation matrix from Euler angles
        rx = np.radians(tilt_x_deg)
        ry = np.radians(tilt_y_deg)
        rz = np.radians(tilt_z_deg)

        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])

        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])

        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])

        # Combined rotation
        R = Rz @ Ry @ Rx

        # Convert to rotation vector
        rvec, _ = cv2.Rodrigues(R)

        # Translation: pattern center is at distance_mm from camera
        # Place pattern center in front of camera
        tvec = np.array([[0], [0], [distance_mm]], dtype=np.float32)

        return rvec, tvec

    def _project_points(self, rvec, tvec, camera_offset=np.array([[0], [0], [0]])):
        """
        Project 3D pattern points to 2D image coordinates

        Args:
            rvec: Rotation vector (pattern pose)
            tvec: Translation vector (pattern pose)
            camera_offset: Camera position offset (for stereo)

        Returns:
            np.array: (N, 1, 2) array of 2D image points
        """
        # Adjust tvec for camera offset (stereo)
        tvec_adjusted = tvec - camera_offset

        # Project 3D points to 2D
        points_2d, _ = cv2.projectPoints(
            self.pattern_points_3d,
            rvec,
            tvec_adjusted,
            self.camera_matrix,
            self.dist_coeffs
        )

        return points_2d

    def _render_circles(self, points_2d):
        """
        Render circles on white background

        Args:
            points_2d: (N, 1, 2) array of 2D image points

        Returns:
            np.array: Rendered image (grayscale)
        """
        # Create white background
        img = np.ones((self.image_height, self.image_width), dtype=np.uint8) * 255

        # Calculate circle radius in pixels (approximate)
        # Assume pattern is at ~400mm distance
        # circle_diameter_mm / distance_mm * focal_length
        avg_distance = 400.0  # mm (approximate)
        radius_px = int((self.circle_diameter_mm / avg_distance) * self.focal_length * 0.5)
        radius_px = max(5, min(radius_px, 30))  # Clamp to reasonable range

        # Draw black circles
        for pt in points_2d:
            center = tuple(pt[0].astype(int))

            # Check if circle is within image bounds
            if (0 <= center[0] < self.image_width and
                0 <= center[1] < self.image_height):
                cv2.circle(img, center, radius_px, 0, -1, lineType=cv2.LINE_AA)

        return img

    def generate_image_pair(self, distance_mm, tilt_x_deg, tilt_y_deg, tilt_z_deg=0):
        """
        Generate a stereo image pair for a given pose

        Args:
            distance_mm: Distance from camera to pattern center
            tilt_x_deg: Tilt around X-axis (pitch)
            tilt_y_deg: Tilt around Y-axis (yaw)
            tilt_z_deg: Tilt around Z-axis (roll)

        Returns:
            img_left, img_right: Pair of rendered images
        """
        # Get pattern pose
        rvec, tvec = self._pose_to_transform(distance_mm, tilt_x_deg, tilt_y_deg, tilt_z_deg)

        # Project to left camera (at origin)
        points_2d_left = self._project_points(rvec, tvec, camera_offset=np.array([[0], [0], [0]]))

        # Project to right camera (offset by baseline)
        points_2d_right = self._project_points(rvec, tvec, camera_offset=self.T)

        # Render images
        img_left = self._render_circles(points_2d_left)
        img_right = self._render_circles(points_2d_right)

        return img_left, img_right

    def generate_dataset(self, num_poses=30, output_dir='calib_images/synthetic'):
        """
        Generate a full dataset of stereo calibration images

        Args:
            num_poses: Number of image pairs to generate
            output_dir: Output directory for images

        Returns:
            list: List of generated poses (for documentation)
        """
        # Create output directories
        left_dir = os.path.join(output_dir, 'left')
        right_dir = os.path.join(output_dir, 'right')
        os.makedirs(left_dir, exist_ok=True)
        os.makedirs(right_dir, exist_ok=True)

        print(f"\nGenerating {num_poses} synthetic calibration image pairs...")
        print(f"Output directory: {output_dir}")
        print()

        # Generate diverse poses
        poses = []

        # Define pose ranges
        distances = np.linspace(300, 500, 6)  # 300-500mm
        tilts_x = np.linspace(-15, 15, 5)     # -15 to +15 degrees
        tilts_y = np.linspace(-15, 15, 5)     # -15 to +15 degrees
        tilts_z = np.linspace(-10, 10, 3)     # -10 to +10 degrees

        # Generate combinations
        idx = 0
        for dist in distances:
            for tx in tilts_x:
                for ty in tilts_y:
                    for tz in tilts_z:
                        if idx >= num_poses:
                            break

                        # Generate image pair
                        img_left, img_right = self.generate_image_pair(dist, tx, ty, tz)

                        # Save images
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                        left_filename = os.path.join(left_dir, f"img_{idx:03d}_{timestamp}.jpg")
                        right_filename = os.path.join(right_dir, f"img_{idx:03d}_{timestamp}.jpg")

                        cv2.imwrite(left_filename, img_left)
                        cv2.imwrite(right_filename, img_right)

                        # Record pose
                        poses.append({
                            'index': idx,
                            'distance_mm': float(dist),
                            'tilt_x_deg': float(tx),
                            'tilt_y_deg': float(ty),
                            'tilt_z_deg': float(tz)
                        })

                        print(f"  ✓ Generated pair {idx:03d}: "
                              f"dist={dist:.0f}mm, "
                              f"tilt=({tx:+.1f}°, {ty:+.1f}°, {tz:+.1f}°)")

                        idx += 1

                    if idx >= num_poses:
                        break
                if idx >= num_poses:
                    break
            if idx >= num_poses:
                break

        print(f"\n✓ Generated {len(poses)} image pairs")

        return poses

    def save_ground_truth(self, poses, output_file='ground_truth.yaml'):
        """
        Save ground truth parameters to YAML file

        Args:
            poses: List of generated poses
            output_file: Output YAML file path
        """
        ground_truth = {
            'description': 'Synthetic Stereo Calibration Ground Truth',
            'generated_date': datetime.now().isoformat(),
            'image_width': int(self.image_width),
            'image_height': int(self.image_height),
            'pattern_type': 'asymmetric_circles',
            'pattern_rows': int(self.pattern_rows),
            'pattern_cols': int(self.pattern_cols),
            'pattern_spacing_mm': float(self.spacing_mm),
            'circle_diameter_mm': float(self.circle_diameter_mm),
            'baseline_mm': float(self.baseline_mm),
            'camera_matrix': self.camera_matrix.tolist(),
            'distortion_coefficients': self.dist_coeffs.tolist(),
            'rotation_matrix': self.R.tolist(),
            'translation_vector': self.T.tolist(),
            'poses': poses
        }

        with open(output_file, 'w') as f:
            yaml.dump(ground_truth, f, default_flow_style=False, sort_keys=False)

        print(f"\n✓ Saved ground truth to: {output_file}")
        print("\nGround Truth Summary:")
        print(f"  Baseline: {self.baseline_mm:.2f} mm")
        print(f"  Focal length: {self.focal_length:.2f} px")
        print(f"  Principal point: ({self.cx:.2f}, {self.cy:.2f})")
        print(f"  Distortion: {self.dist_coeffs.tolist()}")


def main():
    """Generate synthetic stereo calibration dataset"""

    # Create generator with realistic IMX219 parameters
    generator = SyntheticStereoCalibrationGenerator(
        image_width=1280,
        image_height=720,
        focal_length=750.0,      # Realistic for IMX219
        baseline_mm=60.0,         # KNOWN BASELINE (ground truth)
        pattern_rows=5,
        pattern_cols=6,
        spacing_mm=18.0,          # CONFIRMED spacing
        circle_diameter_mm=14.0,
        distortion_coeffs=None    # Perfect lens (no distortion)
    )

    # Generate 30 image pairs
    poses = generator.generate_dataset(num_poses=30, output_dir='calib_images/synthetic')

    # Save ground truth
    generator.save_ground_truth(poses, output_file='ground_truth.yaml')

    print("\n" + "=" * 70)
    print("Synthetic Calibration Data Generation Complete!")
    print("=" * 70)
    print("\nNext steps:")
    print("  1. Run: python3 stereo_calibration.py")
    print("     (Modify to use 'calib_images/synthetic/left/*.jpg')")
    print()
    print("  2. Compare results with ground_truth.yaml:")
    print("     - Expected baseline: 60.0 mm")
    print("     - Expected focal length: ~750 px")
    print("     - Expected RMS error: < 0.5 pixels (perfect data)")
    print()
    print("  3. If calibration results match ground truth:")
    print("     → stereo_calibration.py works correctly!")
    print("     → Problem is with real captured images (focus, spacing, etc.)")
    print()
    print("  4. If calibration results DON'T match:")
    print("     → Bug in stereo_calibration.py algorithm")
    print("=" * 70)


if __name__ == '__main__':
    main()
