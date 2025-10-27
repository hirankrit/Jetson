#!/usr/bin/env python3
"""
Generate Asymmetric Circles Calibration Pattern for Printing

สร้าง calibration pattern สำหรับพิมพ์ออกมาใช้งานจริง
- Pattern type: Asymmetric Circles Grid
- Rows: 5
- Columns: 6
- Diagonal spacing: 18mm (configurable)
- Circle diameter: 14mm (configurable)
- Output: PNG file at 300 DPI (สำหรับพิมพ์คุณภาพสูง)

Usage:
    python3 generate_calibration_pattern.py

Output:
    calibration_pattern_5x6_18mm.png
    calibration_pattern_info.txt
"""

import numpy as np
import cv2
import os


def mm_to_pixels(mm, dpi=300):
    """
    Convert millimeters to pixels at given DPI

    Args:
        mm: Size in millimeters
        dpi: Dots per inch (default 300 for high quality printing)

    Returns:
        int: Size in pixels
    """
    # 1 inch = 25.4 mm
    inches = mm / 25.4
    pixels = int(inches * dpi)
    return pixels


def generate_asymmetric_circles_pattern(
    rows=5,
    cols=6,
    spacing_mm=18.0,
    circle_diameter_mm=14.0,
    dpi=300,
    margin_mm=20.0,
    output_file='calibration_pattern.png'
):
    """
    Generate asymmetric circles calibration pattern

    Args:
        rows: Number of rows in pattern
        cols: Number of columns in pattern
        spacing_mm: Diagonal spacing between circle centers in mm
        circle_diameter_mm: Circle diameter in mm
        dpi: Dots per inch for output image
        margin_mm: Margin around pattern in mm
        output_file: Output PNG filename

    Returns:
        np.array: Generated pattern image
    """

    print("=" * 70)
    print("Asymmetric Circles Calibration Pattern Generator")
    print("=" * 70)
    print(f"\nPattern Parameters:")
    print(f"  Rows: {rows}")
    print(f"  Columns: {cols}")
    print(f"  Spacing: {spacing_mm} mm (diagonal)")
    print(f"  Circle diameter: {circle_diameter_mm} mm")
    print(f"  DPI: {dpi}")
    print(f"  Margin: {margin_mm} mm")
    print()

    # Convert measurements to pixels
    spacing_px = mm_to_pixels(spacing_mm, dpi)
    circle_radius_px = mm_to_pixels(circle_diameter_mm / 2, dpi)
    margin_px = mm_to_pixels(margin_mm, dpi)

    print(f"Pixel Conversions ({dpi} DPI):")
    print(f"  Spacing: {spacing_px} px")
    print(f"  Circle radius: {circle_radius_px} px")
    print(f"  Margin: {margin_px} px")
    print()

    # Calculate image size
    # Asymmetric pattern: last column is at x = (2 * (cols-1) + 0 or 1) * spacing
    max_x = (2 * (cols - 1) + 1) * spacing_px  # Row with offset
    max_y = (rows - 1) * spacing_px

    image_width = max_x + 2 * margin_px + 2 * circle_radius_px
    image_height = max_y + 2 * margin_px + 2 * circle_radius_px

    print(f"Image Size:")
    print(f"  Width: {image_width} px ({image_width / dpi * 25.4:.1f} mm)")
    print(f"  Height: {image_height} px ({image_height / dpi * 25.4:.1f} mm)")
    print()

    # Create white background
    img = np.ones((image_height, image_width), dtype=np.uint8) * 255

    # Calculate circle centers (same formula as stereo_calibration.py)
    print("Generating circle positions...")
    circles_drawn = 0

    for i in range(rows):
        for j in range(cols):
            # Asymmetric grid formula (same as stereo_calibration.py:58-62)
            x = (2 * j + i % 2) * spacing_px
            y = i * spacing_px

            # Add margin offset
            x += margin_px + circle_radius_px
            y += margin_px + circle_radius_px

            # Draw black circle
            center = (int(x), int(y))
            cv2.circle(img, center, circle_radius_px, 0, -1, lineType=cv2.LINE_AA)
            circles_drawn += 1

    print(f"✓ Drew {circles_drawn} circles")
    print()

    # Add border for easier cutting
    border_thickness = mm_to_pixels(1, dpi)
    cv2.rectangle(
        img,
        (border_thickness, border_thickness),
        (image_width - border_thickness, image_height - border_thickness),
        128,  # Gray border
        border_thickness
    )

    # Save as PNG
    cv2.imwrite(output_file, img)
    print(f"✓ Saved pattern to: {output_file}")

    # Calculate actual pattern size
    pattern_width_mm = (2 * (cols - 1) + 1) * spacing_mm + circle_diameter_mm
    pattern_height_mm = (rows - 1) * spacing_mm + circle_diameter_mm
    total_width_mm = pattern_width_mm + 2 * margin_mm
    total_height_mm = pattern_height_mm + 2 * margin_mm

    # Save info file
    info_file = output_file.replace('.png', '_info.txt')
    with open(info_file, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write("CALIBRATION PATTERN INFORMATION\n")
        f.write("=" * 70 + "\n\n")

        f.write("Pattern Type: Asymmetric Circles Grid\n\n")

        f.write("Pattern Specifications:\n")
        f.write(f"  Rows: {rows}\n")
        f.write(f"  Columns: {cols}\n")
        f.write(f"  Total circles: {rows * cols}\n")
        f.write(f"  Spacing (diagonal): {spacing_mm} mm\n")
        f.write(f"  Circle diameter: {circle_diameter_mm} mm\n\n")

        f.write("Image Specifications:\n")
        f.write(f"  DPI: {dpi}\n")
        f.write(f"  Width: {image_width} px ({total_width_mm:.1f} mm)\n")
        f.write(f"  Height: {image_height} px ({total_height_mm:.1f} mm)\n")
        f.write(f"  Margin: {margin_mm} mm\n\n")

        f.write("Printing Instructions:\n")
        f.write(f"  1. Print at EXACTLY {dpi} DPI (no scaling!)\n")
        f.write(f"  2. Use high-quality printer (laser or inkjet)\n")
        f.write(f"  3. Print on white paper or cardstock\n")
        f.write(f"  4. DO NOT enable 'Fit to Page' or 'Scale to Fit'\n")
        f.write(f"  5. Select 'Actual Size' or '100% scale'\n")
        f.write(f"  6. After printing, verify spacing with ruler/caliper:\n")
        f.write(f"     - Diagonal spacing should be {spacing_mm} mm\n")
        f.write(f"     - Circle diameter should be {circle_diameter_mm} mm\n\n")

        f.write("Mounting Instructions:\n")
        f.write(f"  1. Mount on rigid flat surface (foam board, acrylic, wood)\n")
        f.write(f"  2. Ensure pattern is completely flat (no warping)\n")
        f.write(f"  3. Use double-sided tape or spray adhesive\n\n")

        f.write("OpenCV Configuration:\n")
        f.write(f"  pattern_rows = {rows}\n")
        f.write(f"  pattern_cols = {cols}\n")
        f.write(f"  spacing_mm = {spacing_mm}\n")
        f.write(f"  flags = cv2.CALIB_CB_ASYMMETRIC_GRID\n\n")

        f.write("Pattern Layout (Top View):\n")
        f.write("  Row 0: • - • - • - • - • - •  (offset 0)\n")
        f.write("  Row 1: - • - • - • - • - • -  (offset +spacing/2)\n")
        f.write("  Row 2: • - • - • - • - • - •  (offset 0)\n")
        f.write("  Row 3: - • - • - • - • - • -  (offset +spacing/2)\n")
        f.write("  Row 4: • - • - • - • - • - •  (offset 0)\n\n")

        f.write("=" * 70 + "\n")

    print(f"✓ Saved info to: {info_file}")

    print("\n" + "=" * 70)
    print("PRINTING INSTRUCTIONS")
    print("=" * 70)
    print(f"\n1. Print Settings:")
    print(f"   - Paper size: A4 or Letter")
    print(f"   - DPI: {dpi} (EXACT!)")
    print(f"   - Scale: 100% / Actual Size")
    print(f"   - NO 'Fit to Page'")
    print(f"\n2. After Printing - VERIFY:")
    print(f"   - Measure diagonal spacing with ruler/caliper")
    print(f"   - Should be EXACTLY {spacing_mm} mm")
    print(f"   - If not, printer scaling is wrong!")
    print(f"\n3. Mounting:")
    print(f"   - Use rigid flat board (foam board recommended)")
    print(f"   - Ensure NO warping or bending")
    print(f"\n4. Usage:")
    print(f"   - Use with capture_calibration.py")
    print(f"   - Then stereo_calibration.py")
    print("=" * 70 + "\n")

    return img


def main():
    """Generate calibration patterns with different spacings"""

    # Generate 18mm pattern (current/confirmed)
    print("\nGenerating 18mm spacing pattern...")
    pattern_18mm = generate_asymmetric_circles_pattern(
        rows=5,
        cols=6,
        spacing_mm=18.0,
        circle_diameter_mm=14.0,
        dpi=300,
        margin_mm=20.0,
        output_file='calibration_pattern_5x6_18mm.png'
    )

    print("\n" + "=" * 70)

    # Also generate 16mm pattern (in case original was 16mm)
    print("\nGenerating 16mm spacing pattern (alternative)...")
    pattern_16mm = generate_asymmetric_circles_pattern(
        rows=5,
        cols=6,
        spacing_mm=16.0,
        circle_diameter_mm=14.0,
        dpi=300,
        margin_mm=20.0,
        output_file='calibration_pattern_5x6_16mm.png'
    )

    print("\n" + "=" * 70)
    print("PATTERN GENERATION COMPLETE")
    print("=" * 70)
    print("\nGenerated Files:")
    print("  1. calibration_pattern_5x6_18mm.png (current spacing)")
    print("  2. calibration_pattern_5x6_18mm_info.txt")
    print("  3. calibration_pattern_5x6_16mm.png (alternative)")
    print("  4. calibration_pattern_5x6_16mm_info.txt")
    print("\nNext Steps:")
    print("  1. Print ONE of these patterns at 300 DPI")
    print("  2. VERIFY spacing with ruler/caliper")
    print("  3. Mount on rigid board")
    print("  4. Use with capture_calibration.py")
    print("\nIMPORTANT:")
    print("  - After printing, MEASURE the actual spacing")
    print("  - Use that measured value in stereo_calibration.py")
    print("  - Common printer scaling: 96 DPI vs 72 DPI vs 300 DPI")
    print("=" * 70 + "\n")


if __name__ == '__main__':
    main()
