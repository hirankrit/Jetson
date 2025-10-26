#!/usr/bin/env python3
"""
Simple Pattern Generator (works without cv2/numpy)
Generates SVG pattern that can be converted to PNG
"""

def generate_svg_pattern(
    rows=5,
    cols=6,
    spacing_mm=18.0,
    circle_diameter_mm=14.0,
    margin_mm=20.0,
    output_file='calibration_pattern.svg'
):
    """Generate SVG calibration pattern"""

    circle_radius_mm = circle_diameter_mm / 2.0

    # Calculate image size
    max_x = (2 * (cols - 1) + 1) * spacing_mm
    max_y = (rows - 1) * spacing_mm

    width = max_x + 2 * margin_mm + circle_diameter_mm
    height = max_y + 2 * margin_mm + circle_diameter_mm

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg width="{width}mm" height="{height}mm" viewBox="0 0 {width} {height}"
     xmlns="http://www.w3.org/2000/svg">

    <!-- White background -->
    <rect width="{width}" height="{height}" fill="white"/>

    <!-- Border -->
    <rect x="1" y="1" width="{width-2}" height="{height-2}"
          fill="none" stroke="gray" stroke-width="1"/>

    <!-- Circles -->
'''

    # Draw circles
    for i in range(rows):
        for j in range(cols):
            # Asymmetric grid formula
            x = (2 * j + i % 2) * spacing_mm
            y = i * spacing_mm

            # Add margin
            x += margin_mm + circle_radius_mm
            y += margin_mm + circle_radius_mm

            svg += f'    <circle cx="{x}" cy="{y}" r="{circle_radius_mm}" fill="black"/>\n'

    svg += '</svg>'

    with open(output_file, 'w') as f:
        f.write(svg)

    print(f"✓ Generated SVG pattern: {output_file}")
    print(f"  Size: {width:.1f} × {height:.1f} mm")
    print(f"\nTo convert to PNG:")
    print(f"  1. Open in Inkscape or web browser")
    print(f"  2. Export as PNG at 300 DPI")
    print(f"  3. Or use: convert -density 300 {output_file} {output_file.replace('.svg', '.png')}")

# Generate patterns
generate_svg_pattern(
    rows=5, cols=6, spacing_mm=18.0, circle_diameter_mm=14.0,
    output_file='calibration_pattern_18mm.svg'
)

generate_svg_pattern(
    rows=5, cols=6, spacing_mm=16.0, circle_diameter_mm=14.0,
    output_file='calibration_pattern_16mm.svg'
)
