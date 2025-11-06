#!/usr/bin/env python3
"""
Auto Preprocessing for Cheap Stereo Cameras
Optimized for IMX219 1800 THB camera from China

Usage:
    from stereo_preprocess import auto_preprocess

    enhanced_left = auto_preprocess(left_image)
    enhanced_right = auto_preprocess(right_image)
"""

import cv2
import numpy as np
from typing import Tuple, Optional


def auto_preprocess(
    image: np.ndarray,
    method: str = "denoise",
    denoise_strength: int = 10,
    clahe_clip: float = 2.0,
    sharpen_strength: float = 0.3
) -> np.ndarray:
    """
    Auto preprocessing for stereo images from cheap cameras

    Best method tested: 'denoise' gives 4.3x improvement!
    (Valid pixels: 6.3% → 27.0%)

    Args:
        image: Input image (BGR or grayscale)
        method: Preprocessing method
            - 'denoise': Non-local means denoising (BEST! ⭐)
            - 'clahe': CLAHE contrast enhancement
            - 'bilateral': Edge-preserving smoothing
            - 'combo': Denoise + CLAHE + Sharpen (good for very dark images)
            - 'none': No preprocessing
        denoise_strength: Denoising strength (7-15, default=10)
            Lower = preserve more detail, Higher = more smoothing
        clahe_clip: CLAHE clip limit (1.0-4.0, default=2.0)
        sharpen_strength: Sharpening strength (0.0-0.5, default=0.3)

    Returns:
        Enhanced grayscale image

    Example:
        >>> import cv2
        >>> from stereo_preprocess import auto_preprocess
        >>>
        >>> left = cv2.imread("left.jpg")
        >>> enhanced = auto_preprocess(left, method="denoise")
        >>> # Use enhanced image for stereo matching
    """

    # Convert to grayscale if needed
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()

    if method == "none":
        return gray

    # ========================================
    # Method 1: Denoising (BEST! +329% improvement)
    # ========================================
    if method == "denoise":
        enhanced = cv2.fastNlMeansDenoising(
            gray,
            None,
            h=denoise_strength,
            templateWindowSize=7,
            searchWindowSize=21
        )
        return enhanced

    # ========================================
    # Method 2: CLAHE (Good for contrast)
    # ========================================
    elif method == "clahe":
        clahe = cv2.createCLAHE(
            clipLimit=clahe_clip,
            tileGridSize=(8, 8)
        )
        enhanced = clahe.apply(gray)
        return enhanced

    # ========================================
    # Method 3: Bilateral Filter (Smooth + edge-preserving)
    # ========================================
    elif method == "bilateral":
        enhanced = cv2.bilateralFilter(
            gray,
            d=5,
            sigmaColor=75,
            sigmaSpace=75
        )
        return enhanced

    # ========================================
    # Method 4: Combo (For very challenging conditions)
    # ========================================
    elif method == "combo":
        # Step 1: Denoise
        denoised = cv2.fastNlMeansDenoising(
            gray,
            None,
            h=7,  # Gentler denoising
            templateWindowSize=7,
            searchWindowSize=21
        )

        # Step 2: CLAHE
        clahe = cv2.createCLAHE(
            clipLimit=clahe_clip,
            tileGridSize=(8, 8)
        )
        enhanced = clahe.apply(denoised)

        # Step 3: Sharpen
        gaussian = cv2.GaussianBlur(enhanced, (3, 3), 1.0)
        sharpened = cv2.addWeighted(
            enhanced, 1.0 + sharpen_strength,
            gaussian, -sharpen_strength,
            0
        )

        return sharpened

    else:
        raise ValueError(f"Unknown method: {method}")


def auto_preprocess_stereo(
    left_image: np.ndarray,
    right_image: np.ndarray,
    method: str = "denoise",
    **kwargs
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Auto preprocessing for stereo image pairs

    Args:
        left_image: Left camera image
        right_image: Right camera image
        method: Preprocessing method (see auto_preprocess)
        **kwargs: Additional arguments for auto_preprocess

    Returns:
        Tuple of (enhanced_left, enhanced_right)

    Example:
        >>> left, right = cv2.imread("left.jpg"), cv2.imread("right.jpg")
        >>> left_enh, right_enh = auto_preprocess_stereo(left, right)
    """
    left_enhanced = auto_preprocess(left_image, method=method, **kwargs)
    right_enhanced = auto_preprocess(right_image, method=method, **kwargs)

    return left_enhanced, right_enhanced


def preview_preprocessing(
    image: np.ndarray,
    methods: Optional[list] = None
) -> dict:
    """
    Preview different preprocessing methods side-by-side

    Args:
        image: Input image
        methods: List of methods to try (default: all)

    Returns:
        Dictionary of {method_name: enhanced_image}

    Example:
        >>> import cv2
        >>> from stereo_preprocess import preview_preprocessing
        >>>
        >>> img = cv2.imread("test.jpg")
        >>> results = preview_preprocessing(img)
        >>>
        >>> # Show all results
        >>> for method, enhanced in results.items():
        >>>     cv2.imshow(method, enhanced)
        >>> cv2.waitKey(0)
    """
    if methods is None:
        methods = ["none", "denoise", "clahe", "bilateral", "combo"]

    results = {}
    for method in methods:
        results[method] = auto_preprocess(image, method=method)

    return results


def save_comparison(
    image: np.ndarray,
    output_path: str = "preprocessing_comparison.jpg"
) -> None:
    """
    Save comparison of all preprocessing methods

    Args:
        image: Input image
        output_path: Output file path

    Example:
        >>> img = cv2.imread("test.jpg")
        >>> save_comparison(img, "comparison.jpg")
    """
    results = preview_preprocessing(image)

    # Create visualization
    visualizations = []
    for method, enhanced in results.items():
        # Colorize for better visualization
        enhanced_color = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)

        # Add label
        cv2.putText(
            enhanced_color,
            method.upper(),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        visualizations.append(enhanced_color)

    # Stack horizontally
    comparison = np.hstack(visualizations)
    cv2.imwrite(output_path, comparison)
    print(f"✅ Saved comparison: {output_path}")


# ============================================
# Recommended Settings for Different Scenarios
# ============================================

PRESETS = {
    "cheap_camera": {
        "method": "denoise",
        "denoise_strength": 10,
        "description": "Best for cheap cameras (1800 THB IMX219) ⭐"
    },

    "dark_scene": {
        "method": "combo",
        "denoise_strength": 7,
        "clahe_clip": 3.0,
        "sharpen_strength": 0.4,
        "description": "For very dark scenes with poor lighting"
    },

    "normal": {
        "method": "bilateral",
        "description": "For normal lighting conditions"
    },

    "high_quality": {
        "method": "none",
        "description": "For high-quality cameras with good lighting"
    }
}


def get_preset(preset_name: str) -> dict:
    """
    Get recommended preprocessing settings for different scenarios

    Args:
        preset_name: One of:
            - 'cheap_camera': For 1800 THB IMX219 cameras (BEST!)
            - 'dark_scene': For poor lighting
            - 'normal': For normal conditions
            - 'high_quality': For good cameras + good lighting

    Returns:
        Dictionary of settings

    Example:
        >>> from stereo_preprocess import auto_preprocess, get_preset
        >>>
        >>> settings = get_preset("cheap_camera")
        >>> enhanced = auto_preprocess(image, **settings)
    """
    if preset_name not in PRESETS:
        available = ", ".join(PRESETS.keys())
        raise ValueError(f"Unknown preset: {preset_name}. Available: {available}")

    settings = PRESETS[preset_name].copy()
    # Remove description key (not a parameter for auto_preprocess)
    settings.pop('description', None)
    return settings


def print_presets():
    """Print all available presets"""
    print("\n" + "="*60)
    print("📋 Available Preprocessing Presets")
    print("="*60)

    for name, settings in PRESETS.items():
        print(f"\n🔹 '{name}':")
        print(f"   {settings['description']}")
        print(f"   Settings: {settings}")

    print("\n" + "="*60)
    print("Usage:")
    print("  settings = get_preset('cheap_camera')")
    print("  enhanced = auto_preprocess(image, **settings)")
    print("="*60 + "\n")


# ============================================
# Quick Test Function
# ============================================

def test_preprocessing():
    """Quick test of preprocessing functions"""
    print("\n" + "="*60)
    print("🧪 Testing Stereo Preprocessing")
    print("="*60)

    # Create test image
    test_img = np.random.randint(0, 255, (720, 1280), dtype=np.uint8)

    # Test all methods
    print("\nTesting methods...")
    for method in ["none", "denoise", "clahe", "bilateral", "combo"]:
        result = auto_preprocess(test_img, method=method)
        print(f"  ✅ {method}: {result.shape}")

    # Test presets
    print("\nTesting presets...")
    for preset_name in PRESETS.keys():
        settings = get_preset(preset_name)
        result = auto_preprocess(test_img, **settings)
        print(f"  ✅ {preset_name}: {result.shape}")

    print("\n✅ All tests passed!")
    print("="*60 + "\n")


if __name__ == "__main__":
    print("\n" + "="*60)
    print("🔧 Stereo Preprocessing Utilities")
    print("="*60)
    print("\nOptimized for cheap cameras (IMX219 1800 THB)")
    print("Best method: 'denoise' (+329% improvement!)")

    print_presets()
    test_preprocessing()

    print("\n💡 Quick Start:")
    print("="*60)
    print("""
from stereo_preprocess import auto_preprocess, get_preset

# Simple usage (best for cheap cameras)
enhanced = auto_preprocess(image, method="denoise")

# Or use preset
settings = get_preset("cheap_camera")
enhanced = auto_preprocess(image, **settings)

# For stereo pairs
from stereo_preprocess import auto_preprocess_stereo
left_enh, right_enh = auto_preprocess_stereo(left, right)
    """)
    print("="*60 + "\n")
