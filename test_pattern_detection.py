#!/usr/bin/env python3
"""
ทดสอบ Pattern Detection จากรูปที่ capture มาแล้ว
เพื่อตรวจสอบว่า capture_calibration.py detect pattern ถูกต้องหรือไม่
"""

import cv2
import numpy as np
import glob

def test_pattern_detection():
    """ทดสอบ pattern detection กับรูปจริง"""

    print("=" * 70)
    print("ทดสอบ Pattern Detection จาก Calibration Images")
    print("=" * 70)

    # Pattern parameters (ต้องตรงกับทั้ง 2 scripts)
    PATTERN_ROWS = 5
    PATTERN_COLS = 6
    PATTERN_TYPE = cv2.CALIB_CB_ASYMMETRIC_GRID

    # SimpleBlobDetector parameters (copy จาก stereo_calibration.py)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 0
    params.filterByArea = True
    params.minArea = 50
    params.maxArea = 5000
    params.filterByCircularity = True
    params.minCircularity = 0.8
    params.filterByConvexity = True
    params.minConvexity = 0.87
    params.filterByInertia = True
    params.minInertiaRatio = 0.6

    detector = cv2.SimpleBlobDetector_create(params)

    # Load images
    images_left = sorted(glob.glob('calib_images/left/*.jpg'))
    images_right = sorted(glob.glob('calib_images/right/*.jpg'))

    print(f"\nพบรูป Left: {len(images_left)} ภาพ")
    print(f"พบรูป Right: {len(images_right)} ภาพ")

    if len(images_left) != len(images_right):
        print("\n❌ ERROR: จำนวนรูป Left และ Right ไม่เท่ากัน!")
        return

    print(f"\n{'='*70}")
    print("ทดสอบ Pattern Detection แต่ละคู่:")
    print(f"{'='*70}")
    print(f"{'Pair':<6} {'Left Status':<20} {'Right Status':<20} {'Both OK':<10}")
    print(f"{'-'*70}")

    successful_pairs = 0
    failed_pairs = []

    for idx, (img_left_path, img_right_path) in enumerate(zip(images_left, images_right)):
        # Read images
        img_left = cv2.imread(img_left_path)
        img_right = cv2.imread(img_right_path)

        # Convert to grayscale
        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        # Find circles
        ret_left, corners_left = cv2.findCirclesGrid(
            gray_left,
            (PATTERN_COLS, PATTERN_ROWS),
            flags=PATTERN_TYPE,
            blobDetector=detector
        )

        ret_right, corners_right = cv2.findCirclesGrid(
            gray_right,
            (PATTERN_COLS, PATTERN_ROWS),
            flags=PATTERN_TYPE,
            blobDetector=detector
        )

        # Status
        left_status = "✓ DETECTED (30 pts)" if ret_left else "✗ NOT DETECTED"
        right_status = "✓ DETECTED (30 pts)" if ret_right else "✗ NOT DETECTED"
        both_ok = "✓ YES" if (ret_left and ret_right) else "✗ NO"

        # Count corners if detected
        if ret_left:
            left_status = f"✓ DETECTED ({len(corners_left)} pts)"
        if ret_right:
            right_status = f"✓ DETECTED ({len(corners_right)} pts)"

        print(f"{idx:>3}    {left_status:<20} {right_status:<20} {both_ok:<10}")

        if ret_left and ret_right:
            successful_pairs += 1
        else:
            failed_pairs.append(idx)

    print(f"{'-'*70}")
    print(f"\n{'='*70}")
    print("สรุปผลการทดสอบ:")
    print(f"{'='*70}")
    print(f"✓ Successful pairs: {successful_pairs}/{len(images_left)}")
    print(f"✗ Failed pairs: {len(failed_pairs)}/{len(images_left)}")

    if failed_pairs:
        print(f"\nคู่ที่ล้มเหลว: {failed_pairs}")

    # Assessment
    print(f"\n{'='*70}")
    print("การประเมิน:")
    print(f"{'='*70}")

    if successful_pairs >= 10:
        print("✓ จำนวนคู่ที่ใช้ได้ เพียงพอสำหรับ calibration (≥10)")
    else:
        print(f"✗ จำนวนคู่ไม่พอ! ต้องการอย่างน้อย 10 คู่ (มีแค่ {successful_pairs})")

    success_rate = (successful_pairs / len(images_left)) * 100
    print(f"\nอัตราความสำเร็จ: {success_rate:.1f}%")

    if success_rate >= 90:
        print("  → ยอดเยี่ยม! (≥90%)")
    elif success_rate >= 70:
        print("  → ดี (70-90%)")
    elif success_rate >= 50:
        print("  → พอใช้ (50-70%)")
    else:
        print("  → ควรถ่ายรูปเพิ่ม (<50%)")

    print(f"\n{'='*70}")

    # Test with one sample image - visualize detection
    if len(images_left) > 0:
        print("\nทดสอบแสดงผล Detection บนรูปแรก...")
        img_left = cv2.imread(images_left[0])
        img_right = cv2.imread(images_right[0])

        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        ret_left, corners_left = cv2.findCirclesGrid(
            gray_left,
            (PATTERN_COLS, PATTERN_ROWS),
            flags=PATTERN_TYPE,
            blobDetector=detector
        )

        ret_right, corners_right = cv2.findCirclesGrid(
            gray_right,
            (PATTERN_COLS, PATTERN_ROWS),
            flags=PATTERN_TYPE,
            blobDetector=detector
        )

        if ret_left:
            cv2.drawChessboardCorners(img_left, (PATTERN_COLS, PATTERN_ROWS),
                                     corners_left, ret_left)
        if ret_right:
            cv2.drawChessboardCorners(img_right, (PATTERN_COLS, PATTERN_ROWS),
                                     corners_right, ret_right)

        cv2.imwrite('test_detection_left.jpg', img_left)
        cv2.imwrite('test_detection_right.jpg', img_right)

        print("✓ บันทึกภาพทดสอบแล้ว:")
        print("  - test_detection_left.jpg")
        print("  - test_detection_right.jpg")

    print(f"{'='*70}\n")

    return successful_pairs, len(images_left)


if __name__ == '__main__':
    successful, total = test_pattern_detection()

    if successful >= 10:
        print("✓ พร้อมสำหรับ stereo calibration!")
    else:
        print("✗ ต้องถ่ายรูปเพิ่มอีก")
