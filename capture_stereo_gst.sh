#!/bin/bash
# Capture stereo pair images from IMX219 using GStreamer
# Usage: ./capture_stereo_gst.sh [output_prefix]

OUTPUT_PREFIX="${1:-stereo_capture}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LEFT_FILE="${OUTPUT_PREFIX}_left_${TIMESTAMP}.jpg"
RIGHT_FILE="${OUTPUT_PREFIX}_right_${TIMESTAMP}.jpg"

echo "============================================"
echo "IMX219 Stereo Camera Capture (GStreamer)"
echo "============================================"
echo ""

# Capture from left camera (sensor-id=0)
echo "📷 Capturing LEFT camera (sensor-id=0)..."
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=1 \
    ! "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=15/1" \
    ! nvvidconv ! "video/x-raw, format=BGRx" \
    ! videoconvert ! "video/x-raw, format=BGR" \
    ! jpegenc ! filesink location="${LEFT_FILE}" \
    2>&1 | grep -v "GLib\|gst_nv" | tail -3

if [ -f "${LEFT_FILE}" ]; then
    echo "✅ LEFT image saved: ${LEFT_FILE}"
else
    echo "❌ Failed to capture LEFT image"
    exit 1
fi

# Small delay to avoid camera conflict
sleep 1

# Capture from right camera (sensor-id=1)
echo ""
echo "📷 Capturing RIGHT camera (sensor-id=1)..."
gst-launch-1.0 -e nvarguscamerasrc sensor-id=1 num-buffers=1 \
    ! "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=15/1" \
    ! nvvidconv ! "video/x-raw, format=BGRx" \
    ! videoconvert ! "video/x-raw, format=BGR" \
    ! jpegenc ! filesink location="${RIGHT_FILE}" \
    2>&1 | grep -v "GLib\|gst_nv" | tail -3

if [ -f "${RIGHT_FILE}" ]; then
    echo "✅ RIGHT image saved: ${RIGHT_FILE}"
else
    echo "❌ Failed to capture RIGHT image"
    exit 1
fi

echo ""
echo "============================================"
echo "✅ Stereo pair captured successfully!"
echo "============================================"
echo "  LEFT:  ${LEFT_FILE}"
echo "  RIGHT: ${RIGHT_FILE}"
echo ""
echo "Next steps:"
echo "  1. Test with ESS: ./run_ess_test.sh"
echo "  2. View images: eog ${LEFT_FILE} ${RIGHT_FILE}"
echo "  3. Test with YOLO11n detection"
echo ""
