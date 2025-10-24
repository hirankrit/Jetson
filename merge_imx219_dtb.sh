#!/bin/bash
# Script to merge IMX219 Stereo Camera DTB overlay with base DTB
# This creates a single DTB file with camera support enabled

set -e

echo "=========================================="
echo "Merge IMX219 Stereo Camera DTB Overlay"
echo "=========================================="
echo ""

# Paths
BASE_DTB="/boot/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb"
OVERLAY="/boot/tegra234-p3767-camera-p3768-imx219-dual.dtbo"
OUTPUT_DTB="/boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super-imx219.dtb"
BACKUP_EXTLINUX="/boot/extlinux/extlinux.conf.backup-$(date +%Y%m%d-%H%M%S)"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ This script must be run as root (use sudo)"
    exit 1
fi

# Check files exist
echo "Checking files..."
if [ ! -f "$BASE_DTB" ]; then
    echo "❌ Base DTB not found: $BASE_DTB"
    exit 1
fi

if [ ! -f "$OVERLAY" ]; then
    echo "❌ Overlay not found: $OVERLAY"
    exit 1
fi

echo "✅ Base DTB: $BASE_DTB ($(stat -c%s $BASE_DTB) bytes)"
echo "✅ Overlay: $OVERLAY ($(stat -c%s $OVERLAY) bytes)"
echo ""

# Create /boot/dtb directory if it doesn't exist
mkdir -p /boot/dtb

# Backup existing merged DTB if it exists
if [ -f "$OUTPUT_DTB" ]; then
    echo "Backing up existing merged DTB..."
    cp "$OUTPUT_DTB" "${OUTPUT_DTB}.backup-$(date +%Y%m%d-%H%M%S)"
    echo "✅ Backup created"
fi

# Merge using fdtoverlay
echo ""
echo "Merging DTB overlay..."
fdtoverlay -i "$BASE_DTB" -o "$OUTPUT_DTB" "$OVERLAY"

if [ $? -eq 0 ]; then
    echo "✅ Merge successful!"
    echo "   Output: $OUTPUT_DTB ($(stat -c%s $OUTPUT_DTB) bytes)"
else
    echo "❌ Merge failed!"
    exit 1
fi

# Update extlinux.conf
echo ""
echo "Updating boot configuration..."

# Backup extlinux.conf
cp /boot/extlinux/extlinux.conf "$BACKUP_EXTLINUX"
echo "✅ Backup: $BACKUP_EXTLINUX"

# Update FDT line in primary label
sed -i '/LABEL primary/,/APPEND/ {
    s|FDT .*|FDT '"$OUTPUT_DTB"'|
}' /boot/extlinux/extlinux.conf

echo "✅ Updated extlinux.conf"
echo ""

# Show the updated configuration
echo "New boot configuration:"
echo "----------------------------------------"
grep -A 5 "LABEL primary" /boot/extlinux/extlinux.conf | head -6
echo "----------------------------------------"
echo ""

echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Review the configuration above"
echo "2. Reboot the system: sudo reboot"
echo "3. After reboot, test camera:"
echo "   gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! fakesink"
echo ""
