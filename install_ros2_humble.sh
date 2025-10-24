#!/bin/bash
# Install ROS2 Humble on Jetson Orin Nano (Ubuntu 22.04 Jammy)

set -e

echo "========================================"
echo "ROS2 Humble Installation for Jetson"
echo "========================================"
echo ""

# Check Ubuntu version
echo "Checking Ubuntu version..."
source /etc/os-release
if [ "$VERSION_CODENAME" != "jammy" ]; then
    echo "❌ Error: This script is for Ubuntu 22.04 (Jammy) only"
    echo "   Your version: $VERSION_CODENAME"
    exit 1
fi
echo "✅ Ubuntu 22.04 (Jammy) detected"
echo ""

# Set locale
echo "Step 1/6: Setting up locale..."
sudo apt update -qq
sudo apt install -y locales > /dev/null 2>&1
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "✅ Locale configured"
echo ""

# Setup sources
echo "Step 2/6: Adding ROS2 apt repository..."
sudo apt install -y software-properties-common curl > /dev/null 2>&1
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update -qq
echo "✅ ROS2 repository added"
echo ""

# Install ROS2 Humble
echo "Step 3/6: Installing ROS2 Humble Desktop (this may take 10-15 minutes)..."
echo "   This includes: ROS2 core, RViz, demos, tutorials"
sudo apt install -y ros-humble-desktop > /dev/null 2>&1
echo "✅ ROS2 Humble Desktop installed"
echo ""

# Install development tools
echo "Step 4/6: Installing ROS2 development tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    > /dev/null 2>&1
echo "✅ Development tools installed"
echo ""

# Install ROS2 packages for camera
echo "Step 5/6: Installing camera-related ROS2 packages..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-calibration \
    ros-humble-camera-info-manager \
    ros-humble-image-pipeline \
    ros-humble-vision-opencv \
    python3-opencv \
    > /dev/null 2>&1
echo "✅ Camera packages installed"
echo ""

# Initialize rosdep
echo "Step 6/6: Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
echo "✅ rosdep initialized"
echo ""

# Setup bashrc
echo "Adding ROS2 setup to ~/.bashrc..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble setup" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "✅ Added to ~/.bashrc"
else
    echo "✅ Already in ~/.bashrc"
fi
echo ""

echo "========================================"
echo "Installation Complete! ✅"
echo "========================================"
echo ""
echo "To activate ROS2 in current shell, run:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "Or open a new terminal (it will auto-source)"
echo ""
echo "Test ROS2 installation:"
echo "  ros2 --version"
echo "  ros2 topic list"
echo ""
