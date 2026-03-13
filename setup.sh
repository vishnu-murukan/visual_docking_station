#!/bin/bash
# Visual Docking Station - Setup Script
# For ROS2 Jazzy and Gazebo Harmonic

set -e

echo "=========================================="
echo "Visual Docking Station - Setup"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Ubuntu 24.04
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$VERSION_ID" != "24.04" ]; then
        echo -e "${YELLOW}Warning: This script is designed for Ubuntu 24.04${NC}"
        echo "Your version: $VERSION_ID"
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Check if ROS2 Jazzy is installed
if [ ! -d "/opt/ros/jazzy" ]; then
    echo -e "${RED}ROS2 Jazzy not found!${NC}"
    echo "Please install ROS2 Jazzy first:"
    echo "https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 Jazzy detected${NC}"

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install system dependencies
echo ""
echo "Installing system dependencies..."
sudo apt update

PACKAGES=(
    "gz-harmonic"
    "ros-jazzy-ros-gz"
    "ros-jazzy-cv-bridge"
    "ros-jazzy-image-transport"
    "ros-jazzy-robot-state-publisher"
    "ros-jazzy-xacro"
    "ros-jazzy-tf2-ros"
    "ros-jazzy-tf2-geometry-msgs"
    "ros-jazzy-rqt-image-view"
    "python3-pip"
)

for package in "${PACKAGES[@]}"; do
    if dpkg -l | grep -q "^ii  $package "; then
        echo -e "${GREEN}✓ $package already installed${NC}"
    else
        echo "Installing $package..."
        sudo apt install -y "$package"
    fi
done

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip install --break-system-packages \
    opencv-python \
    opencv-contrib-python \
    scipy \
    numpy \
    pillow

echo -e "${GREEN}✓ Python dependencies installed${NC}"

# Build the workspace
echo ""
echo "Building ROS2 workspace..."
cd "$(dirname "$0")"

if [ -d "build" ] || [ -d "install" ] || [ -d "log" ]; then
    echo "Cleaning previous build..."
    rm -rf build install log
fi

colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful!${NC}"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi

# Create convenience scripts
echo ""
echo "Creating convenience scripts..."

# Source script
cat > source_workspace.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source $(dirname "$0")/install/setup.bash
echo "Visual Docking workspace sourced!"
EOF
chmod +x source_workspace.sh

# Run simulation script
cat > run_simulation.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source $(dirname "$0")/install/setup.bash
ros2 launch visual_docking docking_simulation.launch.py
EOF
chmod +x run_simulation.sh

echo -e "${GREEN}✓ Convenience scripts created${NC}"

# Setup complete
echo ""
echo "=========================================="
echo -e "${GREEN}Setup Complete!${NC}"
echo "=========================================="
echo ""
echo "To run the simulation:"
echo "  ./run_simulation.sh"
echo ""
echo "Or manually:"
echo "  source ./source_workspace.sh"
echo "  ros2 launch visual_docking docking_simulation.launch.py"
echo ""
echo "To view camera feed:"
echo "  ros2 run rqt_image_view rqt_image_view /docking/debug_image"
echo ""
echo "=========================================="
