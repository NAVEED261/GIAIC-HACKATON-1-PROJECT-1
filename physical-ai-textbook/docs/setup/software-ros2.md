---
title: "Software: ROS 2 Humble Installation"
description: "Step-by-step guide to installing ROS 2 Humble Hawksbill on Ubuntu 22.04 for Physical AI development"
sidebar_label: "ROS 2 Humble"
sidebar_position: 5
estimated_time: 2
week: 1
module: 0
prerequisites: ["setup/hardware-digital-twin", "setup/hardware-edge-kit", "setup/hardware-cloud"]
learning_objectives:
  - "Install ROS 2 Humble from Debian packages"
  - "Configure ROS 2 environment and workspace"
  - "Verify installation with basic commands and sample nodes"
  - "Understand ROS 2 directory structure and environment variables"
---

# Software: ROS 2 Humble Installation

## Overview

**ROS 2 Humble Hawksbill** is the Long-Term Support (LTS) release of ROS 2, supported until May 2027. This guide covers installation on Ubuntu 22.04 for all three hardware options (Workstation, Jetson, Cloud).

**Installation Time:** 30-45 minutes (including download and verification)

## Prerequisites

Before installing ROS 2, ensure you have:

- [ ] Ubuntu 22.04 LTS installed and updated
- [ ] Stable internet connection (will download ~1.5GB)
- [ ] At least 10GB free disk space
- [ ] Sudo privileges

## Installation Steps

### Step 1: Set System Locale

ROS 2 requires UTF-8 locale support:

```bash
# Check current locale
locale

# If not already set, configure UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale
```

### Step 2: Add ROS 2 Repository

```bash
# Install software-properties-common
sudo apt install software-properties-common

# Add universe repository
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Humble Desktop

**Desktop vs Base:**
- **Desktop:** Includes GUI tools (RViz, rqt, Gazebo integration) - **Recommended for this course**
- **Base:** Command-line only, lighter weight

```bash
# Update package index
sudo apt update

# Upgrade existing packages
sudo apt upgrade -y

# Install ROS 2 Humble Desktop (full installation)
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y

# Installation size: ~1.8GB
# Time: 15-20 minutes
```

**What Gets Installed:**
- ROS 2 core libraries
- RViz (3D visualization tool)
- rqt (Qt-based GUI framework)
- `colcon` build system
- Python 3 client library (`rclpy`)
- C++ client library (`rclcpp`)
- Common message types (std_msgs, geometry_msgs, sensor_msgs)

### Step 4: Environment Setup

Add ROS 2 to your shell configuration so it's sourced automatically:

```bash
# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Source for current terminal session
source ~/.bashrc

# Verify ROS_DISTRO environment variable
echo $ROS_DISTRO
# Expected output: humble
```

**Important Environment Variables:**

| Variable | Purpose | Default Value |
|----------|---------|---------------|
| `ROS_DISTRO` | ROS 2 distribution name | `humble` |
| `ROS_VERSION` | ROS major version | `2` |
| `ROS_PYTHON_VERSION` | Python version | `3` |
| `ROS_LOCALHOST_ONLY` | Restrict to localhost (security) | `0` (disabled) |
| `ROS_DOMAIN_ID` | DDS domain (multi-robot isolation) | `0` |

### Step 5: Install Additional ROS 2 Packages

Install commonly used packages for robotics development:

```bash
# Gazebo simulation integration
sudo apt install ros-humble-gazebo-ros-pkgs -y

# ROS 2 control (for robot manipulation)
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers -y

# Navigation2 (for autonomous navigation)
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y

# TF2 transforms (spatial relationships)
sudo apt install ros-humble-tf2-tools ros-humble-tf-transformations -y

# Image processing
sudo apt install ros-humble-image-transport ros-humble-image-pipeline -y

# Python dependencies
pip3 install transforms3d pyquaternion
```

## Verification

### Test 1: Check ROS 2 Version

```bash
ros2 --version
```

**Expected Output:**
```
ros2 cli version 0.25.3
```

### Test 2: Run Talker-Listener Demo

**Terminal 1 (Talker):**
```bash
ros2 run demo_nodes_cpp talker
```

**Expected Output:**
```
[INFO] [1733750420.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1733750421.123456789] [talker]: Publishing: 'Hello World: 2'
...
```

**Terminal 2 (Listener):**
```bash
ros2 run demo_nodes_py listener
```

**Expected Output:**
```
[INFO] [1733750420.234567890] [listener]: I heard: [Hello World: 1]
[INFO] [1733750421.234567890] [listener]: I heard: [Hello World: 2]
...
```

✅ **Success:** If listener receives messages from talker, ROS 2 is working correctly!

### Test 3: Check Available Nodes

```bash
# List running nodes
ros2 node list

# Expected output:
# /talker
# /listener

# Get info about a node
ros2 node info /talker
```

### Test 4: Visualize with RQT

```bash
# Launch rqt (GUI tool for ROS 2)
rqt
```

- Click **Plugins → Introspection → Node Graph**
- You should see `/talker` and `/listener` nodes connected by `/chatter` topic

### Test 5: Check TF2 (Transforms)

```bash
# View TF tree
ros2 run tf2_tools view_frames

# This generates a PDF showing transform tree
# Open with: evince frames.pdf
```

## Create Your First Workspace

ROS 2 uses **workspaces** to organize packages. Let's create a workspace for this course:

```bash
# Create workspace directory
mkdir -p ~/physical_ai_ws/src
cd ~/physical_ai_ws/src

# Clone a sample package
git clone https://github.com/ros/ros_tutorials.git -b humble

# Build workspace
cd ~/physical_ai_ws
colcon build --symlink-install

# Source workspace overlay
source install/setup.bash

# Verify workspace
echo $COLCON_PREFIX_PATH
# Should include ~/physical_ai_ws/install
```

**Directory Structure:**
```
~/physical_ai_ws/
├── build/        # Build artifacts (CMake cache, object files)
├── install/      # Installed packages (binaries, libraries)
├── log/          # Build logs
└── src/          # Source code for ROS 2 packages
    └── ros_tutorials/
```

### Add Workspace to .bashrc

```bash
# Automatically source workspace on terminal startup
echo "source ~/physical_ai_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Common Issues and Solutions

### Issue 1: "Unable to locate package ros-humble-desktop"

**Cause:** ROS 2 repository not added correctly.

**Solution:**
```bash
# Remove and re-add repository
sudo rm /etc/apt/sources.list.d/ros2.list
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### Issue 2: "colcon: command not found"

**Cause:** `ros-dev-tools` not installed.

**Solution:**
```bash
sudo apt install python3-colcon-common-extensions -y
```

### Issue 3: Talker/Listener Not Communicating

**Cause:** Firewall blocking DDS communication or wrong ROS_DOMAIN_ID.

**Solution:**
```bash
# Disable firewall temporarily (testing only)
sudo ufw disable

# OR allow DDS multicast
sudo ufw allow from 224.0.0.0/4
sudo ufw allow 7400:7500/udp

# Check if nodes are on same domain
echo $ROS_DOMAIN_ID
# Should be same value in both terminals (default: 0)
```

### Issue 4: "ImportError: No module named 'rclpy'"

**Cause:** ROS 2 environment not sourced.

**Solution:**
```bash
source /opt/ros/humble/setup.bash
# OR
source ~/.bashrc  # if already added to .bashrc
```

## Jetson-Specific Notes

**Pre-Installed ROS 2:** Some JetPack images come with ROS 2 pre-installed. Check first:

```bash
# Check if ROS 2 already installed
dpkg -l | grep ros-humble

# If already installed, skip to verification step
# If not, follow standard installation steps above
```

**ARM Architecture:** All ROS 2 Humble packages support ARM64 (no special steps needed).

## Cloud-Specific Notes

**Headless Operation:** Cloud instances don't need GUI tools. Use `ros-humble-ros-base` instead:

```bash
# Lighter installation without GUI tools
sudo apt install ros-humble-ros-base -y

# This saves ~500MB disk space and installation time
```

**Remote Visualization:** To use RViz/rqt on cloud instance, set up X11 forwarding:

```bash
# On local machine, connect with X11 forwarding
ssh -X ubuntu@<CLOUD_IP>

# Launch RViz
rviz2
# GUI will display on your local machine
```

## Performance Optimization

### 1. Enable Shared Memory Transport (FastDDS)

For same-machine communication, shared memory is faster than UDP:

```bash
# Create FastDDS XML configuration
cat > ~/fastdds_config.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>SHM</transport_id>
            <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
EOF

# Export configuration path
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_config.xml" >> ~/.bashrc
source ~/.bashrc
```

### 2. Increase DDS Buffer Size (for Large Messages)

```bash
# For high-resolution images or point clouds
echo "export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS" >> ~/.bashrc
source ~/.bashrc
```

## Next Steps

✅ ROS 2 Humble installed and verified!

**Continue to:**
1. **[Module 1: ROS 2 Fundamentals](/docs/module-1-ros2)** - Learn nodes, topics, services, actions
2. **[Software: Isaac Sim Setup](/docs/setup/software-isaac-sim)** - For Module 3 (Weeks 8-10)
3. **Gazebo Installation** (coming soon) - For Module 2 (Digital Twin Simulation)

## Additional Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Discourse Forum](https://discourse.ros.org/)
- [ROS 2 GitHub](https://github.com/ros2)

---

**Installation Complete!** Ready to start Module 1.
