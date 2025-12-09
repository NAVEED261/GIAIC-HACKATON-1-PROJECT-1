---
title: "Software: NVIDIA Isaac Sim Installation"
description: "Install NVIDIA Isaac Sim for high-fidelity robot simulation and computer vision"
sidebar_label: "Isaac Sim"
sidebar_position: 6
estimated_time: 3
week: 1
module: 0
prerequisites: ["setup/software-ros2"]
learning_objectives:
  - "Install NVIDIA Omniverse and Isaac Sim 2023.1"
  - "Configure Isaac Sim with ROS 2 Humble bridge"
  - "Verify GPU acceleration and rendering performance"
  - "Run first Isaac Sim simulation with robot model"
---

# Software: NVIDIA Isaac Sim Installation

## Overview

**NVIDIA Isaac Sim** is a high-fidelity robotics simulation platform built on Om

niverse, providing GPU-accelerated physics, ray-traced rendering, and synthetic data generation for training perception models.

**Requirements:**
- **GPU:** NVIDIA RTX 3060 or better (12GB+ VRAM)
- **Storage:** 50GB free space
- **OS:** Ubuntu 22.04 or Windows 11

**⚠️ Jetson Compatibility:** Isaac Sim is **NOT supported** on Jetson Orin Nano due to insufficient VRAM. Use cloud or workstation for Module 3.

## Installation Steps (Ubuntu 22.04)

### Step 1: Install Omniverse Launcher

```bash
# Download Omniverse Launcher
cd ~/Downloads
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

### Step 2: Sign In and Install Isaac Sim

1. **Create NVIDIA Account** (if you don't have one)
2. Sign in to Omniverse Launcher
3. Click **Exchange** tab → Search for "Isaac Sim"
4. Select **Isaac Sim 2023.1.1** (recommended for ROS 2 Humble)
5. Click **Install** (downloads ~15GB, takes 30-45 minutes)

**Installation Path:** `~/.local/share/ov/pkg/isaac_sim-2023.1.1/`

### Step 3: Verify Installation

```bash
# Launch Isaac Sim
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

# Expected: Isaac Sim GUI opens with sample scene
# First launch takes 2-3 minutes (shader compilation)
```

### Step 4: Install ROS 2 Bridge

```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# Install ROS 2 dependencies
./python.sh -m pip install --upgrade pip
./python.sh -m pip install rospy-message-converter

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Verify bridge
./python.sh standalone_examples/api/omni.isaac.ros2_bridge/ros2_sample.py
```

## Cloud Installation (Docker Method)

For cloud instances without GUI, use Isaac Sim container:

```bash
# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run headless
docker run --name=isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    nvcr.io/nvidia/isaac-sim:2023.1.1
```

## Verification Tests

### Test 1: GPU Detection

```bash
# Inside Isaac Sim Python environment
~/.local/share/ov/pkg/isaac_sim-2023.1.1/python.sh

>>> from pxr import Usd, UsdGeom
>>> import carb
>>> carb.get_framework().get_renderer_info()
# Should show NVIDIA GPU model
```

### Test 2: Run Sample Scene

```bash
# Launch Carter robot navigation demo
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh standalone_examples/api/omni.isaac.wheeled_robots/carter_lidar.py
```

**Expected:** Carter robot appears in 3D viewport, lidar rays visible, FPS >20.

### Test 3: ROS 2 Bridge Test

**Terminal 1: Start Isaac Sim with ROS bridge**
```bash
source /opt/ros/humble/setup.bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh standalone_examples/api/omni.isaac.ros2_bridge/clock.py
```

**Terminal 2: Check ROS topics**
```bash
source /opt/ros/humble/setup.bash
ros2 topic list

# Expected output includes:
# /clock
# /rosout
```

✅ **Success:** If `/clock` topic is publishing, ROS 2 bridge is working!

## Performance Optimization

### 1. Enable RTX Mode (for RTX GPUs)

```python
# In Isaac Sim scripts
import omni.replicator.core as rep
rep.settings.carb_settings("/rtx/pathtracing/spp", 16)  # Samples per pixel
rep.settings.carb_settings("/rtx/pathtracing/maxBounces", 4)
```

### 2. Adjust Physics Timestep

```python
# For faster-than-realtime simulation
from omni.isaac.core.utils.physics import simulate_async
simulate_async(1/240.0)  # 240 Hz physics (vs default 60 Hz)
```

### 3. Reduce Shadow Quality (for FPS boost)

In Isaac Sim GUI:
- **Edit → Preferences → Rendering**
- Set "Shadow Map Size" to 1024 (from 2048)
- Disable "Ambient Occlusion" if not needed

## Troubleshooting

### Issue: "vulkan: error getting physical device properties"

**Cause:** NVIDIA driver or Vulkan not properly installed.

**Solution:**
```bash
# Install Vulkan support
sudo apt install mesa-vulkan-drivers vulkan-tools
vulkaninfo | grep "deviceName"

# Update NVIDIA driver if needed
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Issue: Isaac Sim crashes on launch

**Cause:** Insufficient VRAM or GPU not supported.

**Solution:**
```bash
# Check GPU VRAM
nvidia-smi --query-gpu=name,memory.total --format=csv

# Minimum: 12GB VRAM
# If less, reduce scene complexity or use cloud GPU
```

### Issue: ROS 2 topics not visible

**Cause:** ROS 2 environment not sourced before launching Isaac Sim.

**Solution:**
```bash
# ALWAYS source ROS 2 first
source /opt/ros/humble/setup.bash
~/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh
```

## Next Steps

✅ Isaac Sim installed!

**Module 3 (Weeks 8-10):**
- [Week 8: Isaac Sim Setup](/docs/module-3-isaac/week-8-setup)
- [Week 9: Perception Pipeline](/docs/module-3-isaac/week-9-perception)
- [Week 10: Advanced Perception](/docs/module-3-isaac/week-10-advanced)

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [ROS 2 Bridge Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/)
- [Isaac Sim Forums](https://forums.developer.nvidia.com/c/omniverse/simulation/69)

---

**Installation Complete!** Ready for Module 3 perception work.
