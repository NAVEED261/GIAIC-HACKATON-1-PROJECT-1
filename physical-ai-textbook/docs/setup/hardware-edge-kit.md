---
title: "Hardware Option 2: Jetson Orin Nano Edge Kit"
description: "Portable edge computing kit for on-device robot deployment and testing"
sidebar_label: "Jetson Orin Nano Kit"
sidebar_position: 3
estimated_time: 3
week: 1
module: 0
prerequisites: []
learning_objectives:
  - "Understand edge deployment requirements for humanoid robotics"
  - "Configure Jetson Orin Nano for ROS 2 development"
  - "Evaluate trade-offs between edge and workstation hardware"
  - "Deploy inference models on resource-constrained devices"
---

# Hardware Option 2: Jetson Orin Nano Edge Kit

## Overview

The **Jetson Orin Nano Edge Kit** is an NVIDIA-powered embedded system designed for on-device AI inference and robot deployment. This setup prioritizes portability and power efficiency over raw performance, making it ideal for testing real-world robot deployments.

**Best For:** Students focused on embedded systems, edge deployment, and testing on physical hardware with limited budget.

**Total Cost:** $500 - $700 USD

## What is Edge Computing?

**Edge Computing** means running AI models and robot control directly on the robot's onboard computer (the "edge") rather than in a datacenter or cloud. This approach:

- ✅ **Reduces Latency:** No network round-trip (critical for real-time control)
- ✅ **Works Offline:** No internet dependency
- ✅ **Improves Privacy:** Sensor data stays on device
- ❌ **Limited Compute:** Smaller models and lower-quality simulations
- ❌ **Power Constrained:** Battery life considerations

## Hardware Specifications

### Jetson Orin Nano Developer Kit

| Component | Specification |
|-----------|--------------|
| **GPU** | NVIDIA Ampere (1024 CUDA cores, 32 Tensor cores) |
| **CPU** | 6-core ARM Cortex-A78AE (2.0 GHz) |
| **RAM** | 8GB 128-bit LPDDR5 (102 GB/s bandwidth) |
| **Storage** | MicroSD card (64GB+ recommended) + NVMe SSD support |
| **AI Performance** | 40 TOPS (INT8) |
| **Power** | 7W - 15W configurable TDP |
| **Dimensions** | 100mm × 79mm × 31mm |
| **Weight** | ~300g (0.66 lbs) |
| **Price** | $499 USD (Developer Kit) |

### Required Accessories

| Item | Purpose | Cost |
|------|---------|------|
| **Power Supply** | 19V 4.74A (90W) USB-C PD | Included with Dev Kit |
| **MicroSD Card** | 64GB+ UHS-I (boot drive) | $15 - $30 |
| **NVMe SSD** | 256GB M.2 2280 (storage expansion) | $30 - $50 |
| **USB Keyboard/Mouse** | Setup and debugging | $20 - $40 |
| **HDMI Cable + Monitor** | Initial configuration | $10 - $20 (if not owned) |
| **Ethernet Cable** | Network connectivity | $5 - $10 |
| **USB-C Hub** | Additional ports (optional) | $20 - $40 |

**Total Cost:** $599 - $689 USD (with all accessories)

## Performance Comparison: Jetson vs Workstation

| Task | Jetson Orin Nano | RTX 3060 Workstation |
|------|------------------|---------------------|
| **Gazebo Simulation** | 10-15 FPS (basic scenes) | 30-40 FPS |
| **Isaac Sim** | ❌ Not Supported (insufficient VRAM) | 20-30 FPS |
| **Unity ML-Agents** | ❌ Not Recommended | 3-4 hours/1M steps |
| **VLA Inference (7B)** | ✅ 0.5-1 token/sec (quantized) | 2-3 tokens/sec |
| **Object Detection (YOLOv8)** | ✅ 30 FPS @ 640x640 | 120+ FPS |
| **ROS 2 Node Count** | 20-30 nodes max | 100+ nodes |
| **Power Consumption** | 7-15W | 400-600W (full system) |

**Key Takeaway:** Jetson is **NOT suitable for Isaac Sim** due to 8GB RAM and limited GPU memory. Use for deployment testing and inference only.

## Setup Instructions

### Step 1: Flash JetPack 6.0 to MicroSD Card

**On Your Host Computer (Windows/Linux/Mac):**

1. Download **NVIDIA SDK Manager** from [developer.nvidia.com/sdk-manager](https://developer.nvidia.com/sdk-manager)
2. Install SDK Manager:
   ```bash
   # On Ubuntu host
   sudo apt install ./sdkmanager_[version].deb
   ```
3. Launch SDK Manager and sign in with NVIDIA Developer account
4. Select target hardware: **Jetson Orin Nano Developer Kit**
5. Select JetPack version: **6.0** (Ubuntu 22.04 base)
6. Choose installation method: **Manual Setup** (MicroSD card)
7. Download components (this may take 30-60 minutes)

**Flash MicroSD Card:**
```bash
# Insert 64GB+ MicroSD card into host computer
# SDK Manager will guide through flashing process

# Alternatively, use Etcher for pre-built image
# 1. Download Jetson Orin Nano image from NVIDIA
# 2. Flash with balenaEtcher to MicroSD card
```

### Step 2: Initial Boot and Configuration

1. Insert flashed MicroSD card into Jetson
2. Connect HDMI monitor, keyboard, mouse, Ethernet
3. Connect USB-C power supply (19V 4.74A)
4. Jetson will boot automatically (first boot takes 5-10 minutes)
5. Complete Ubuntu setup wizard:
   - Username: `student` (or your preference)
   - Password: Choose strong password
   - Timezone and language settings

### Step 3: Install NVMe SSD (Optional but Recommended)

**Why?** MicroSD cards are slow for development. NVMe SSD provides 10-20x faster read/write speeds.

```bash
# After first boot, check if NVMe SSD is detected
lsblk | grep nvme

# Expected output if SSD installed:
# nvme0n1     259:0    0   256G  0 disk

# Format and mount NVMe SSD
sudo mkfs.ext4 /dev/nvme0n1
sudo mkdir /mnt/nvme
sudo mount /dev/nvme0n1 /mnt/nvme

# Make mount permanent
echo '/dev/nvme0n1 /mnt/nvme ext4 defaults 0 2' | sudo tee -a /etc/fstab

# Move home directory to NVMe for better performance
sudo rsync -aXS /home/ /mnt/nvme/home/
sudo mv /home /home.bak
sudo ln -s /mnt/nvme/home /home
```

### Step 4: Configure Power Mode

Jetson supports multiple power modes. For robotics development, use **15W mode** for best performance.

```bash
# Check current power mode
sudo /usr/sbin/nvpmodel -q

# Set to 15W mode (maximum performance)
sudo /usr/sbin/nvpmodel -m 0

# Verify GPU is active
sudo tegrastats

# Expected output includes:
# GPU@1300MHz
```

### Step 5: Install Development Tools

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install -y build-essential cmake git wget curl \
    python3-pip python3-dev python3-venv \
    libopencv-dev libboost-all-dev

# Install ROS 2 dependencies (covered in software-ros2.md)
# Install PyTorch for Jetson
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## Use Cases: When to Use Jetson

### ✅ **Ideal For:**

1. **On-Device Inference**
   - Running pre-trained VLA models for robot control
   - Real-time object detection (YOLOv8, DETR)
   - Sensor fusion and localization (EKF, particle filters)

2. **Physical Robot Testing**
   - Deploying code to humanoid platforms (Unitree, Boston Dynamics)
   - Testing latency-sensitive control loops
   - Battery-powered autonomous operation

3. **Edge AI Experiments**
   - Quantized model deployment (INT8, FP16)
   - Model optimization with TensorRT
   - Power efficiency benchmarking

### ❌ **NOT Suitable For:**

1. **High-Fidelity Simulation**
   - Isaac Sim requires 12GB+ VRAM (Jetson has 8GB shared RAM)
   - Gazebo performance limited to basic scenes
   - Unity ML-Agents training too slow

2. **Large Model Training**
   - 8GB RAM insufficient for training transformer models
   - Training should happen on workstation or cloud

3. **Multi-Robot Swarms**
   - Limited RAM for running 50+ ROS 2 nodes simultaneously

## Limitations and Workarounds

| Limitation | Impact | Workaround |
|------------|--------|------------|
| **No Isaac Sim Support** | Can't run Module 3 locally | Use cloud instance or workstation for Module 3 |
| **8GB Shared RAM** | Limited concurrent ROS nodes | Profile and optimize node memory usage |
| **ARM Architecture** | Some x86 binaries incompatible | Use ARM-compatible containers or compile from source |
| **MicroSD Speed** | Slow disk I/O | Upgrade to NVMe SSD (highly recommended) |
| **No Display Port** | HDMI only | Use HDMI monitor or remote desktop (VNC/NoMachine) |

## Hybrid Approach: Jetson + Cloud

Many students use a **hybrid setup**:

- **Jetson Orin Nano:** For Module 1 (ROS 2), Module 4 (VLA inference), and capstone deployment
- **Cloud GPU Instance:** For Module 2 (Unity ML-Agents training) and Module 3 (Isaac Sim)
- **Total Cost:** $599 (Jetson) + $200-300 (cloud usage) = $799-899 over 13 weeks

This approach provides the best balance of cost, portability, and performance.

## Verification Checklist

After setup, verify your Jetson:

- [ ] JetPack 6.0 installed: `cat /etc/nv_tegra_release`
- [ ] GPU active: `sudo tegrastats` shows GPU utilization
- [ ] Power mode set to 15W: `sudo nvpmodel -q`
- [ ] NVMe SSD mounted (if installed): `df -h | grep nvme`
- [ ] CUDA available: `python3 -c "import torch; print(torch.cuda.is_available())"`
- [ ] ROS 2 Humble installed (see software-ros2.md)
- [ ] Network connectivity: `ping google.com`

## Cost Breakdown

| Item | Cost | Notes |
|------|------|-------|
| Jetson Orin Nano Dev Kit | $499 | One-time purchase |
| MicroSD Card 64GB | $20 | Required for boot |
| NVMe SSD 256GB | $40 | Highly recommended |
| Accessories (cables, etc.) | $40 | Keyboard, mouse, HDMI |
| **Total** | **$599** | 3-5 year hardware lifespan |
| **Cloud Supplement** | $200-300 | For Isaac Sim (Modules 3) |
| **Grand Total** | **$799-899** | Complete hybrid setup |

## Next Steps

1. **Install ROS 2 on Jetson** → [Software: ROS 2 Setup](/docs/setup/software-ros2)
2. **Set Up Remote Development** → Use VS Code Remote SSH for comfortable coding
3. **Deploy First Robot Node** → Test inference latency and performance

## Additional Resources

- [NVIDIA Jetson Developer Zone](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [JetPack SDK Documentation](https://docs.nvidia.com/jetson/jetpack/)
- [Jetson Community Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)
- [ROS 2 on Jetson Tutorial](https://nvidia-ai-iot.github.io/ros2_jetson/)

---

**Jetson Configured?** → Continue to [ROS 2 Installation](/docs/setup/software-ros2)
