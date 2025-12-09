---
title: "Hardware Option 1: Digital Twin Workstation"
description: "High-performance local workstation setup for Physical AI development with GPU acceleration"
sidebar_label: "Digital Twin Workstation"
sidebar_position: 2
estimated_time: 4
week: 1
module: 0
prerequisites: []
learning_objectives:
  - "Understand hardware requirements for local development with digital twin simulation"
  - "Evaluate cost-performance trade-offs for workstation components"
  - "Install and verify GPU drivers and CUDA toolkit"
  - "Configure Ubuntu 22.04 for robotics development"
---

# Hardware Option 1: Digital Twin Workstation

## Overview

The **Digital Twin Workstation** is a high-performance local development environment optimized for running Gazebo, Unity ML-Agents, and NVIDIA Isaac Sim with GPU acceleration. This setup provides the best performance for real-time simulation and deep learning model training.

**Best For:** Students with budget for hardware investment who want maximum performance and offline development capability.

**Total Cost:** $1,500 - $2,500 USD

## Minimum Hardware Requirements

### GPU (Critical Component)

**Minimum:** NVIDIA RTX 3060 (12GB VRAM)
**Recommended:** NVIDIA RTX 4070 or RTX 4080 (≥12GB VRAM)
**Why:** Isaac Sim requires ray-tracing capable GPU with minimum 12GB VRAM for high-fidelity physics simulation

| Component | Minimum Spec | Recommended Spec | Cost Estimate |
|-----------|--------------|------------------|---------------|
| **GPU** | RTX 3060 12GB | RTX 4070 16GB | $400 - $600 |
| **CPU** | Intel i7-12700 / AMD Ryzen 7 5800X | Intel i9-13900K / AMD Ryzen 9 7950X | $300 - $500 |
| **RAM** | 32GB DDR4 3200MHz | 64GB DDR5 5600MHz | $100 - $200 |
| **Storage** | 512GB NVMe SSD | 1TB NVMe SSD (Gen4) | $80 - $150 |
| **Motherboard** | B660/B550 chipset | Z790/X670 chipset | $150 - $250 |
| **PSU** | 750W 80+ Gold | 850W 80+ Platinum | $100 - $150 |
| **Case + Cooling** | Mid-tower ATX | Full-tower with AIO cooling | $100 - $200 |
| **Display** | 1080p 60Hz | 1440p 144Hz | $200 - $400 |

**Total Estimated Cost:** $1,530 - $2,450 USD

## Performance Expectations

| Task | RTX 3060 (Min) | RTX 4070 (Rec) | RTX 4080 (High-End) |
|------|----------------|----------------|---------------------|
| Gazebo Simulation | 30-40 FPS | 60+ FPS | 60+ FPS |
| Isaac Sim (Medium Quality) | 20-30 FPS | 45-60 FPS | 60+ FPS |
| Isaac Sim (High Quality) | 10-15 FPS | 30-45 FPS | 60 FPS |
| Unity ML-Agents Training | 3-4 hours/1M steps | 2-3 hours/1M steps | 1.5-2 hours/1M steps |
| VLA Model Inference (7B params) | 2-3 tokens/sec | 5-7 tokens/sec | 10-12 tokens/sec |

## Software Requirements

### Operating System

**Required:** Ubuntu 22.04 LTS (Jammy Jellyfish)

**Why Ubuntu?** ROS 2 Humble and Isaac Sim officially support Ubuntu 22.04. While other Linux distributions work, Ubuntu ensures maximum compatibility.

**Installation Steps:**
1. Download Ubuntu 22.04 Desktop ISO from [ubuntu.com](https://ubuntu.com/download/desktop)
2. Create bootable USB with Rufus (Windows) or Etcher (Linux/Mac)
3. Boot from USB and select "Install Ubuntu"
4. Choose "Erase disk and install Ubuntu" (or dual-boot if preferred)
5. Create user account and set password
6. Complete installation (15-20 minutes)

### GPU Drivers and CUDA

**Step 1: Install NVIDIA Driver**

```bash
# Add NVIDIA driver PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver (version 535 or later)
sudo ubuntu-drivers autoinstall

# Reboot system
sudo reboot
```

**Step 2: Verify Driver Installation**

```bash
# Check NVIDIA driver
nvidia-smi

# Expected output:
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 535.xx.xx    Driver Version: 535.xx.xx    CUDA Version: 12.2     |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |===============================+======================+======================|
# |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
# | 30%   45C    P8    15W / 220W |    500MiB / 12288MiB |      5%      Default |
# +-------------------------------+----------------------+----------------------+
```

**Step 3: Install CUDA Toolkit 12.2**

```bash
# Download CUDA Toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.2.0/local_installers/cuda-repo-ubuntu2204-12-2-local_12.2.0-535.54.03-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-12-2-local_12.2.0-535.54.03-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-12-2-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```

**Step 4: Add CUDA to PATH**

```bash
# Add to ~/.bashrc
echo 'export PATH=/usr/local/cuda-12.2/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA installation
nvcc --version

# Expected output:
# nvcc: NVIDIA (R) Cuda compiler driver
# Cuda compilation tools, release 12.2, V12.2.xxx
```

## Verification Checklist

After completing installation, verify your setup:

- [ ] `nvidia-smi` shows GPU information without errors
- [ ] CUDA Toolkit installed: `nvcc --version` returns version 12.2+
- [ ] System has 32GB+ RAM: `free -h`
- [ ] Storage has 100GB+ free: `df -h`
- [ ] Ubuntu 22.04 kernel version: `uname -r` (should be 5.15+)
- [ ] Python 3.10+ installed: `python3 --version`
- [ ] Internet connection stable (>50 Mbps download recommended)

## Performance Optimization Tips

### 1. Enable Hardware Acceleration

```bash
# Install Vulkan drivers for better graphics performance
sudo apt install mesa-vulkan-drivers vulkan-tools
vulkaninfo | grep "deviceName"
```

### 2. Increase Swap Space (for 32GB RAM systems)

```bash
# Create 16GB swap file
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 3. Disable Unnecessary Services

```bash
# Disable bluetooth (if not needed)
sudo systemctl disable bluetooth

# Disable printer service (if not needed)
sudo systemctl disable cups

# Check running services
systemctl list-units --type=service --state=running
```

## Troubleshooting

### Issue: NVIDIA driver installation fails

**Solution:**
```bash
# Remove existing NVIDIA drivers
sudo apt purge nvidia-*
sudo apt autoremove

# Reinstall clean
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Issue: `nvidia-smi` shows "Failed to initialize NVML"

**Solution:**
```bash
# Check if nouveau driver is conflicting
lsmod | grep nouveau

# If nouveau is loaded, blacklist it
sudo bash -c "echo blacklist nouveau > /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo bash -c "echo options nouveau modeset=0 >> /etc/modprobe.d/blacklist-nvidia-nouveau.conf"
sudo update-initramfs -u
sudo reboot
```

### Issue: Isaac Sim crashes with "Out of Memory"

**Possible Causes:**
- GPU VRAM exhausted (need 12GB+ for high-quality scenes)
- System RAM insufficient (need 32GB+ for large simulations)
- Too many physics objects in scene

**Solutions:**
1. Reduce scene complexity (fewer objects)
2. Lower render quality in Isaac Sim settings
3. Close other GPU-intensive applications
4. Increase system swap space

## Next Steps

Once your workstation is verified:

1. **Install ROS 2 Humble** → [Software: ROS 2 Setup](/docs/setup/software-ros2)
2. **Install Isaac Sim** → [Software: Isaac Sim Setup](/docs/setup/software-isaac-sim)
3. **Set Up Development Tools** → [Setup Overview](/docs/setup)

## Cost Breakdown Summary

| Item | Cost Range | Notes |
|------|-----------|-------|
| Core Components | $1,230 - $1,700 | GPU, CPU, RAM, Storage, Mobo, PSU |
| Peripherals | $300 - $750 | Display, keyboard, mouse, case, cooling |
| **Total** | **$1,530 - $2,450** | One-time investment, 3-5 year lifespan |
| **Cost per Week** | **$10 - $16** | Amortized over 3 years (156 weeks) |

**Comparison to Cloud:** Digital twin workstation pays for itself after 300-500 hours of cloud GPU usage (approximately 6-9 months of this course).

---

**Hardware Verified?** → Continue to [Software Installation](/docs/setup/software-ros2)
