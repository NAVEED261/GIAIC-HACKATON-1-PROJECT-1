---
title: "Foundational Setup"
description: "Hardware requirements, software installation, and development environment configuration for Physical AI course"
sidebar_label: "Setup Guide"
sidebar_position: 1
estimated_time: 4
week: 1
module: 0
prerequisites: []
learning_objectives:
  - "Evaluate and select appropriate hardware setup (Workstation, Jetson, or Cloud)"
  - "Install ROS 2 Humble and verify installation"
  - "Configure Isaac Sim for perception pipeline development"
  - "Set up development environment and tools"
---

# Foundational Setup

## Overview

Before starting the 13-week Physical AI course, you need to choose a hardware setup and install the required software stack. This guide helps you select the right option based on your budget, performance needs, and learning goals.

**Total Setup Time:** 4-6 hours (varies by hardware option)

## Hardware Setup Options

You have **three hardware options** for this course. Each has trade-offs in cost, performance, and use cases.

### Quick Comparison Table

| Feature | Digital Twin Workstation | Jetson Orin Nano | Cloud GPU |
|---------|------------------------|------------------|-----------|
| **Upfront Cost** | $1,530-2,450 | $599 | $0 |
| **13-Week Cost** | $0 (one-time) | $200-300 (cloud supplement) | $200-600 |
| **Total Cost** | $1,530-2,450 | $799-899 | $200-600 |
| **Isaac Sim Support** | ✅ Full support | ❌ Not supported | ✅ Full support |
| **Gazebo Performance** | 30-40 FPS | 10-15 FPS | 30-40 FPS |
| **VLA Inference** | 2-3 tokens/sec | 0.5-1 token/sec (quantized) | 2-3 tokens/sec |
| **Offline Work** | ✅ Yes | ✅ Yes | ❌ Requires internet |
| **Physical Deployment** | ❌ Not portable | ✅ Edge deployment | ❌ Cloud only |
| **Power Consumption** | 400-600W | 7-15W | N/A |
| **Best For** | Long-term use, max performance | Edge deployment testing | No hardware budget |

### Decision Tree

```
START: What's your priority?

├─ **Maximum Performance** + Budget for Hardware Investment
│  └─> Digital Twin Workstation ($1,530-2,450)
│     - Runs all modules locally
│     - Best for offline development
│     - 3-5 year hardware lifespan
│
├─ **Edge Deployment** + Portable Testing
│  └─> Jetson Orin Nano + Cloud Supplement ($799-899)
│     - Test on physical robot hardware
│     - Low power consumption
│     - Use cloud for Isaac Sim (Module 3)
│
└─ **No Upfront Cost** + Remote Learning
   └─> Cloud GPU Only ($200-600)
      - Pay-as-you-go pricing
      - Access from anywhere
      - Requires stable internet
```

## Hardware Setup Guides

Choose your hardware option and follow the corresponding guide:

1. **[Digital Twin Workstation Setup](/docs/setup/hardware-digital-twin)**
   - Hardware requirements (RTX 3060+, 32GB RAM, Ubuntu 22.04)
   - GPU driver and CUDA installation
   - Performance optimization tips
   - **Cost:** $1,530-2,450 (one-time)

2. **[Jetson Orin Nano Edge Kit Setup](/docs/setup/hardware-edge-kit)**
   - Jetson specifications and accessories
   - JetPack 6.0 installation
   - NVMe SSD setup and power configuration
   - **Cost:** $599 hardware + $200-300 cloud supplement

3. **[Cloud GPU Development Setup](/docs/setup/hardware-cloud)**
   - Provider comparison (AWS, GCP, Lambda Labs, Vast.ai)
   - Cost optimization strategies (spot instances, auto-shutdown)
   - Remote desktop configuration
   - **Cost:** $200-600 for 13 weeks

## Software Installation

After setting up hardware, install the software stack in this order:

### Step 1: ROS 2 Humble (Required for All Modules)

**[ROS 2 Humble Installation Guide](/docs/setup/software-ros2)**

- Install ROS 2 Humble Desktop on Ubuntu 22.04
- Configure environment and create workspace
- Verify installation with talker/listener demo
- **Time:** 45-60 minutes

**Module Coverage:** Required for Modules 1-4

### Step 2: Isaac Sim (Required for Module 3 Only)

**[Isaac Sim Installation Guide](/docs/setup/software-isaac-sim)**

- Install NVIDIA Omniverse Launcher
- Install Isaac Sim 2023.1.1
- Configure ROS 2 bridge
- Verify GPU acceleration
- **Time:** 2-3 hours (including download)

**Module Coverage:** Module 3 (Weeks 8-10)

**⚠️ Hardware Requirements:**
- GPU: RTX 3060+ (12GB+ VRAM)
- **NOT supported on Jetson Orin Nano**
- Use cloud GPU if you have Jetson setup

### Step 3: Development Tools (Recommended)

```bash
# Install VS Code
sudo snap install code --classic

# Install Git (if not already installed)
sudo apt install git -y

# Install Python tools
pip3 install --upgrade pip
pip3 install jupyter notebook matplotlib numpy scipy

# Install Docker (optional, for containerized deployments)
sudo apt install docker.io -y
sudo usermod -aG docker $USER
```

## Verification Checklist

After completing setup, verify all components:

### Hardware Verification

- [ ] Ubuntu 22.04 LTS installed and updated
- [ ] GPU detected (if applicable): `nvidia-smi`
- [ ] At least 32GB RAM (workstation) or 8GB (Jetson)
- [ ] 100GB+ free disk space
- [ ] Stable internet connection (>50 Mbps recommended)

### Software Verification

- [ ] ROS 2 Humble installed: `ros2 --version`
- [ ] ROS 2 environment sourced: `echo $ROS_DISTRO` returns `humble`
- [ ] Talker/listener demo works across terminals
- [ ] Workspace created: `~/physical_ai_ws/`
- [ ] Isaac Sim installed (if using Workstation/Cloud for Module 3)
- [ ] Git configured: `git --version`
- [ ] VS Code or preferred IDE installed

### Network Configuration

- [ ] Firewall allows ROS 2 DDS communication (ports 7400-7500)
- [ ] ROS_DOMAIN_ID set (default: 0)
- [ ] SSH access configured (for remote development)

## Common Issues

### Issue: ROS 2 nodes can't communicate between terminals

**Cause:** Firewall blocking DDS multicast.

**Solution:**
```bash
# Allow DDS traffic
sudo ufw allow 7400:7500/udp
sudo ufw allow from 224.0.0.0/4
```

### Issue: Isaac Sim won't launch

**Cause:** Insufficient VRAM or driver issue.

**Solution:**
```bash
# Check VRAM (need 12GB+)
nvidia-smi --query-gpu=memory.total --format=csv

# Update driver if needed
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Issue: Slow performance on Jetson

**Cause:** Not using maximum power mode.

**Solution:**
```bash
# Set to 15W mode
sudo nvpmodel -m 0
sudo jetson_clocks
```

## Cost Summary

### Option 1: Digital Twin Workstation
- **Hardware:** $1,530-2,450 (one-time)
- **Software:** $0 (all free/open-source)
- **13-Week Total:** $1,530-2,450
- **Cost per Week:** $118-188
- **Amortized (3 years):** $10-16/week

### Option 2: Jetson + Cloud Hybrid
- **Jetson Hardware:** $599 (one-time)
- **Cloud Supplement:** $200-300 (13 weeks for Isaac Sim)
- **13-Week Total:** $799-899
- **Cost per Week:** $61-69

### Option 3: Cloud Only
- **Hardware:** $0
- **Cloud GPU:** $200-600 (13 weeks, depends on usage)
- **13-Week Total:** $200-600
- **Cost per Week:** $15-46

## Next Steps

✅ **Hardware and Software Setup Complete!**

**Start Learning:**

1. **[Module 1: ROS 2 Fundamentals](/docs/module-1-ros2)** (Weeks 3-5)
   - Nodes, topics, services, actions
   - Multi-node system architecture
   - ROS 2 package development

2. **[Module 2: Digital Twin Simulation](/docs/module-2-digital-twin)** (Weeks 6-7)
   - Gazebo simulation
   - Unity ML-Agents
   - Sim-to-real transfer

3. **[Module 3: NVIDIA Isaac Sim](/docs/module-3-isaac)** (Weeks 8-10)
   - High-fidelity perception pipeline
   - Object detection and pose estimation
   - Sensor fusion

4. **[Module 4: VLA Models](/docs/module-4-vla)** (Weeks 11-13)
   - Natural language control
   - Task decomposition
   - Autonomous humanoid capstone

## Support and Troubleshooting

If you encounter issues during setup:

1. **Check Troubleshooting Guide:** [Reference → Troubleshooting](/docs/reference/troubleshooting)
2. **Search Glossary:** [Reference → Glossary](/docs/reference/glossary)
3. **Community Forums:** ROS Discourse, NVIDIA Isaac Sim Forums

---

**Setup Complete!** → Start with [Module 1: ROS 2 Fundamentals](/docs/module-1-ros2)
