---
title: "Hardware Option 3: Cloud GPU Development"
description: "Cloud-based GPU instances for flexible, scalable Physical AI development without upfront hardware costs"
sidebar_label: "Cloud GPU Setup"
sidebar_position: 4
estimated_time: 2
week: 1
module: 0
prerequisites: []
learning_objectives:
  - "Compare cloud GPU providers (AWS, Azure, GCP, Lambda Labs)"
  - "Calculate cloud costs for 13-week course duration"
  - "Set up cloud GPU instance with ROS 2 and Isaac Sim"
  - "Implement cost-saving strategies (spot instances, auto-shutdown)"
---

# Hardware Option 3: Cloud GPU Development

## Overview

**Cloud GPU Development** uses remote GPU instances from providers like AWS, Azure, Google Cloud, or Lambda Labs. This approach eliminates upfront hardware costs and provides scalability, but requires careful cost management.

**Best For:** Students without access to local GPU hardware, remote learners, or those preferring operational expenses over capital investment.

**Total Cost:** $200 - $600 USD (for 13-week course)

## Cloud Provider Comparison

### Cost Comparison (as of December 2025)

| Provider | Instance Type | GPU | vCPUs | RAM | Cost/Hour | Cost/Week (30hrs) | Cost/Course (13 weeks) |
|----------|--------------|-----|-------|-----|-----------|-------------------|----------------------|
| **AWS EC2** | g5.2xlarge | A10G (24GB) | 8 | 32GB | $1.21 | $36.30 | $472 |
| **Azure** | NC6s v3 | V100 (16GB) | 6 | 112GB | $3.06 | $91.80 | $1,193 |
| **Google Cloud** | n1-highmem-8 + T4 | T4 (16GB) | 8 | 52GB | $0.95 | $28.50 | $371 |
| **Lambda Labs** | gpu_1x_a10 | A10 (24GB) | 30 | 200GB | $0.75 | $22.50 | $293 |
| **Paperspace** | A4000 | RTX A4000 (16GB) | 8 | 45GB | $0.76 | $22.80 | $296 |
| **Vast.ai** | Variable | RTX 3090 (24GB) | 8-16 | 32-64GB | $0.30-0.60 | $9-18 | $117-234 |

**Recommended:** Lambda Labs or Google Cloud for best cost-performance balance.

### Feature Comparison

| Feature | AWS | Azure | GCP | Lambda Labs | Paperspace | Vast.ai |
|---------|-----|-------|-----|-------------|------------|---------|
| **Spot Instances** | ✅ (60-70% savings) | ✅ (eviction risk) | ✅ (preemptible VMs) | ❌ | ❌ | ✅ (auction model) |
| **Persistent Storage** | EBS volumes | Managed Disks | Persistent Disk | Included (NVMe) | Included | Variable |
| **Isaac Sim Support** | ✅ (g5 instances) | ✅ (NCv3 series) | ✅ (T4+ GPUs) | ✅ (A10/A100) | ✅ (A4000+) | ✅ (RTX 3090+) |
| **Free Tier** | 750 hrs/month (CPU only) | $200 credit | $300 credit | ❌ | $10 credit | ❌ |
| **Jupyter Notebook** | SageMaker | Azure ML | Vertex AI | ✅ Native | ✅ Native | ❌ |
| **ROS 2 Preinstalled** | ❌ (manual setup) | ❌ | ❌ | ❌ | ❌ | ❌ |
| **Beginner Friendly** | ⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐ |

## Recommended Setup: Lambda Labs

**Why Lambda Labs?**
- Simple pricing ($0.75/hour for A10 GPU)
- Pre-configured for ML/AI (CUDA, PyTorch, TensorFlow pre-installed)
- Fast NVMe storage included
- No complex billing tiers
- Easy SSH access

### Step-by-Step: Lambda Labs Setup

**Step 1: Create Account**

1. Visit [lambdalabs.com/service/gpu-cloud](https://lambdalabs.com/service/gpu-cloud)
2. Sign up with email and add payment method
3. **Important:** Set up billing alerts to avoid surprise charges

**Step 2: Launch GPU Instance**

1. Click "Launch Instance"
2. Select **gpu_1x_a10** (1x A10 GPU, 30 vCPUs, 200GB RAM)
3. Choose region closest to you (lower latency)
4. Select Ubuntu 22.04 LTS
5. Add SSH key:
   ```bash
   # On your local machine, generate SSH key if needed
   ssh-keygen -t ed25519 -C "your_email@example.com"

   # Copy public key
   cat ~/.ssh/id_ed25519.pub
   ```
6. Paste public key into Lambda Labs dashboard
7. Click "Launch" (instance starts immediately)

**Step 3: Connect via SSH**

```bash
# Get instance IP from Lambda Labs dashboard
ssh ubuntu@<INSTANCE_IP>

# First login will show system info:
# Ubuntu 22.04.3 LTS
# GPU: NVIDIA A10 (24GB)
# CUDA: 12.2
# Driver: 535.xx.xx
```

**Step 4: Install ROS 2 Humble**

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Expected: ros2 cli version 0.25.x
```

**Step 5: Install Isaac Sim (for Module 3)**

```bash
# Isaac Sim requires Omniverse Launcher
# Download and install Omniverse
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage

# For headless cloud setup, use Isaac Sim container instead
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
```

## Cost Optimization Strategies

### 1. Use Spot/Preemptible Instances (60-70% Savings)

**AWS Spot Instances:**
```bash
# Example: g5.2xlarge spot price
# On-Demand: $1.21/hour
# Spot: $0.36-0.50/hour (70% savings)
# Risk: Instance can be terminated with 2-minute warning
```

**When to Use Spot:**
- Long-running training jobs with checkpointing
- Development work (can tolerate interruptions)
- Non-critical simulation tasks

**When NOT to Use Spot:**
- Live demonstrations
- Time-sensitive deadlines
- Unsaved work (always commit code frequently!)

### 2. Auto-Shutdown Idle Instances

**Set Up Idle Detection Script:**

```bash
# Create auto-shutdown script
cat > ~/auto_shutdown.sh << 'EOF'
#!/bin/bash
# Shutdown instance if idle for 30 minutes

IDLE_THRESHOLD=1800  # 30 minutes in seconds
LAST_ACTIVITY=$(who -u | awk '{print $6}' | sed 's/old//' | sort -rn | head -1)

if [ -z "$LAST_ACTIVITY" ]; then
    LAST_ACTIVITY=9999
fi

if [ $LAST_ACTIVITY -gt $IDLE_THRESHOLD ]; then
    echo "Instance idle for $LAST_ACTIVITY seconds. Shutting down..."
    sudo shutdown -h now
fi
EOF

chmod +x ~/auto_shutdown.sh

# Add to crontab (runs every 10 minutes)
(crontab -l 2>/dev/null; echo "*/10 * * * * ~/auto_shutdown.sh") | crontab -
```

**Lambda Labs Specific:**
- Lambda Labs bills by the hour (rounded up)
- Stop instance immediately after finishing work
- Use `tmux` or `screen` to keep sessions alive if disconnected

### 3. Use Persistent Volumes Wisely

**Don't:** Store large datasets on instance root volume (expensive)
**Do:** Use object storage (S3, GCS) for datasets and models

```bash
# Install AWS CLI for S3 access
sudo apt install awscli

# Download dataset from S3 only when needed
aws s3 cp s3://your-bucket/dataset.tar.gz /tmp/
tar -xzf /tmp/dataset.tar.gz -C /workspace/

# Upload results back to S3
aws s3 cp /workspace/results/ s3://your-bucket/results/ --recursive

# Delete local copy
rm -rf /tmp/dataset.tar.gz /workspace/dataset/
```

### 4. Schedule Training Jobs for Off-Peak Hours

Some providers offer lower rates during off-peak times:
- **Vast.ai:** Prices fluctuate based on demand
- **AWS/GCP:** Regional pricing varies

### 5. Use Free Tiers and Credits

| Provider | Free Tier | Duration | Suitable For |
|----------|-----------|----------|--------------|
| **Google Cloud** | $300 credit | 90 days | Modules 1-2 (no GPU) |
| **AWS** | $100 credit (students) | 12 months | Module 1 (CPU only) |
| **Azure** | $200 credit | 30 days | Short experiments |
| **Paperspace** | $10 credit | N/A | Testing setup |

**Strategy:** Use free tiers for Module 1 (ROS 2, CPU-only), then switch to paid GPU instances for Modules 2-4.

## Cloud-Specific Limitations

### 1. No Physical Robot Testing

Cloud instances cannot connect to physical robots directly. For capstone project:
- Develop and test in simulation on cloud
- Deploy final code to Jetson Orin Nano or local robot hardware

### 2. Latency for Remote Desktop

**Problem:** Graphical tools (RViz, Gazebo GUI) can be laggy over remote desktop.

**Solutions:**
- Use NoMachine or TurboVNC (optimized for graphics)
- Run headless simulation and visualize locally
- Use X11 forwarding with compression: `ssh -XC ubuntu@<IP>`

### 3. Data Transfer Costs

**Inbound (to cloud):** Usually free
**Outbound (from cloud):** $0.09-0.12 per GB (AWS/GCP)

**Tip:** If downloading large Isaac Sim recordings or datasets, factor in egress costs (~$10-20 per 100GB).

## Sample Cost Calculation: 13-Week Course

**Scenario:** Student using Lambda Labs A10 instance

| Module | Weeks | Hours/Week | Total Hours | Cost/Hour | Total Cost |
|--------|-------|------------|-------------|-----------|------------|
| Module 1: ROS 2 | 3 | 10 | 30 | $0.00* | $0 |
| Module 2: Digital Twin | 2 | 15 | 30 | $0.75 | $23 |
| Module 3: Isaac Sim | 3 | 20 | 60 | $0.75 | $45 |
| Module 4: VLA | 3 | 20 | 60 | $0.75 | $45 |
| Capstone | 2 | 25 | 50 | $0.75 | $38 |
| **Total** | **13** | - | **230** | - | **$151** |

*Module 1 can run on free tier or CPU-only instance

**With Spot Instances (50% savings):** ~$75
**With Conservative Estimate (300 hours):** $225
**With Buffer for Experimentation:** $250-300

## Verification Checklist

After cloud setup, verify:

- [ ] GPU detected: `nvidia-smi`
- [ ] CUDA available: `nvcc --version`
- [ ] ROS 2 Humble installed: `ros2 --version`
- [ ] SSH key configured for passwordless login
- [ ] Auto-shutdown script active (optional but recommended)
- [ ] Billing alerts set (e.g., notify if >$50/week)
- [ ] Data backup strategy in place (S3/GCS)

## Troubleshooting

### Issue: High Latency for GUI Applications

**Solution:**
```bash
# Use NoMachine for better graphics performance
# On cloud instance:
wget https://download.nomachine.com/download/7.10/Linux/nomachine_7.10.1_1_amd64.deb
sudo dpkg -i nomachine_7.10.1_1_amd64.deb

# Download NoMachine client on your local machine
# Connect to <INSTANCE_IP>:4000
```

### Issue: Instance Terminated Unexpectedly (Spot)

**Prevention:**
```bash
# Save work frequently with Git
git add . && git commit -m "Checkpoint"
git push origin main

# Use tmux to keep sessions alive
tmux new -s dev
# Detach with Ctrl+B, D
# Reattach later: tmux attach -t dev
```

### Issue: Out of Disk Space

**Solution:**
```bash
# Check disk usage
df -h

# Clean up Docker images
docker system prune -a

# Clean up apt cache
sudo apt clean

# Remove old kernels
sudo apt autoremove
```

## Next Steps

1. **Install ROS 2** → [Software: ROS 2 Setup](/docs/setup/software-ros2)
2. **Set Up Isaac Sim** → [Software: Isaac Sim Setup](/docs/setup/software-isaac-sim)
3. **Configure Remote Desktop** → For graphical tools (Gazebo, RViz)

## Cost Comparison Summary

| Hardware Option | Upfront Cost | 13-Week Cost | Total | Best For |
|----------------|--------------|--------------|-------|----------|
| **Digital Twin Workstation** | $1,530-2,450 | $0 | $1,530-2,450 | Long-term use, offline work |
| **Jetson + Cloud Hybrid** | $599 | $200-300 | $799-899 | Edge deployment focus |
| **Cloud Only** | $0 | $200-600 | $200-600 | No hardware budget, remote learners |

**Recommendation:** If budget allows, Digital Twin Workstation offers best long-term value. For short-term learning, Cloud GPU is most cost-effective.

---

**Cloud Instance Ready?** → Continue to [Software Installation](/docs/setup/software-ros2)
