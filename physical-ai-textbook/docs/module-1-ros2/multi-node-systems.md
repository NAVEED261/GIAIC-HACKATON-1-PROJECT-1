---
title: "Multi-Node Systems"
description: "Design and implement complex robotic systems with multiple interconnected ROS 2 nodes using launch files and best practices"
sidebar_label: "Multi-Node Systems"
sidebar_position: 6
estimated_time: 8
week: 5
module: 1
prerequisites: ["module-1-ros2/actions"]
learning_objectives:
  - "Design multi-node architectures for complex robotic systems"
  - "Create launch files to coordinate multiple nodes and parameters"
  - "Implement inter-node communication patterns for distributed systems"
  - "Debug and monitor multi-node systems using ROS 2 tools"
assessment_type: "ros2-package"
difficulty_level: "intermediate"
capstone_component: "communication-layer"
---

# Multi-Node Systems: Integrating ROS 2 Communication Patterns

## Overview

Real-world robotic systems require multiple cooperating nodes to handle perception, planning, control, and execution. In this chapter, you'll learn to design and implement multi-node architectures using all ROS 2 communication patterns you've learned.

**Duration:** 8 hours
**Week:** 5
**Prerequisites:** Understanding of nodes, topics, services, and actions

## Learning Objectives

By the end of this chapter, you will be able to:

1. Design modular multi-node architectures for robotic systems
2. Create launch files to start and configure multiple nodes
3. Implement parameter servers for dynamic configuration
4. Use namespaces and remapping for node reusability
5. Debug communication issues in distributed systems
6. Apply ROS 2 best practices for scalable system design

## Chapter Contents

### 1. Multi-Node Architecture Patterns

*Content to be added: Perception-planning-control pipeline, hierarchical architectures, distributed sensing.*

### 2. Launch Files

*Content to be added: Python launch files, XML/YAML configurations, parameter passing, node composition.*

### 3. Parameters and Configuration

*Content to be added: Parameter declaration, dynamic reconfiguration, parameter files (YAML).*

### 4. Namespaces and Remapping

*Content to be added: Topic remapping, namespace isolation, multi-robot systems.*

### 5. ROS 2 Component Composition

*Content to be added: Composable nodes, intra-process communication, performance optimization.*

### 6. Debugging Multi-Node Systems

*Content to be added: rqt_graph, ros2 node list, topic echo, logging strategies.*

## Hands-On Labs

**Lab 5.1: Sensor Fusion System**

*Lab instructions to be added: Build a multi-node system with IMU, GPS, and odometry fusion.*

**Lab 5.2: Launch File Configuration**

*Lab instructions to be added: Create a launch file that starts 5+ nodes with parameter configuration.*

## Code Examples

### Python Launch File Example

```python
# Dependencies: launch>=1.0.0, launch_ros>=0.17.0
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_pkg',
            executable='imu_node',
            name='imu_sensor',
            parameters=[{'publish_rate': 100}]
        ),
        Node(
            package='sensor_pkg',
            executable='gps_node',
            name='gps_sensor',
            parameters=[{'publish_rate': 10}]
        ),
        Node(
            package='fusion_pkg',
            executable='sensor_fusion',
            name='fusion_node',
            parameters=[
                {'use_imu': True},
                {'use_gps': True}
            ],
            remappings=[
                ('imu_in', '/imu_sensor/data'),
                ('gps_in', '/gps_sensor/fix')
            ]
        )
    ])
```

### Multi-Node Communication Pattern

```python
# Dependencies: rclpy>=3.3.0
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class SensorFusionNode(Node):
    """Fuses IMU and odometry data for state estimation."""

    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Declare parameters
        self.declare_parameter('fusion_rate', 50.0)
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')

        # Get parameters
        fusion_rate = self.get_parameter('fusion_rate').value
        imu_topic = self.get_parameter('imu_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        # Create subscribers
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        # Create publisher
        self.pose_pub = self.create_publisher(
            PoseStamped, '/fused_pose', 10
        )

        # Fusion timer
        self.timer = self.create_timer(
            1.0 / fusion_rate, self.fusion_callback
        )

        # State variables
        self.latest_imu = None
        self.latest_odom = None

    def imu_callback(self, msg):
        self.latest_imu = msg

    def odom_callback(self, msg):
        self.latest_odom = msg

    def fusion_callback(self):
        if self.latest_imu is None or self.latest_odom is None:
            return

        # Perform sensor fusion (simplified example)
        fused_pose = PoseStamped()
        fused_pose.header.stamp = self.get_clock().now().to_msg()
        fused_pose.header.frame_id = 'map'

        # Fuse position from odometry and orientation from IMU
        fused_pose.pose.position = self.latest_odom.pose.pose.position
        fused_pose.pose.orientation = self.latest_imu.orientation

        self.pose_pub.publish(fused_pose)
        self.get_logger().info('Published fused pose')
```

*Additional code examples to be added.*

## System Design Best Practices

*Content to be added:*
- Single responsibility per node
- Loose coupling through topics
- Parameter-driven configuration
- Graceful degradation strategies
- Logging and diagnostics

## Assessment Project: ROS 2 Multi-Node Package

**Build a complete ROS 2 package demonstrating:**

1. **Sensor Layer** (3 nodes):
   - Simulated camera node (publishes image data)
   - Simulated lidar node (publishes point clouds)
   - IMU node (publishes orientation)

2. **Processing Layer** (2 nodes):
   - Object detection node (subscribes to camera)
   - Obstacle detection node (subscribes to lidar)

3. **Planning Layer** (1 node):
   - Navigation planner (uses services to query goals)

4. **Control Layer** (1 node):
   - Velocity controller (executes navigation commands via actions)

**Requirements:**
- Launch file starts all 7 nodes
- Parameter file configures sensor rates
- Service for setting navigation goals
- Action for executing trajectories
- Proper error handling and logging
- Documentation with architecture diagram

**Success Criteria:**
- All nodes run without errors
- Communication verified with rqt_graph
- Service calls work correctly
- Action provides periodic feedback
- System handles node failures gracefully

## Troubleshooting

*Common issues to be added:*
- Topic name mismatches
- QoS incompatibility between publishers and subscribers
- Launch file parameter errors
- Namespace conflicts

## Resources

- [ROS 2 Launch System](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [ROS 2 Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [ROS 2 Composition](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
- [RQT Tools](https://docs.ros.org/en/humble/Concepts/About-RQt.html)

---

**Module 1 Complete!** You've mastered ROS 2 fundamentals. Continue to [Module 2: Digital Twin Simulation](/docs/module-2-digital-twin) to learn about Gazebo and Unity ML-Agents.
