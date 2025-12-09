---
title: "Nodes and Topics"
description: "Master ROS 2 publish-subscribe communication with nodes and topics for asynchronous data streaming"
sidebar_label: "Nodes & Topics"
sidebar_position: 3
estimated_time: 10
week: 3
module: 1
prerequisites: ["module-1-ros2/ros2-fundamentals"]
learning_objectives:
  - "Create ROS 2 nodes in both Python and C++"
  - "Implement publisher and subscriber patterns for topic-based communication"
  - "Understand message types and how to create custom messages"
  - "Debug communication issues using ROS 2 introspection tools"
difficulty_level: "beginner"
capstone_component: "communication-layer"
---

# Nodes and Topics: Publish-Subscribe Communication

## Overview

Nodes and topics form the backbone of ROS 2's asynchronous communication system. In this chapter, you'll learn to create nodes that publish sensor data and subscribe to command streams, enabling distributed robotic systems.

**Duration:** 10 hours
**Week:** 3-4
**Prerequisites:** ROS 2 Fundamentals, basic Python and C++

## Learning Objectives

By the end of this chapter, you will be able to:

1. Create and manage ROS 2 nodes using Python and C++ client libraries
2. Implement publishers to broadcast data on topics
3. Implement subscribers to receive data from topics
4. Work with standard ROS 2 message types (std_msgs, geometry_msgs, sensor_msgs)
5. Create custom message definitions for application-specific data
6. Use rqt_graph and ros2 topic tools to visualize and debug communication

## Chapter Contents

### 1. Understanding Nodes

*Content to be added: Node lifecycle, executors, and the ROS 2 computational graph.*

### 2. Topic-Based Communication

*Content to be added: Publish-subscribe pattern, topic naming conventions, Quality of Service (QoS) settings.*

### 3. Creating Publishers

*Content to be added: Python and C++ publisher examples, publishing rates, message construction.*

### 4. Creating Subscribers

*Content to be added: Callback functions, message handling, subscriber patterns.*

### 5. Standard Message Types

*Content to be added: Overview of std_msgs, geometry_msgs, sensor_msgs, and when to use each.*

### 6. Custom Messages

*Content to be added: Defining .msg files, building custom messages, versioning strategies.*

### 7. Quality of Service (QoS)

*Content to be added: Reliability, durability, history policies, and choosing appropriate QoS profiles.*

## Hands-On Labs

**Lab 4.1: Temperature Sensor Publisher**

*Lab instructions to be added: Create a node that publishes simulated temperature readings.*

**Lab 4.2: Multi-Subscriber System**

*Lab instructions to be added: Build a system with multiple subscribers processing the same topic.*

**Lab 4.3: Custom Message Type**

*Lab instructions to be added: Define and use a custom RobotStatus message.*

## Code Examples

### Python Publisher Example

```python
# Dependencies: rclpy>=3.3.0
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Publishing at {self.get_clock().now()}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

*Additional code examples to be added.*

## Assessment

**Practical Project:** Build a sensor fusion node that subscribes to IMU and GPS topics and publishes a fused odometry estimate.

**Success Criteria:**
- Publishes data at 10 Hz minimum
- Handles missing sensor data gracefully
- Uses appropriate QoS settings

## Troubleshooting

*Common issues and solutions to be added.*

## Resources

- [ROS 2 Publisher/Subscriber Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Message Types Reference](https://docs.ros2.org/latest/api/)

---

**Next Chapter:** [Services](/docs/module-1-ros2/services) - Learn request-response communication patterns
