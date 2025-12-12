---
title: "ROS 2 Fundamentals"
week: 1
module: 1
---

# ROS 2 Fundamentals

## What is ROS 2?

ROS 2 (Robot Operating System 2) is an open-source framework for building robotic applications. It's the next generation of ROS, designed to address the limitations of ROS 1 and provide better support for production robotics systems.

### Key Features

**Real-time Support**: Unlike ROS 1, ROS 2 is built with real-time systems in mind, making it suitable for time-critical robotic applications.

**DDS Middleware**: ROS 2 uses Data Distribution Service (DDS) as its communication layer, providing:
- Quality of Service (QoS) policies
- Reliable data delivery
- Better scalability for multi-robot systems

**Cross-platform**: Runs on Linux, Windows, and macOS, making it accessible to more developers.

**Security**: Built-in security features including authentication and encryption.

## Core Concepts

### Nodes
Nodes are individual processes that perform computation. Each node should have a single, well-defined purpose.

**Example**:
- Camera node: Publishes image data
- Vision node: Processes images
- Control node: Commands robot motors

### Topics
Topics are named buses over which nodes exchange messages. Publishers send messages to topics, and subscribers receive them.

**Publish-Subscribe Pattern**:
- One-to-many communication
- Asynchronous messaging
- Decoupled architecture

### Messages
Messages are data structures that define the type of information exchanged between nodes.

**Standard Message Types**:
- `std_msgs`: Basic data types (String, Int32, Float64)
- `geometry_msgs`: Geometric primitives (Point, Pose, Twist)
- `sensor_msgs`: Sensor data (Image, LaserScan, Imu)

### Services
Services provide synchronous request-response communication between nodes.

**Use Cases**:
- Configuration changes
- One-time calculations
- State queries

### Actions
Actions are for long-running tasks that provide feedback and can be canceled.

**Use Cases**:
- Navigation to a goal
- Grasping objects
- Complex motion sequences

## ROS 2 Architecture

### Layers

1. **OS Layer**: Linux, Windows, or macOS
2. **DDS Layer**: Communication middleware
3. **RCL (ROS Client Library)**: Core ROS functionality
4. **Client Libraries**: Language-specific APIs (rclpy for Python, rclcpp for C++)

### Quality of Service (QoS)

QoS policies control message delivery behavior:

- **Reliability**: Best effort vs. reliable
- **Durability**: Transient local vs. volatile
- **History**: Keep last N messages vs. keep all
- **Deadline**: Expected message frequency

## Common ROS 2 Commands

### Workspace Management
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

# Source workspace
source install/setup.bash
```

### Node Operations
```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /node_name

# Run a node
ros2 run package_name node_name
```

### Topic Operations
```bash
# List topics
ros2 topic list

# Echo topic messages
ros2 topic echo /topic_name

# Publish to topic
ros2 topic pub /topic_name msg_type "data"

# Get topic info
ros2 topic info /topic_name
```

### Service Operations
```bash
# List services
ros2 service list

# Call a service
ros2 service call /service_name srv_type "request"
```

## Creating a Simple ROS 2 Node

### Python Example (Publisher)
```python
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
        msg.data = 'Hello ROS 2'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Python Example (Subscriber)
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String, 'topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## ROS 2 vs ROS 1

### Improvements in ROS 2

**Communication**: DDS middleware instead of custom TCPROS/UDPROS
**Real-time**: Support for real-time systems and deterministic behavior
**Security**: Built-in authentication, encryption, and access control
**Multi-robot**: Better support for multi-robot and distributed systems
**Embedded Systems**: Smaller footprint, works on microcontrollers (micro-ROS)
**Lifecycle**: Managed nodes with lifecycle states

### Migration Considerations

- Most ROS 1 concepts map directly to ROS 2
- Different build system (ament/colcon vs. catkin)
- Python 3 only (ROS 1 used Python 2)
- Different package structure and CMakeLists.txt

## Best Practices

1. **Single Responsibility**: Each node should do one thing well
2. **Loose Coupling**: Use topics for data flow, services for commands
3. **Proper Naming**: Use descriptive node and topic names with namespaces
4. **QoS Configuration**: Choose appropriate QoS policies for your use case
5. **Error Handling**: Always handle failures gracefully
6. **Logging**: Use ROS logging instead of print statements
7. **Parameters**: Make nodes configurable via parameters

## Common Packages

- **tf2**: Transform library for tracking coordinate frames
- **urdf**: Unified Robot Description Format for robot models
- **rviz2**: 3D visualization tool
- **ros2_control**: Framework for robot control
- **navigation2**: Autonomous navigation stack
- **moveit2**: Motion planning framework

## Resources

- Official ROS 2 Documentation: docs.ros.org
- ROS Discourse: discourse.ros.org
- GitHub: github.com/ros2
- ROS Answers: answers.ros.org
