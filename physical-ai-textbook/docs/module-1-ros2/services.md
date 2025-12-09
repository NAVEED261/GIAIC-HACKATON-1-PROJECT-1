---
title: "Services"
description: "Implement synchronous request-response communication using ROS 2 services for configuration and command execution"
sidebar_label: "Services"
sidebar_position: 4
estimated_time: 6
week: 4
module: 1
prerequisites: ["module-1-ros2/nodes-topics"]
learning_objectives:
  - "Understand when to use services vs topics in ROS 2 systems"
  - "Create service servers to handle synchronous requests"
  - "Implement service clients to make requests and wait for responses"
  - "Define custom service types for application-specific operations"
difficulty_level: "intermediate"
capstone_component: "communication-layer"
---

# Services: Request-Response Communication

## Overview

While topics enable asynchronous streaming data, services provide synchronous request-response communication for operations that require immediate feedback. In this chapter, you'll learn to implement services for robot configuration, command execution, and state queries.

**Duration:** 6 hours
**Week:** 4
**Prerequisites:** Understanding of nodes and topics

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the difference between services and topics, and choose appropriately
2. Create service servers that process requests and return responses
3. Implement service clients that make blocking or asynchronous calls
4. Define custom service (.srv) files for specific operations
5. Handle service failures and timeouts gracefully
6. Use ros2 service command-line tools for debugging

## Chapter Contents

### 1. Services vs Topics

*Content to be added: When to use each communication pattern, comparison table, use case examples.*

### 2. Service Architecture

*Content to be added: Client-server model, synchronous blocking, request-response flow.*

### 3. Creating Service Servers

*Content to be added: Python and C++ server implementations, callback functions, error handling.*

### 4. Creating Service Clients

*Content to be added: Synchronous vs asynchronous clients, waiting for services, timeout handling.*

### 5. Standard Service Types

*Content to be added: Overview of std_srvs, example_interfaces, common patterns.*

### 6. Custom Services

*Content to be added: Defining .srv files, request and response message structure, building custom services.*

## Hands-On Labs

**Lab 4.1: Configuration Service**

*Lab instructions to be added: Create a service that accepts robot configuration parameters and validates them.*

**Lab 4.2: Sensor Calibration Service**

*Lab instructions to be added: Implement a service for on-demand sensor calibration.*

## Code Examples

### Python Service Server Example

```python
# Dependencies: rclpy>=3.3.0
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: {request.a} + {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Python Service Client Example

```python
# Dependencies: rclpy>=3.3.0
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        return self.future
```

*Additional code examples to be added.*

## Use Cases in Robotics

*Content to be added: Common service patterns - mode switching, calibration, trajectory planning, state queries.*

## Assessment

**Practical Project:** Build a robot arm control system with:
- Service for setting joint positions
- Service for querying current joint states
- Service for executing pre-defined trajectories
- Proper error handling and validation

## Troubleshooting

*Common service-related issues and solutions to be added.*

## Resources

- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Creating Custom Service Types](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

---

**Next Chapter:** [Actions](/docs/module-1-ros2/actions) - Learn about long-running tasks with feedback
