---
title: "Actions"
description: "Implement long-running tasks with progress feedback and cancellation using ROS 2 actions"
sidebar_label: "Actions"
sidebar_position: 5
estimated_time: 6
week: 4
module: 1
prerequisites: ["module-1-ros2/services"]
learning_objectives:
  - "Understand the action communication pattern and when to use it"
  - "Create action servers for long-running tasks with feedback"
  - "Implement action clients with goal handling and cancellation"
  - "Define custom action types for robotics tasks"
difficulty_level: "intermediate"
capstone_component: "communication-layer"
---

# Actions: Long-Running Tasks with Feedback

## Overview

Actions extend the request-response pattern of services by adding support for long-running tasks, periodic feedback, and cancellation. In this chapter, you'll learn to implement actions for navigation, manipulation, and other tasks that require time to complete.

**Duration:** 6 hours
**Week:** 4-5
**Prerequisites:** Understanding of services and asynchronous programming

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain when to use actions instead of topics or services
2. Create action servers that execute long-running tasks
3. Implement action clients that send goals and receive feedback
4. Handle goal acceptance, feedback publishing, and result delivery
5. Implement goal cancellation and preemption
6. Define custom action (.action) files for specific robotics tasks

## Chapter Contents

### 1. Why Actions?

*Content to be added: Comparison with topics and services, use cases (navigation, manipulation, trajectory execution).*

### 2. Action Architecture

*Content to be added: Goal-feedback-result pattern, action server/client model, state machine.*

### 3. Action Components

*Content to be added: Goal, feedback, and result messages; action state transitions.*

### 4. Creating Action Servers

*Content to be added: Python and C++ server implementations, goal handling, feedback publishing.*

### 5. Creating Action Clients

*Content to be added: Sending goals, receiving feedback, handling results, cancellation.*

### 6. Custom Actions

*Content to be added: Defining .action files, message structure, building custom actions.*

## Hands-On Labs

**Lab 5.1: Fibonacci Action Server**

*Lab instructions to be added: Implement an action server that computes Fibonacci sequences with progress feedback.*

**Lab 5.2: Navigation Action Client**

*Lab instructions to be added: Create an action client for robot navigation with waypoint feedback.*

## Code Examples

### Python Action Server Example

```python
# Dependencies: rclpy>=3.3.0, action_tutorials_interfaces
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Create feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Execute the Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()

        # Create result message
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
```

### Python Action Client Example

```python
# Dependencies: rclpy>=3.3.0, action_tutorials_interfaces
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
```

*Additional code examples to be added.*

## Robotics Use Cases

*Content to be added: Navigation (move_base), manipulation (MoveIt), autonomous charging, multi-step task execution.*

## Goal Lifecycle and States

*Content to be added: State diagram showing PENDING, ACTIVE, SUCCEEDED, CANCELED, ABORTED states.*

## Assessment

**Practical Project:** Implement a pick-and-place action server with:
- Goal: Target object pose
- Feedback: Current gripper position and task stage
- Result: Success/failure with error diagnostics
- Cancellation support during approach phase

**Success Criteria:**
- Publishes feedback at 5 Hz minimum
- Handles cancellation gracefully
- Returns detailed error messages on failure

## Troubleshooting

*Common action-related issues to be added: goal timeouts, feedback delays, cancellation race conditions.*

## Resources

- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [Creating Custom Action Types](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html)
- [Action Design Guidelines](https://design.ros2.org/articles/actions.html)

---

**Next Chapter:** [Multi-Node Systems](/docs/module-1-ros2/multi-node-systems) - Integrate all communication patterns into complex robotic systems
