---
title: "Motion Planning for Robots"
week: 9
module: 3
---

# Motion Planning for Robots

## Introduction

Motion planning is the process of finding a collision-free path for a robot to move from a start configuration to a goal configuration. It's a fundamental problem in robotics that combines geometry, kinematics, and optimization.

## Problem Formulation

### Configuration Space (C-space)
- **Configuration**: Complete specification of robot's pose
- **Degrees of Freedom (DOF)**: Number of independent parameters
- **C-space**: Set of all possible configurations
- **C-obstacles**: Regions in C-space representing collisions

### Planning Problem
Given:
- Start configuration q_start
- Goal configuration q_goal
- Obstacles in environment

Find:
- Collision-free path from q_start to q_goal
- Optionally: Optimize for path length, smoothness, or time

## Planning Algorithms

### Graph-Based Methods

**A* Algorithm**
- Finds optimal path on a graph
- Uses heuristic to guide search
- Complete and optimal

```python
import heapq

def astar(start, goal, graph, heuristic):
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in graph.neighbors(current):
            tentative_g = g_score[current] + graph.cost(current, neighbor)

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found
```

**Dijkstra's Algorithm**
- A* without heuristic
- Guaranteed shortest path
- Explores all directions equally

**D* (Dynamic A*)**
- Replans efficiently when obstacles change
- Used in dynamic environments
- Incremental search

### Sampling-Based Methods

**RRT (Rapidly-exploring Random Tree)**
- Probabilistically complete
- Fast for high-dimensional spaces
- Builds tree by random sampling

```python
import numpy as np

class RRT:
    def __init__(self, start, goal, obstacles, bounds, step_size=0.5):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.bounds = bounds
        self.step_size = step_size
        self.tree = {tuple(start): None}

    def plan(self, max_iterations=10000):
        for _ in range(max_iterations):
            # Sample random configuration
            if np.random.rand() < 0.1:  # Goal bias
                q_rand = self.goal
            else:
                q_rand = self.sample_free()

            # Find nearest node in tree
            q_near = self.nearest(q_rand)

            # Extend towards random sample
            q_new = self.steer(q_near, q_rand)

            # Check collision
            if not self.collision_free(q_near, q_new):
                continue

            # Add to tree
            self.tree[tuple(q_new)] = tuple(q_near)

            # Check if goal reached
            if np.linalg.norm(q_new - self.goal) < self.step_size:
                self.tree[tuple(self.goal)] = tuple(q_new)
                return self.extract_path()

        return None  # No path found

    def sample_free(self):
        while True:
            q = np.random.uniform(self.bounds[0], self.bounds[1])
            if not self.in_collision(q):
                return q

    def nearest(self, q):
        distances = [np.linalg.norm(np.array(node) - q) for node in self.tree]
        return np.array(list(self.tree.keys())[np.argmin(distances)])

    def steer(self, q_near, q_rand):
        direction = q_rand - q_near
        length = np.linalg.norm(direction)
        if length > self.step_size:
            return q_near + (direction / length) * self.step_size
        return q_rand
```

**RRT***
- Asymptotically optimal version of RRT
- Rewires tree to improve paths
- Converges to optimal solution

**PRM (Probabilistic Roadmap)**
- Builds roadmap in preprocessing
- Query phase finds path on roadmap
- Good for multi-query scenarios

### Potential Field Methods

**Artificial Potential Fields**
- Attractive potential towards goal
- Repulsive potential from obstacles
- Robot moves along negative gradient

**Advantages**:
- Simple and fast
- Real-time computation

**Disadvantages**:
- Local minima problems
- Oscillations in narrow passages

## Optimization-Based Planning

### Trajectory Optimization
Find trajectory that minimizes cost while satisfying constraints:

**Cost Function**:
- Smoothness (minimize jerk)
- Energy consumption
- Execution time

**Constraints**:
- Collision avoidance
- Joint limits
- Velocity/acceleration limits
- Dynamic feasibility

### Model Predictive Control (MPC)
- Optimize over finite horizon
- Execute first action
- Replan at each time step
- Handles constraints explicitly

```python
import casadi as ca

def mpc_trajectory(x0, goal, obstacles, horizon=10):
    # Decision variables
    x = ca.SX.sym('x', horizon+1, 2)  # States
    u = ca.SX.sym('u', horizon, 2)    # Controls

    # Cost function
    cost = 0
    for k in range(horizon):
        # State cost (distance to goal)
        cost += ca.norm_2(x[k+1] - goal)**2

        # Control effort
        cost += 0.1 * ca.norm_2(u[k])**2

    # Dynamics constraints
    constraints = []
    for k in range(horizon):
        constraints.append(x[k+1] == x[k] + u[k])  # Simple integrator

    # Obstacle avoidance
    for k in range(horizon+1):
        for obs in obstacles:
            constraints.append(ca.norm_2(x[k] - obs['center']) >= obs['radius'])

    # Solve optimization
    nlp = {'x': ca.vertcat(ca.reshape(x, -1, 1), ca.reshape(u, -1, 1)),
           'f': cost,
           'g': ca.vertcat(*constraints)}

    solver = ca.nlpsol('solver', 'ipopt', nlp)
    solution = solver(x0=initial_guess, lbg=0, ubg=0)

    return solution['x']
```

## Planning for Different Robot Types

### Mobile Robots
- 2D or 3D navigation
- Holonomic vs. non-holonomic constraints
- Grid-based or continuous planning

### Manipulators
- High-dimensional configuration space (6-7 DOF)
- Joint limits and singularities
- Inverse kinematics
- Collision checking with environment and self

### Legged Robots
- Footstep planning
- Center of mass trajectory
- Contact constraints
- Dynamic stability

### Aerial Robots
- 3D path planning
- Minimum snap trajectories
- Wind disturbance consideration
- Obstacle avoidance with safety margins

## Collision Detection

### Geometric Primitives
- Bounding boxes (AABB, OBB)
- Spheres and cylinders
- Convex hulls

### Collision Checking Libraries
- FCL (Flexible Collision Library)
- Bullet Physics
- Drake collision engine

```python
# Example with FCL
import fcl

# Create collision objects
mesh1 = fcl.BVHModel()
mesh1.beginModel()
mesh1.addSubModel(vertices1, triangles1)
mesh1.endModel()

obj1 = fcl.CollisionObject(mesh1, transform1)
obj2 = fcl.CollisionObject(mesh2, transform2)

# Check collision
request = fcl.CollisionRequest()
result = fcl.CollisionResult()
fcl.collide(obj1, obj2, request, result)

is_collision = result.is_collision()
```

## Path Smoothing and Post-Processing

### Shortcutting
- Connect non-adjacent waypoints if collision-free
- Iterative improvement
- Reduces path length

### B-Spline Smoothing
- Fit smooth curve through waypoints
- Control smoothness vs. deviation
- Continuous derivatives

### Velocity Profile Generation
- Respect velocity and acceleration limits
- Trapezoidal or S-curve profiles
- Time-optimal trajectories

## ROS 2 Integration

### MoveIt 2
Motion planning framework for manipulators:

```python
from moveit_py import MoveItPy

# Initialize MoveIt
moveit = MoveItPy(node_name="moveit_py")
robot = moveit.get_robot_model()
planning_scene = moveit.get_planning_scene_monitor().planning_scene

# Plan to pose
arm = moveit.get_planning_component("arm")
arm.set_goal_state(target_pose)

# Plan
plan_result = arm.plan()

if plan_result:
    # Execute
    arm.execute(plan_result.trajectory)
```

### Nav2
Navigation stack for mobile robots:

- AMCL: Adaptive Monte Carlo Localization
- Global planner: A*, NavFn, Theta*
- Local planner: DWA, TEB, MPPI
- Recovery behaviors
- Costmaps for obstacle representation

## Advanced Topics

### Multi-Robot Planning
- Centralized vs. decentralized
- Priority-based planning
- Velocity obstacles
- Reciprocal collision avoidance

### Learning-Based Planning
- Neural network policies
- Reinforcement learning
- Imitation learning from demonstrations
- Combining classical and learned approaches

### Real-Time Replanning
- Dynamic environments
- Moving obstacles
- Anytime algorithms
- Safe corridor generation

## Best Practices

1. **Choose Right Algorithm**: Match algorithm to problem characteristics
2. **Tune Parameters**: Adjust step size, goal bias, collision margins
3. **Hierarchical Planning**: Global path + local refinement
4. **Validate Plans**: Check for dynamic feasibility
5. **Graceful Degradation**: Have fallback behaviors
6. **Benchmarking**: Test on diverse scenarios

## Common Pitfalls

- **Ignoring Dynamics**: Kinematically feasible but dynamically infeasible
- **Insufficient Collision Margin**: Paths too close to obstacles
- **Poor Heuristics**: Slow A* convergence
- **Local Minima**: Potential fields getting stuck
- **Over-Smoothing**: Loss of obstacle clearance

## Resources
- OMPL (Open Motion Planning Library): ompl.kavrakilab.org
- MoveIt 2 Documentation: moveit.ros.org
- Nav2 Documentation: navigation.ros.org
- Planning Algorithms Book: lavalle.pl/planning
- Robotics: Modelling, Planning and Control (Siciliano)
