---
title: "Sensor Integration and Perception"
week: 6
module: 2
---

# Sensor Integration and Perception

## Introduction

Sensor integration is crucial for robots to perceive and understand their environment. This module covers various sensors used in robotics and how to process their data effectively.

## Types of Sensors

### Vision Sensors

**RGB Cameras**
- Color image acquisition
- Object detection and recognition
- Visual servoing
- Common: USB cameras, industrial cameras

**Depth Cameras**
- 3D perception
- Obstacle avoidance
- Object grasping
- Technologies: Stereo, structured light, ToF (Time-of-Flight)
- Examples: RealSense, Kinect, ZED

**LiDAR (Light Detection and Ranging)**
- 360-degree scanning
- Precise distance measurements
- SLAM (Simultaneous Localization and Mapping)
- Types: 2D (planar), 3D (spinning, solid-state)

### Proximity Sensors

**Ultrasonic**
- Short-range distance measurement (0.02-4m)
- Low cost
- Affected by surface materials
- Common in mobile robots

**Infrared (IR)**
- Proximity detection
- Line following
- Object detection
- Reflectivity-based

**Time-of-Flight**
- Accurate distance measurement
- Fast response time
- Wide field of view

### Inertial Sensors

**IMU (Inertial Measurement Unit)**
- Accelerometer: Linear acceleration
- Gyroscope: Angular velocity
- Magnetometer: Magnetic field direction
- Sensor fusion for orientation estimation

**GPS/GNSS**
- Outdoor localization
- Waypoint navigation
- Accuracy: 1-10m (consumer), cm-level (RTK)

### Force and Tactile Sensors

**Force/Torque Sensors**
- Measure applied forces
- Located at robot joints or end-effector
- Compliance control
- Safe human-robot interaction

**Tactile Sensors**
- Surface contact detection
- Texture recognition
- Grasp stability
- Types: Resistive, capacitive, optical

### Other Sensors

**Encoders**
- Joint position measurement
- Wheel odometry
- Incremental vs. absolute

**Microphones**
- Voice commands
- Sound localization
- Environmental monitoring

**Temperature Sensors**
- Motor thermal protection
- Environmental monitoring
- Safety systems

## Sensor Fusion

### Why Sensor Fusion?

- **Redundancy**: Multiple sensors provide backup
- **Complementary Information**: Combine strengths of different sensors
- **Improved Accuracy**: Reduce individual sensor errors
- **Robustness**: Handle sensor failures gracefully

### Fusion Techniques

**Kalman Filter**
- Optimal estimation for linear systems
- Predict-update cycle
- Commonly used for IMU + GPS fusion

```python
class KalmanFilter:
    def __init__(self, A, H, Q, R):
        self.A = A  # State transition matrix
        self.H = H  # Observation matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance

    def predict(self, x, P):
        x = self.A @ x
        P = self.A @ P @ self.A.T + self.Q
        return x, P

    def update(self, x, P, z):
        y = z - self.H @ x  # Innovation
        S = self.H @ P @ self.H.T + self.R
        K = P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        x = x + K @ y
        P = (np.eye(len(x)) - K @ self.H) @ P
        return x, P
```

**Extended Kalman Filter (EKF)**
- Handles non-linear systems
- Linearization around current estimate
- Used in robot localization

**Particle Filter**
- Non-parametric Bayesian filter
- Represents belief as set of particles
- Good for multi-modal distributions

**Complementary Filter**
- Simple and computationally efficient
- Combines high-frequency and low-frequency data
- Common for IMU attitude estimation

## Perception Pipeline

### Image Processing

1. **Acquisition**: Capture image from camera
2. **Preprocessing**: Denoise, enhance, correct distortion
3. **Feature Extraction**: Edges, corners, blobs
4. **Segmentation**: Separate objects from background
5. **Recognition**: Identify objects, classify

**Common Libraries**:
- OpenCV: Computer vision algorithms
- PIL/Pillow: Image manipulation
- scikit-image: Image processing

### Point Cloud Processing

1. **Acquisition**: LiDAR or depth camera
2. **Filtering**: Remove noise and outliers
3. **Downsampling**: Reduce computational load
4. **Segmentation**: Cluster points into objects
5. **Registration**: Align multiple clouds
6. **Object Recognition**: Match to known models

**Common Libraries**:
- PCL (Point Cloud Library)
- Open3D
- ROS perception packages

### Object Detection

**Classical Methods**
- Haar Cascades
- HOG (Histogram of Oriented Gradients)
- SIFT, SURF features

**Deep Learning Methods**
- YOLO (You Only Look Once): Real-time detection
- SSD (Single Shot Detector)
- Faster R-CNN: High accuracy
- EfficientDet: Balanced speed/accuracy

```python
# YOLO example with ROS 2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')

def detect_objects(image):
    results = model(image)
    detections = []

    for result in results:
        for box in result.boxes:
            detection = {
                'class': model.names[int(box.cls)],
                'confidence': float(box.conf),
                'bbox': box.xyxy[0].tolist()
            }
            detections.append(detection)

    return detections
```

## ROS 2 Sensor Integration

### Camera Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.033, self.publish_image)  # 30 FPS
        self.bridge = CvBridge()
        self.camera = cv2.VideoCapture(0)

    def publish_image(self):
        ret, frame = self.camera.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            self.publisher.publish(msg)
```

### LiDAR Integration

```python
from sensor_msgs.msg import LaserScan

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = self.get_lidar_ranges()  # From sensor
        self.publisher.publish(scan)
```

### IMU Integration

```python
from sensor_msgs.msg import Imu

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz

    def publish_imu(self):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Orientation (from sensor fusion)
        imu_msg.orientation.x, imu_msg.orientation.y = ...

        # Angular velocity (from gyroscope)
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y = ...

        # Linear acceleration (from accelerometer)
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y = ...

        self.publisher.publish(imu_msg)
```

## Calibration

### Camera Calibration
- Correct lens distortion
- Estimate intrinsic parameters (focal length, principal point)
- Estimate extrinsic parameters (position, orientation)
- Tools: OpenCV calibration, ROS camera_calibration

### Sensor-to-Sensor Calibration
- Determine relative positions between sensors
- Hand-eye calibration for camera-robot
- LiDAR-camera calibration for fusion
- IMU-camera synchronization

### Example: Chessboard Calibration
```python
import cv2
import numpy as np

# Prepare object points
objp = np.zeros((6*9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

objpoints = []  # 3D points
imgpoints = []  # 2D points

# Find chessboard corners
for image in calibration_images:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)
```

## Best Practices

1. **Sensor Selection**: Choose sensors based on task requirements
2. **Redundancy**: Use multiple sensors for critical measurements
3. **Calibration**: Regularly calibrate sensors
4. **Data Validation**: Check sensor data for errors and outliers
5. **Synchronization**: Timestamp and synchronize multi-sensor data
6. **Power Management**: Monitor sensor power consumption
7. **Environmental Factors**: Account for lighting, weather, interference

## Resources
- OpenCV Documentation: docs.opencv.org
- PCL Tutorials: pointclouds.org/documentation/tutorials
- ROS 2 Perception: docs.ros.org/en/rolling/Tutorials
- Sensor Datasheets: Manufacturer websites
