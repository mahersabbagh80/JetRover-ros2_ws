# JetRover ROS2 Workspace - AI Coding Instructions

## Architecture Overview

This is a **ROS2 Humble** robotics workspace for the **Hiwonder JetRover** running on Jetson Orin Nano. The codebase follows a modular architecture with clearly separated concerns:

### Core Package Structure

- **`bringup/`** - System initialization and startup orchestration
- **`app/`** - High-level application nodes (AR, object tracking, line following, gesture control)
- **`driver/`** - Hardware abstraction layer (robot controller, kinematics, servos)
- **`peripherals/`** - Sensor interfaces (camera, lidar, joystick)
- **`navigation/`** - SLAM and autonomous navigation
- **`interfaces/`** - Custom message and service definitions
- **`large_models/`** - AI/ML integration (LLM, TTs, voice processing)

### Critical Development Patterns

#### 1. Dual Environment Setup

All launch files use this pattern for development vs. production environments:

```python
compiled = os.environ['need_compile']
if compiled == 'True':
    package_path = get_package_share_directory('package_name')
else:
    package_path = '/home/ubuntu/ros2_ws/src/package_name'
```

- `need_compile=True`: Production (installed packages)
- `need_compile=False`: Development (source workspace)

#### 2. Service-Based Architecture

Nodes expose service interfaces for external control:

```python
# Standard pattern in all app nodes
self.create_service(Trigger, '~/enter', self.enter_srv_callback)
self.create_service(Trigger, '~/exit', self.exit_srv_callback)
self.create_service(Trigger, '~/init_finish', self.get_node_state)
```

#### 3. Heart Beat Pattern

Critical nodes implement heartbeat monitoring for reliability:

```python
from app.common import Heart
Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(...))
```

#### 4. Thread-Safe Resource Management

Sensor processing nodes use threading locks:

```python
self.lock = threading.RLock()
with self.lock:
    # Critical section for shared resources
```

## Build and Deployment Workflow

### Local Development (WSL2)

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Robot Deployment

```bash
# Git-based deployment
git push origin main
ssh ubuntu@192.168.2.101
cd ~/ros2_ws && git pull && colcon build --symlink-install

# Environment setup on robot
source ~/.robotrc  # Contains CUDA, ROS2, and hardware-specific env vars
```

### Launch System

- **System bringup**: `ros2 launch bringup bringup.launch.py`
- **Individual apps**: `ros2 launch app ar_app_node.launch.py`
- **Navigation**: `ros2 launch navigation navigation.launch.py map:=map_01`

## Key Integration Points

### Hardware Communication

- **Robot Controller**: `/ros_robot_controller` - centralized motor, servo, IMU, battery interface
- **Camera**: `/depth_cam/rgb/image_raw` - primary RGB-D camera stream
- **LiDAR**: `/scan` - 2D laser scan for navigation

### Message Interfaces

Custom messages in `interfaces/` package:

- `ObjectInfo.msg` - object detection results with bounding box, class, score
- `SetString.srv` - common string parameter service
- `ColorInfo.msg` - color detection data
- `Point2D.msg`, `Pose2D.msg` - geometric primitives
- `ROI.msg`, `LineROI.msg` - region of interest definitions

Complete interface structure:

```
interfaces/
├── msg/
│   ├── ObjectInfo.msg      # Detection: class_name, box[], score, width, height
│   ├── ColorInfo.msg       # Color data with position and RGB values
│   ├── Point2D.msg         # 2D coordinate point
│   └── Pose2D.msg          # 2D pose with position and orientation
└── srv/
    ├── SetString.srv       # Generic string parameter setting
    ├── SetPose2D.srv       # Robot pose setting
    └── GetPose.srv         # Robot pose retrieval
```

### Service Coordination

Apps coordinate through standardized service interfaces:

```bash
# App lifecycle management
ros2 service call /ar_app/enter std_srvs/srv/Trigger
ros2 service call /ar_app/exit std_srvs/srv/Trigger
ros2 service call /ar_app/init_finish std_srvs/srv/Trigger

# App configuration
ros2 service call /ar_app/set_model interfaces/srv/SetString "{data: 'bicycle'}"
ros2 service call /object_tracking/set_target interfaces/srv/SetString "{data: 'person'}"

# Hardware control
ros2 service call /ros_robot_controller/bus_servo/get_state ros_robot_controller_msgs/srv/GetBusServoState
```

## Code Conventions

### Node Initialization Pattern

```python
class MyNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        # Setup publishers, subscribers, services
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
```

### Launch File Structure

All launch files follow this standardized pattern:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        package_path = get_package_share_directory('package_name')
    else:
        package_path = '/home/ubuntu/ros2_ws/src/package_name'

    # Include other launch files
    dependency_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_path, 'launch/dependency.launch.py')),
    )

    # Define nodes with parameters
    my_node = Node(
        package='package_name',
        executable='node_executable',
        output='screen',
        parameters=[{'debug': True}],
    )

    return [dependency_launch, my_node]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
```

**Key Launch Patterns:**

- **Hierarchical includes**: `bringup.launch.py` → `start_app.launch.py` → individual node launches
- **Parameter passing**: Use `launch_arguments` for configuration
- **Conditional nodes**: Based on `LaunchConfiguration` values

### Resource Management

**Required Service Pattern for App Nodes:**

```python
def enter_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "app enter")
    with self.lock:
        # Initialize resources (subscribers, publishers)
        if self.image_sub is None:
            self.image_sub = self.create_subscription(...)
    response.success = True
    return response

def exit_srv_callback(self, request, response):
    self.get_logger().info('\033[1;32m%s\033[0m' % "app exit")
    try:
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
    except Exception as e:
        self.get_logger().error(str(e))
    response.success = True
    return response

def get_node_state(self, request, response):
    response.success = True
    return response
```

**Critical Patterns:**

- Always implement enter/exit/init_finish services for app nodes
- Use `self.destroy_subscription()` in exit callbacks to prevent resource leaks
- Check for `None` before destroying resources
- Use thread locks (`self.lock = threading.RLock()`) for concurrent access

## Debugging and Monitoring

### Essential ROS2 Commands

```bash
# Monitor node status
ros2 node list
ros2 service list /node_name

# Debug data flows
ros2 topic echo /depth_cam/rgb/image_raw
ros2 bag record -a  # Record all topics

# System visualization
rqt_graph
rviz2 -d navigation/rviz/rtabmap.rviz
```

### Log Patterns

Use colored logging for status visibility:

```python
self.get_logger().info('\033[1;32m%s\033[0m' % 'success_message')
self.get_logger().error('\033[1;31m%s\033[0m' % 'error_message')
```

## Hardware-Specific Considerations

- **Jetson Orin Nano**: CUDA-accelerated computer vision, limited memory
- **Robot Base**: 4-wheel mecanum drive, IMU, servo arm
- **Sensors**: RGB-D camera, 2D LiDAR, ring microphone array
- **Network**: WiFi AP mode for remote control, SSH access at 192.168.2.101

## Testing Strategy

### Visual Debugging Pattern

Use the `debug` parameter for OpenCV visualization in computer vision nodes:

```python
self.debug = self.get_parameter('debug').value
# In processing loop:
if self.debug:
    cv2.imshow("result", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
```

### Service-Based Testing

Test app functionality through service calls:

```bash
# Test app lifecycle
ros2 service call /ar_app/enter std_srvs/srv/Trigger
ros2 service call /ar_app/set_model interfaces/srv/SetString "{data: 'bicycle'}"
ros2 service call /ar_app/exit std_srvs/srv/Trigger

# Monitor heartbeat
ros2 service call /ar_app/heartbeat std_srvs/srv/SetBool "{data: true}"
```

### Performance Monitoring

- **Memory**: Monitor Jetson memory usage with `htop` or `nvidia-smi`
- **CPU**: Use `rqt_top` for ROS2 node resource monitoring
- **Network**: Check topic frequencies with `ros2 topic hz /topic_name`
- **System**: Leverage ROS2 launch tests in `test/` directories

## Important Implementation Details

### Computer Vision Processing

Standard pattern for camera-based nodes:

```python
def image_callback(self, ros_image):
    cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
    result_image = np.copy(cv_image)

    with self.lock:
        try:
            result_image = self.image_proc(cv_image, result_image)
        except Exception as e:
            self.get_logger().error(str(e))

    # Debug visualization
    if self.debug:
        cv2.imshow("result", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

    # Publish result
    self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))
```

### Hardware Integration Patterns

Robot controller communication follows this pattern:

```python
# Motor control - array of [id, speed] pairs
self.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

# Servo control with duration and position arrays
self.board.bus_servo_set_position(duration, [[servo_id, position]])

# Sensor reading with error handling
data = self.board.get_imu()
if data is not None:
    ax, ay, az, gx, gy, gz = data
    # Process IMU data...
```

### Startup System Integration

The `bringup/startup_check.py` provides system initialization:

- Checks for hardware devices (`/dev/ring_mic`)
- Displays system info on OLED (SSID, IP address)
- Plays startup beep through buzzer
- Runs microphone test launch automatically
