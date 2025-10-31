# RoboCollector Project

## Overview

The **RoboCollector** is an autonomous block collection system for the Hiwonder JetRover robot. It demonstrates integration of computer vision, navigation, and robotic manipulation to autonomously locate, pick up, and collect colored blocks.

## Architecture

The project consists of 5 main packages:

### 1. `robocollector_vision`

- **Node**: `vision_node`
- **Function**: Color-based object detection using HSV color space
- **Publishes**: `/vision_node/color_info` (ColorInfo), `/vision_node/result_image` (Image)
- **Services**: `~/enter`, `~/exit`, `~/init_finish`

### 2. `robocollector_manipulation`

- **Node**: `pickup_node`
- **Function**: Controls the robotic arm for picking and placing objects
- **Services**: `~/enter`, `~/exit`, `~/pickup`, `~/place`, `~/home`

### 3. `robocollector_controller`

- **Node**: `controller_node`
- **Function**: State machine coordinator that orchestrates the entire mission
- **Subscribes**: `/vision_node/color_info`
- **Publishes**: `/cmd_vel` (Twist)
- **Services**: `~/enter`, `~/exit`, `~/set_target`

### 4. `robocollector_navigation`

- **Function**: SLAM and autonomous navigation integration (to be implemented)

### 5. `robocollector_bringup`

- **Function**: Launch file orchestration for the entire system

## State Machine

The controller follows this state sequence:

1. **IDLE** - Waiting to start
2. **SEARCHING** - Rotating to find colored blocks
3. **APPROACHING** - Moving toward detected block
4. **ALIGNING** - Fine-tuning position relative to block
5. **PICKING** - Executing pickup sequence with arm
6. **RETURNING** - Navigating back to collection area
7. **PLACING** - Depositing block in collection box
8. **COMPLETE** - Mission complete, ready for next cycle

## Building

```bash
cd ~/JetRover/ros2_ws
colcon build --packages-select robocollector_vision robocollector_manipulation robocollector_controller robocollector_bringup
source install/setup.bash
```

## Running

### Full System

```bash
# Set environment variable for development
export need_compile=False

# Launch all nodes
ros2 launch robocollector_bringup robocollector.launch.py
```

### Individual Nodes (for testing)

```bash
# Vision node only
ros2 launch robocollector_vision vision_node.launch.py

# Pickup node only
ros2 launch robocollector_manipulation pickup_node.launch.py

# Controller node only
ros2 launch robocollector_controller controller_node.launch.py
```

## Testing

### Start the mission

```bash
ros2 service call /controller_node/enter std_srvs/srv/Trigger
```

### Change target color

```bash
ros2 service call /controller_node/set_target interfaces/srv/SetString "{data: 'blue'}"
```

### Stop the mission

```bash
ros2 service call /controller_node/exit std_srvs/srv/Trigger
```

### Monitor detection

```bash
ros2 topic echo /vision_node/color_info
```

### View debug visualization

Launch with debug parameter:

```bash
ros2 run robocollector_vision vision_node --ros-args -p debug:=true
```

## Configuration

### Vision Parameters

Edit: `robocollector_vision/config/vision_params.yaml`

- `target_color`: Default color to detect ('red', 'blue', 'green', 'yellow')
- `min_area`: Minimum pixel area for valid detection
- HSV color ranges can be customized for your environment

### Manipulation Parameters

Edit: `robocollector_manipulation/config/pickup_params.yaml`

- Servo IDs and positions for arm movements
- Calibrate positions based on your specific robot setup

## Development Workflow

1. **Local Development (WSL2)**:

   - Develop and test code locally
   - Use `export need_compile=False` for source workspace

2. **Deploy to Robot**:

   ```bash
   git add robocollector/
   git commit -m "RoboCollector updates"
   git push origin main

   # On JetRover
   ssh ubuntu@192.168.2.101
   cd ~/ros2_ws
   git pull
   colcon build --packages-select robocollector_vision robocollector_manipulation robocollector_controller robocollector_bringup
   export need_compile=True
   ros2 launch robocollector_bringup robocollector.launch.py
   ```

## Next Steps

1. **Calibrate servo positions** for your specific arm configuration
2. **Tune HSV color ranges** for your lighting conditions
3. **Integrate navigation** for autonomous return-to-base
4. **Add depth sensing** for accurate distance estimation
5. **Implement multi-block collection** workflow

## Troubleshooting

### No detections

- Check camera topic: `ros2 topic echo /depth_cam/rgb/image_raw`
- Enable debug mode to visualize detection
- Adjust HSV ranges in config file

### Arm not moving

- Verify servo connections and IDs
- Check robot controller is running: `ros2 node list | grep robot_controller`
- Test individual servo commands

### Service calls fail

- Ensure all nodes are running: `ros2 node list`
- Check service availability: `ros2 service list`
- Verify nodes are initialized: `ros2 service call /vision_node/init_finish std_srvs/srv/Trigger`

## License

Apache-2.0

## Author

Maher Alshirazi Alsabbagh  
Stuttgart, Germany  
October 2025
