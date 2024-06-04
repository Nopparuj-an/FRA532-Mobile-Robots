# FRA532 Mobile Robots Project

### amr_coco: A Mecanum Wheeled Mobile Robot with automatic mapping and navigation capabilities.

This project is a part of the course FRA532 Mobile Robots at Institute of Field Robotics, King Mongkut's University of Technology Thonburi. The project is to design and implement a mobile robot with automatic mapping and navigation capabilities. The robot is equipped with a LIDAR sensor for mapping and localization, and a camera for visual-based odometry. The robot is able to navigate autonomously in an indoor environment, avoiding obstacles and reaching the goal position.

### Team Members
- Nopparuj Ananvoranich
- Paweekorn Buasakorn

### Usage

Start MQTT teleoperation node
```bash
ros2 run amr_coco mqtt_teleop_mecanum.py
```

Start Micro-ROS node
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 2000000
```

Start core node
```bash
ros2 launch amr_coco amr_coco_bringup.launch.py
```

Start mapping node
```bash
ros2 launch amr_coco_navigation manual_mapping.launch.py
```

Save map
```bash
ros2 launch amr_coco_navigation save_map.launch.py
```