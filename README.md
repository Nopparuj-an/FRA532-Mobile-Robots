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

Start Micro-ROS node (PC only)
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

## Installation: ROS2 Humble on PC

1. Clone the repository
    
    ```bash
    cd ~
    git clone https://github.com/Nopparuj-an/FRA532-Mobile-Robots.git
    cd FRA532-Mobile-Robots
    ```

2. Build

   ```bash
   colcon build
   ```

## Installation: ROS2 Humble on Raspberry Pi 5 (Raspberry Pi OS)

1. Clone the repository
    
    ```bash
    cd ~
    git clone https://github.com/Nopparuj-an/FRA532-Mobile-Robots.git ros2_docker
    cd ros2_docker
    ```

2. Install Docker

    ```bash
    sudo curl -sSL https://get.docker.com/ | sh
    ```
    
    Optional (run Docker without sudo): [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/)

3. Install Docker compose

    ```bash
    sudo apt install docker-compose
    ```

4. Start Docker containers

   ```bash
   docker-compose up
   ```

5. Build

   Go to http://127.0.0.1:6080/

   Open a terminal and type

   ```bash
   cd /amr-coco-ws/FRA532-Mobile-Robots
   colcon build
   ```
