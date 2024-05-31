# FRA532 Mobile Robots Project

### amr_coco: A Mecanum Wheeled Mobile Robot with automatic mapping and navigation capabilities.

This project is a part of the course FRA532 Mobile Robots at Institute of Field Robotics, King Mongkut's University of Technology Thonburi. The project is to design and implement a mobile robot with automatic mapping and navigation capabilities. The robot is equipped with a LIDAR sensor for mapping and localization, and a camera for visual-based odometry. The robot is able to navigate autonomously in an indoor environment, avoiding obstacles and reaching the goal position.

### Team Members
- Nopparuj Ananvoranich
- Paweekorn Buasakorn

## Usage: ROS2

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

Start navigation system
```bash
ros2 launch linorobot2_navigation navigation.launch.py
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

## Usage: Docker

- Start compose and attach logs
    
    ```bash
    docker-compose up
    ```

> [!TIP]
> Press `ctrl + c` is equivalent to `docker-compose stop`

- Start compose and leave terminal free
    
    ```bash
    docker-compose up -d
    ```

> [!TIP]
> You can use this command even while the docker compose is already running

> [!WARNING]
> If you made any changes to the docker compose file, the existing containers will be deleted and a new one will take place.

- Stop compose and keep the containers

    ```bash
    docker-compose stop
    ```

- Start existing stopped compose

    ```bash
    docker-compose start
    ```

- Stop compose and **delete all containers**

    ```bash
    docker-compose down
    ```

- Attach terminal to a docker container

    ```bash
    # docker attach container_name
    docker attach ros2
    ```

> [!TIP]
> Detach without closing the container by `ctrl + p` then `ctrl + q`
> If you cannot detach this way, you can only exit the terminal by clicking the X (close) button, otherwise the container will be shut down.
