# amr_coco: A Mecanum Wheeled Mobile Robot with Automatic Mapping and Navigation Capabilities

## Project Overview

This project is a part of the FRA532 Mobile Robots course at the Institute of Field Robotics, King Mongkut's University of Technology Thonburi. The aim is to design and implement a mobile robot equipped with a LIDAR sensor for mapping and localization, and a camera for visual-based odometry. The robot can autonomously navigate in an indoor environment, avoiding obstacles and reaching a specified goal position.

## Team Members
- Nopparuj Ananvoranich
- Paweekorn Buasakorn

## Installation

### ROS2 Humble on PC

1. Clone the repository:
    ```bash
    cd ~
    git clone https://github.com/Nopparuj-an/FRA532-Mobile-Robots.git
    cd FRA532-Mobile-Robots
    ```

2. Build the project:
    ```bash
    colcon build
    ```

### ROS2 Humble on Raspberry Pi 5 (Raspberry Pi OS)

1. Clone the repository:
    ```bash
    cd ~
    git clone https://github.com/Nopparuj-an/FRA532-Mobile-Robots.git ros2_docker
    cd ros2_docker
    ```

2. Install Docker:
    ```bash
    sudo curl -sSL https://get.docker.com/ | sh
    ```
    *Optional*: Run Docker without `sudo`: [Linux post-installation steps for Docker Engine](https://docs.docker.com/engine/install/linux-postinstall/)

3. Install Docker Compose:
    ```bash
    sudo apt install docker-compose
    ```

4. Start Docker containers:
    ```bash
    docker-compose up
    ```
    You should see a terminal output similar to this:

    ![Docker Compose Up](https://github.com/Nopparuj-an/FRA532-Mobile-Robots/assets/122732439/cf68d51b-7aff-460b-a4ca-6380a8ea53c4)

5. Build the project:
    Open a browser and navigate to `http://127.0.0.1:6080/`. Then, open a terminal in the web interface and type:
    ```bash
    cd /amr-coco-ws/FRA532-Mobile-Robots
    colcon build
    ```

## Usage

### Teleoperation

1. Attach a terminal to the ROS2 container (Raspberry Pi 5 only):
    ```bash
    docker attach ros2
    ```
    You should see a terminal similar to this:

    ![ROS2 Terminal](https://github.com/Nopparuj-an/FRA532-Mobile-Robots/assets/122732439/3077c60c-cdbe-4db1-8871-eb77c6989144)

2. Get a controller on your phone:
    ```bash
    cd WORK_SPACE/FRA532-Mobile-Robots/mqtt_teleop
    python3 run.py
    ```
    Once you see the QR code, press `Ctrl + C`.

3. Start the MQTT teleoperation node:
    ```bash
    ros2 run amr_coco mqtt_teleop_mecanum.py
    ```

4. Start the Micro-ROS node (PC only):
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 2000000
    ```

    You should now be able to control the robot. If not, please reset the microcontroller.

### Navigation

1. Start the core node:
    ```bash
    ros2 launch amr_coco amr_coco_bringup.launch.py
    ```

2. Start the mapping node:
    ```bash
    ros2 launch amr_coco_navigation manual_mapping.launch.py
    ```

3. Save the map:
    ```bash
    ros2 launch amr_coco_navigation save_map.launch.py
    ```

4. Start the navigation system:
    ```bash
    ros2 launch linorobot2_navigation navigation.launch.py
    ```

### Accessing noVNC on Raspberry Pi 5 (For RPI5 Only)

#### Method 1: Accessing noVNC Directly on the Raspberry Pi 5

1. Open a web browser on your Raspberry Pi 5.
2. Navigate to http://127.0.0.1:6080/

#### Method 2: Accessing noVNC Remotely from Another Device

1. Ensure your Raspberry Pi 5 and the remote device are connected to the same network.
2. Open a web browser on the remote device.
3. Navigate to http://raspberrypi.local:6080/

## Docker Commands

- Start compose and attach logs:
    ```bash
    docker-compose up
    ```

    > *Tip*: Press `Ctrl + C` to stop (equivalent to `docker-compose stop`).

- Start compose and leave the terminal free:
    ```bash
    docker-compose up -d
    ```

    > *Tip*: This command can be used even while Docker Compose is already running.

    > *Warning*: If you made any changes to the Docker Compose file, the existing containers will be deleted, and new ones will take their place.

- Stop compose and keep the containers:
    ```bash
    docker-compose stop
    ```

- Start existing stopped compose:
    ```bash
    docker-compose start
    ```

- Stop compose and delete all containers:
    ```bash
    docker-compose down
    ```

- Attach a terminal to a Docker container:
    ```bash
    docker attach container_name
    ```
    For example:
    ```bash
    docker attach ros2
    ```

    > *Tip*: Detach without closing the container by pressing `Ctrl + P` then `Ctrl + Q`. If you cannot detach this way, you can only exit the terminal by clicking the X (close) button, otherwise, the container will be shut down.

---

Feel free to contribute to this project by forking the repository and submitting pull requests. For any issues or questions, please open an issue in the repository.
