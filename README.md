# Self-Driving Car Simulation

This Repository contains a ROS2 application for a Self-Driving Car in Carla Simulator.

## Minimum System Requirements

- Ubuntu 20.04 LTS
- Quad-core Intel or AMD processor (2.5 GHz or faster)
- NVIDIA GeForce 1650 GTX or higher
- 8 GB RAM, 32 GB free Disk Space

## Software Prerequisites:

- CARLA 0.9.11
    - Download CARLA simulator packaged version from this **[Link](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.11.tar.gz)** and extract it to the **$HOME** directory.

- ROS2 Foxy Fitzroy 
    - Install ROS2 Desktop by following these **[Instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)**.

- Additional requirements
    - Run the following commands in the terminal.
        ```bash     
        sudo apt-get install python3-numpy python3-pygame

        sudo apt-get install python3-colcon-common-extensions python3-rosdep
        sudo rosdep init
        rosdep update

        sudo apt-get install python3-opencv ros-foxy-image-pipeline

        # Uncomment below line to automatically setup the ros environment on every startup.
        #echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
        ```

## Setup

- Install CARLA ROS Bridge by following these **[Instructions](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/#ros-bridge-installation)**.
