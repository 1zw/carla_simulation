# Self-Driving Car Simulation

This Repository contains a ROS2 application for a Self-Driving Car in Carla Simulator.

## System Requirements

- Ubuntu 20.04 LTS
- Quad-core or higher Intel or AMD processor (2.5 GHz or faster)
- NVIDIA GeForce 1650 GTX or higher
- Minimum 8 GB RAM, 32 GB free Disk Space

## Software Prerequisites:

- CARLA 0.9.11

- ROS2 Foxy Fitzroy

- CARLA ROS Bridge

### Install Software Prerequisites

- Run below commands in terminal.
    ```bash
    sudo apt install git
    git clone https://github.com/harsha-vk/carla_simulation.git ~/carla-ros-bridge/src/carla_simulation
    cd ~/carla-ros-bridge/src/carla_simulation/assets
    chmod +x setup.sh
    . setup.sh
    ```
