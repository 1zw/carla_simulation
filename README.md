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

- Install CARLA ROS Bridge by following these **[Instructions](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/#ros-bridge-installation)**.

- Install TensorFlow (2.5.0 or 2.6.0)
    - Software requirements:
        - NVIDIA® GPU driver — 460
        - CUDA® Toolkit — 11.2 [[download](https://developer.nvidia.com/cuda-11.2.0-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=2004&target_type=debnetwork)] [[documentation](https://docs.nvidia.com/cuda/archive/11.2.0/cuda-installation-guide-linux/index.html)]
        - cuDNN SDK 8.1.0 [[download1](https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.1.0.77/11.2_20210127/Ubuntu20_04-x64/libcudnn8_8.1.0.77-1+cuda11.2_amd64.deb)] [[download2](https://developer.nvidia.com/compute/machine-learning/cudnn/secure/8.1.0.77/11.2_20210127/Ubuntu20_04-x64/libcudnn8-dev_8.1.0.77-1+cuda11.2_amd64.deb)] [[documentation](https://docs.nvidia.com/deeplearning/cudnn/archives/cudnn-810/install-guide/index.html)]

    - Install TensorFlow by following these **[Instructions](https://www.tensorflow.org/install/pip)**.

- Install TensorFlow models repository
    ```bash
    source ./venv/bin/activate
    git clone --depth 1 https://github.com/tensorflow/models
    cd models/research
    cp ./object_detection/packages/tf2/setup.py .
    wget -O protobuf.zip https://github.com/protocolbuffers/protobuf/releases/download/v3.17.3/protoc-3.17.3-linux-x86_64.zip
    unzip protobuf.zip -d protobuf
    ./protobuf/bin/protoc object_detection/protos/*.proto --python_out=.
    python -m pip install .
    ```
