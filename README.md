This repo is modified from [ai-winter/ros_motion_planning](https://github.com/ai-winter/ros_motion_planning).


## Installation
*Tested on ubuntu 20.04 LTS with ROS Noetic.*

1. Install [ROS](http://wiki.ros.org/ROS/Installation) (Desktop-Full Install *Recommended*).

2. Install git:

    ```bash
    sudo apt install git
    ```

3. Install [Anaconda](https://www.anaconda.com/download)

4. Clone this repo:

    ```bash
    cd ~/
    git clone xxxxxxxxx
    mv OPCD-Navigation ros_motion_planning # rename this repo, very important!

    ```

6. Create a conda virtual environment:

    ```bash
    conda create -n opcd python=3.8.3
    ```

7. Install python dependence:

    ```bash
    cd ~/ros_motion_planning
    conda activate opcd
    pip3 install -r requirements.txt
    ```

8. Install ROS packages:
    ```bash
    sudo apt update
    ```
    
    ```bash
    sudo apt install python-is-python3 \
    ros-noetic-gazebo-plugins \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-control \
    ros-noetic-geometry-msgs \
    ros-noetic-move-base \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    ros-noetic-tf2 \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-urdf \
    ros-noetic-xacro \
    ros-noetic-amcl \
    ros-noetic-base-local-planner \
    ros-noetic-map-server \
    ros-noetic-navfn \
    libgoogle-glog-dev
    ```

9. Compile the code：

10. Launch the simulation environment

11. Run the OPCD navigation system















