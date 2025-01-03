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
    git clone https://github.com/XinJingHao/OPCD-Navigation.git
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

    If you are in China, you can use the following command to speed up installation:
   ```bash
   cd ~/ros_motion_planning
   conda activate opcd
   pip3 install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple/
   ```
   

9. Install ROS packages:
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

10. Compile the code:
   
   **NOTE: Please refer to [#48](https://github.com/ai-winter/ros_motion_planning/issues/48) if you meet libignition dependency error.**

    ```bash
    cd ~/ros_motion_planning/scripts/
    conda activate opcd
    ./build.sh  # you may need to install catkin-tools using: sudo apt install python-catkin-tools
    ```


## Quick Start
1. Launch the simulation environments:
    ```bash
    cd ~/ros_motion_planning/scripts/
    ./ColorDynamic.sh # wait for the Gazebo and Rviz to get ready
    ```

2. Start the **OkayPlan** global planner:
    ```bash
    cd ~/ros_motion_planning/src/sim_env/scripts/OkayPlan_ColorDynamic/Play/OkayPlan/
    conda activate opcd
    python OkayPlan_main.py
    ```

3. Start the **ColorDynamic** local planner:
    ```bash
    cd ~/ros_motion_planning/src/sim_env/scripts/OkayPlan_ColorDynamic/Play/ColorDynamic/
    conda activate opcd
    python ColorDynamic_main.py
    ```

4. Wait for about 5 seconds until the OkayPlan and ColorDynamic are ready

5. Use **2D Nav Goal** in Rviz to set the navigation goal.

6. Moving!

7. You can use the other script to shutdown them rapidly.

    ```bash
    cd ~/ros_motion_planning/scripts/
    ./killpro.sh
    ```


## Remarks

### System Information:
```c++
    Ubuntu=20.04
    ROS=Noetic
    gazebo=11.11.0
    rviz=1.14.20
    python=3.8.3
    CUDA Driver=550.120
    CUDA Version=12.4
```

### File structure:
```bash
~/ros_motion_planner
├── 3rd
├── docs
├── docker
├── assets
├── scripts # .sh scripts for start simulation
└── src
    ├── core
    │   ├── common
    │   ├── path_planner # Classical global planner
    │   └── controller # Classical local planner
    ├── pgm_map_creator # Occupancy grid map generator
    ├── sim_env             # simulation environment
    │   ├── config
    │   ├── launch # Launch files for OPCD and Classical Planners
    │   ├── maps
    │   ├── meshes
    │   ├── models
    │   ├── rviz
    │   ├── scripts # Python scripts of OkayPlan and ColorDynamic
    │   ├── urdf # urdf files for Gazebo robot simulation
    │   └── worlds
    ├── plugins
    │   ├── dynamic_rviz_config
    │   ├── dynamic_xml_config
    │   ├── gazebo_plugins
    │   ├── map_plugins
    │   └── rviz_plugins
    └── user_config         # user configure file for Classical Planner
```



## Citing this Project

To cite this repository in publications:

```bibtex
@article{OPCD2025,
title = {OkayPlan and ColorDynamic},
journal = {XXX},
volume = {XXX},
pages = {XXX},
year = {2025},
issn = {XXXX-XXXX},
author = {Jinghao Xin},
}
```

