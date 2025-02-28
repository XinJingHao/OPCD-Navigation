<p align="center">
  <img src="https://github.com/XinJingHao/Images/blob/main/ColorDynamic/OPCD-logo.png" width="300" /><br>
  <img src="https://img.shields.io/badge/Python-blue" />
  <img src="https://img.shields.io/badge/Navigation-blueviolet" />
  <img src="https://img.shields.io/badge/ROS-ff69b4" />
</p>

<br/>

<p align="center">
  <strong>The OPCD (OkayPlan & ColorDynamic) Navigation System</strong><br>
  <strong>for Unknown & Dynamic Environments</strong>
</p>


<br/>

## Gallery
<p align="center">
  <img src="https://github.com/XinJingHao/Images/blob/main/ColorDynamic/warehouse1.gif" width="250" />
  <img src="https://github.com/XinJingHao/Images/blob/main/ColorDynamic/warehouse2.gif" width="250" />
  <img src="https://github.com/XinJingHao/Images/blob/main/ColorDynamic/warehouse3.gif" width="250" />
</p>

<p align="center">
  <img src="https://github.com/XinJingHao/Images/blob/main/Sparrow_V2/real1.gif" width="250" />
  <img src="https://github.com/XinJingHao/Images/blob/main/Sparrow_V2/real2.gif" width="250" />
  <img src="https://github.com/XinJingHao/Images/blob/main/Sparrow_V2/real3.gif" width="250" />
</p>

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
The video can be found [here](https://www.youtube.com/watch?v=5NMHKDOdt54). 



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
    ```bash
    cd ~/ros_motion_planning/scripts/
    conda activate opcd
    ./build.sh  # you may need to install catkin-tools using: sudo apt install python-catkin-tools
    ```
    *NOTE: Please refer to [#48](https://github.com/ai-winter/ros_motion_planning/issues/48) if you meet libignition dependency error.*


## Quick Start
1. Launch the simulation environments:
    ```bash
    cd ~/ros_motion_planning/scripts/
    ./ColorDynamic.sh # wait for the Gazebo and Rviz to get ready
    ```

2. Start the [OkayPlan](https://github.com/XinJingHao/OkayPlan) global planner:
    ```bash
    cd ~/ros_motion_planning/src/sim_env/scripts/OkayPlan_ColorDynamic/Play/OkayPlan/
    conda activate opcd
    python OkayPlan_main.py
    ```

3. Start the [ColorDynamic](https://github.com/XinJingHao/ColorDynamic) local planner:
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

<br>

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


<br/>

## Related Repos:
- [OkayPlan](https://github.com/XinJingHao/OkayPlan)
- [ColorDynamic](https://github.com/XinJingHao/ColorDynamic)
- [Sparrow-V2](https://github.com/XinJingHao/Sparrow-V2)
- [Sparrow-V3](https://github.com/XinJingHao/Sparrow-V3)
- [OPCD](https://github.com/XinJingHao/OPCD-Navigation)

<br/>

## Citing this Project

To cite this repository in publications:

```bibtex
@article{ColorDynamic,
  title={ColorDynamic: Generalizable, Scalable, Real-time, End-to-end Local Planner for Unstructured and Dynamic Environments},
  author={Jinghao Xin, Zhichao Liang, Zihuan Zhang, Peng Wang, and Ning Li},
  journal={arXiv preprint arXiv:2502.19892},
  year={2025}
}
```

<br/>

## Important Notes:
The ROS simulation environment is built on [ai-winter/ros_motion_planning](https://github.com/ai-winter/ros_motion_planning).

