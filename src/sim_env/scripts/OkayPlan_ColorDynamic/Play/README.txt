用rviz的 Set 2D Navigation Goal来设置导航终点:

运行步骤:

# 1: 开启仿真环境
cd ~/ros_motion_planning/scripts
./ColorDynamic.sh
等待仿真环境启动完成...

# 2: 开启OkayPlan全局规划
python OkayPlan_main.py


# 3: 开启ColorDynamic局部规划
python ColorDynamic_main.py


等待约5秒钟(OkayPlan启动需要调用torch.compile，需要一定时间)


# 4: 在rviz中设置导航点
Enjoy!