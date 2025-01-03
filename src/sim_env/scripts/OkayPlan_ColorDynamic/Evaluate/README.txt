OkayPlan_main.py中设置终点
ColorDynamic_main.py中设置起点
通过ColorDynamic_main.py控制reset,实现C次evaluation

运行步骤:

# 1: 开启仿真环境
cd ~/ros_motion_planning/scripts
./ColorDynamic.sh
等待仿真环境启动完成...

# 2: 开启OkayPlan全局规划
python OkayPlan_main.py

等待rviz中出现全局路径(OkayPlan启动需要调用torch.compile，需要一定时间)
等待非常重要！！！！


# 3: 开启ColorDynamic局部规划
python ColorDynamic_main.py


开始C轮评估