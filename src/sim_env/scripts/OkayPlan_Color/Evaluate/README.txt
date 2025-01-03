OkayPlan_main.py中设置终点
Color_main.py中设置起点
Color_main.py控制reset,实现C次evaluation

运行步骤:

# 1: 开启仿真环境
cd ~/ros_motion_planning/scripts
./ColorDynamic.sh  (PS: OPCD和OPC使用的启动脚本是一样的)
等待仿真环境启动完毕...

# 2: 开启OkayPlan全局规划
python OkayPlan_main.py

等待rviz中出现全局路径(OkayPlan启动需要调用torch.compile，需要一定时间)
等待非常重要！！！


# 3: 开启Color局部规划
python Color_main.py


开始C轮评估