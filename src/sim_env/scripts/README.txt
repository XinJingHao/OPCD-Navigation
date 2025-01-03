Classical_Planner:对ros_motion_planner里提供的planner进行评估

ColorDynamic: 仅有局部导航功能，在rviz中用 2D Nav Goal设置目标

OkayPlan_ColorDynamic: 
	用OkayPlan作为全局规划，用ColorDynamic做为局部规划
	/Play: 在rviz中用 2D Nav Goal设置目标，与环境互动
	/Evaluate: 对OPCD导航架构进行评估

OkayPlan_Color: 
	用OkayPlan作为全局规划，用Color做为局部规划
	/Evaluate: 对OPC导航架构进行评估


FakeAMCL.py: 通过gazebo获取精准定位信息，替代acml定位功能包(Designed for Classical_Planner). 
			 在sim_env/launch/app/environment_single.launch.xml中被使用

Layer2_RandomPlanner_RGBD.py: 获取Layer2(已知动态障碍物)的RGBD数据，实现直线运动+基本避障(前方有障碍就转弯)
Layer2_RandomPlanner_laser.py(机器人太多时会卡，已弃用): 获取Layer2(已知动态障碍物)的laser数据，实现直线运动+基本避障(前方有障碍就转弯)

multi_pub_real_coordinate.py: 通过gazebo获取精准定位信息， 为OkayPlan提供Layer1 & 2的位置信息
