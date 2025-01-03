---------------------.world文件说明----------------------
empty.world: gazebo自带的空地图文件

warehouse.world: 含L1/L3的warehouse

warehouse_L1_L3_L4.world: 含L1/L3/L4的warehouse (被ColorDynamic调用)

warehouse_with_pedestrians.world: 含L1/L3/L4的warehouse (内容与warehouse_L1_L3_L4.world一致;该文件由ros_motion_planning/scripts/main.sh创建，仅被Classic Planner调用)

问：为什么ColorDynamic不直接用warehouse_with_pedestrians.world？
答：担心地图文件被main.sh自动创建覆盖或者修改

注意:为保证仿真结果的一致性，建议将gazebo的Real Time Factor配置在0.7左右(max_step_size应小于等于0.002)
问:为什么是0.7?
答:在2070台式机上，max_step_size=0.002情况下，warehouse_L1_L3_L4.world的最大Real Time Factor=0.7
---------------------.world文件说明----------------------




---------------------创建新world流程----------------------
1.gazebo打开无人，无车环境
2.移动物件，搭建新环境（不能有<static>的物件，否则会乱飞，建议用带<gravity>的非<static>物件。不要对物件使用[复制粘贴/旋转]等操作）
3.保存为.world文件
4.在.world文件里手动复制粘贴添加行人代码
5.使用pgm_map_creator对.world文件建图，方便rviz渲染。详情参考:https://github.com/JZX-MY/pgm_map_creator
---------------------创建新world流程----------------------




---------------------奇怪的BUG-------------------------
1.雷达扫描行人出现断续情况： 需要将laser.xacro中的雷达切换为gpu_ray

2.gazebo中无法添加其他的.sdf物件： sdf版本不对（2070台式机上的gazebo需要1.7的sdf版本）

3.行人过多时，gazebo中的物件乱飞： 这是由于物件<static>=1而gazebo没有对物件进行重新定位计算导致的。
  解决方案：对这些物件设置<static>=0,并且<gravity>=1（此时仿真步长max_step_size不可太大（建议小于0.002），否则会导致物件细微移动）


4. 行人不多时，加载保存的.world地图也出现出现物件乱飞现象：
	上述问题一般是布置.world过程中，对物件的复制粘贴/旋转导致的。在gazebo布置过程中，一般会出现以下错误：
	Node::Advertise(): Error advertising topic [/shelf_11/joint_cmd]. Did you forget to start the discovery service?
	解决方案：在gazebo中布置.world文件时，不要使用[复制粘贴/旋转]等操作

5.行人和围墙之间没有碰撞： pedestrain插件本身的bug
---------------------奇怪的BUG-------------------------

