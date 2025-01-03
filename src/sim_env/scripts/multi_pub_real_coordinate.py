#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetModelState
import sys, time


if __name__ == '__main__':
    rospy.init_node("pub_real_coordinate")

    # 静态障碍物时，launch文件中的args只有一个参数输入(存放在sys.argv[1])，此时len(sys.argv)==4
    # 动态障碍物时，launch文件中的args有两个参数输入(存放在sys.argv[1]和sys.argv[2])，此时len(sys.argv)==5
    if len(sys.argv) > 4:
        # fml_robot_0和Layer2_X的rate最好 <= spawn_a_XXX_robot.launch中的rate/5, 否则会报TF重复的Warning
        if sys.argv[1] == 'fml_robot_0': rate = rospy.Rate(15) # fml_robot_0
        else: rate = rospy.Rate(10) # Layer2_X
    else: rate = rospy.Rate(1) # B_1_Xg


    dt = 1
    print(f"Wait for {dt} seconds for gazebo initialisation")
    time.sleep(dt)

    # 创建TF广播器
    broadcaster = tf2_ros.TransformBroadcaster()

    # 创建真实状态获取服务
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    while not rospy.is_shutdown():
        try:
            tfs = TransformStamped() # 创建TF数据
            # 由于gazebo启动较慢，launch一开始时会报错，是正常的
            # 填写表头
            tfs.header.frame_id = "map"
            tfs.header.stamp = rospy.Time.now()

            # 静态障碍物时，launch文件中的args只有一个参数输入(存放在sys.argv[1])，此时len(sys.argv)==4
            # 动态障碍物时，launch文件中的args有两个参数输入(存放在sys.argv[1]和sys.argv[2])，此时len(sys.argv)==5
            tfs.child_frame_id = sys.argv[1]
            if len(sys.argv) > 4: tfs.child_frame_id += sys.argv[2] # len==4: box_10_03; # len==5: car0 + /base_footprint

            # 抽取真实数据，写入TF数据
            real_coordinates = model_coordinates(sys.argv[1], "")  # 获取car0(spawn_model那里声明的)在gazebo世界中心坐标系下的pose
            tfs.transform.translation = real_coordinates.pose.position
            tfs.transform.rotation = real_coordinates.pose.orientation

            # 发布TF数据
            broadcaster.sendTransform(tfs)

            # 固定频率
            rate.sleep()
        except:
            pass
