#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped
from gazebo_msgs.srv import GetModelState
import tf2_ros
import rospy
import sys

'''通过gazebo获取真实定位信息，充当amcl给move_base使用 (方便用python随意设置小车位置)'''

if __name__ == '__main__':
    rospy.init_node("FakeAMCL")
    rate = rospy.Rate(20)

    # 创建TF广播器
    broadcaster = tf2_ros.TransformBroadcaster()

    # map->odom (完全重合)
    map2odom = TransformStamped()
    map2odom.header.frame_id = "map"
    map2odom.child_frame_id = "odom"
    map2odom.transform.translation.x = 0.0
    map2odom.transform.translation.y = 0.0
    map2odom.transform.translation.z = 0.0
    map2odom.transform.rotation.x = 0.0
    map2odom.transform.rotation.y = 0.0
    map2odom.transform.rotation.z = 0.0
    map2odom.transform.rotation.w = 1.0

    # odom->base_footprint
    odom2bf = TransformStamped()

    #创建真实状态获取服务
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    while not rospy.is_shutdown():
        try:
            t_now = rospy.Time.now()
            map2odom.header.stamp = t_now

            # 填写时间及坐标父子级关系
            odom2bf.header.stamp = t_now
            odom2bf.header.frame_id = "odom"
            odom2bf.child_frame_id = "base_footprint"

            # 抽取真实数据，写入TF数据
            real_coordinates = model_coordinates("fml_robot_movebase", "")  # 获取fml_robot_movebase(gazebo中的模型名称)在gazebo世界中心坐标系下的pose
            odom2bf.transform.translation = real_coordinates.pose.position
            odom2bf.transform.rotation = real_coordinates.pose.orientation

            # 发布TF数据
            broadcaster.sendTransform(map2odom)
            broadcaster.sendTransform(odom2bf)

            # 固定频率
            rate.sleep()
        except Exception as e:
            # 打印错误信息
            print(e)


