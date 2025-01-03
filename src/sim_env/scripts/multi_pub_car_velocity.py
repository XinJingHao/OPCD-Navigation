#! /usr/bin/env python3

import rospy
import random
import sys
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    car_name = sys.argv[1]
    #2.初始化 ROS 节点:命名(唯一)
    rospy.init_node(car_name+"_V_publisher")
    #3.实例化 发布者 对象
    v_puber = rospy.Publisher(f'/{car_name}/cmd_vel', Twist, queue_size = 1)

    #4.组织被发布的数据，并编写逻辑发布数据
    twist = Twist()
    # 设置循环频率
    rate = rospy.Rate(2)

    sign = (random.random()>0.5) #随机起始方向
    while not rospy.is_shutdown():
        interval = random.randint(1,20)
        for _ in range(interval):
            twist.linear.x = 0.3
            twist.angular.z = sign*random.random()
            v_puber.publish(twist)
            rate.sleep()
        sign*=-1


