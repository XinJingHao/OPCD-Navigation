#! /usr/bin/env python3

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import threading
import rospy
import sys
import numpy as np





# 雷达线程
class CameraThread(threading.Thread):
    def __init__(self,idx):
        super(CameraThread, self).__init__()
        self.idx = idx
        self.depth = []

    def _callback(self, msg):
        self.depth = np.frombuffer(bytes(msg.data), dtype=np.float32)

    def run(self):
        rospy.Subscriber(f"/Layer2_{self.idx}/camera/depth/image_raw", Image, self._callback, queue_size=1)
        print(f"----------Subscribing Layer2_{self.idx} RGBD data----------")
        rospy.spin()


class Gazebo_Layer2_Random_Planner():
    def __init__(self, idx):
        self.idx = idx

        # 转向方向
        if (idx % 2) == 0: self.direction = 2
        else: self.direction = -2

        # Velocity publisher
        self.pub_vel = rospy.Publisher(f"/Layer2_{self.idx}/cmd_vel", Twist, queue_size=1)
        self.action = Twist()
        self.pub_vel.publish(self.action) # enable publisher

        # Start lidar thread, subscribing lidar data
        self.sub_depth = CameraThread(idx)
        self.sub_depth.start()

        # rospy.sleep(0.5) # waiting for initialization

    def control(self):
        try:
            if min(self.sub_depth.depth) < 1.5:
                self.action.linear.x, self.action.angular.z = 0.1, self.direction # turn
                self.pub_vel.publish(self.action)
            else:
                self.action.linear.x, self.action.angular.z = 0.4, 0 # go straight
                self.pub_vel.publish(self.action)
        except:
            # 一开始gazebo还未跑起来时，可能会导致self.sub_depth.laser_data为空，所以要用try
            self.action.linear.x, self.action.angular.z = 0, 0.1*self.direction  # turn slowly
            self.pub_vel.publish(self.action)



if __name__ == '__main__':
    idx = int(sys.argv[1])
    rospy.init_node(f'Gazebo_Layer2_Random_Planner_{idx}')

    # 创建随机planner
    planner = Gazebo_Layer2_Random_Planner(idx)

    # 设置循环频率
    rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    while True:
        planner.control()
        rate.sleep()



















