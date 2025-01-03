import time
import numpy as np
import rospy
import torch
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from tf import transformations
from visualization_msgs.msg import Marker
import tf2_ros
import threading
import tf.transformations
from collections import deque
from std_srvs.srv import Empty
import os

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame

"""使用gazebo定位"""

# 雷达线程
class LaserThread(threading.Thread):
    def __init__(self, ld_range, ld_num, ld_GN):
        super(LaserThread, self).__init__()
        # self._ld_adapter = torch.compile(self._ld_adapter)
        self.ld_range = ld_range
        self.ld_num = ld_num # 雷达线数
        self.ld_GN = ld_GN # 雷达归组线数
        self.ld_g_num = int(ld_num/ld_GN) # 传递给agent的雷达状态线数
        self.laser_data = 3*torch.ones(self.ld_g_num)

    def _callback(self, msg):
        # msg is the raw LiDAR data(360°), dtype=tuple, len=72
        self.laser_data = self._ld_adapter(msg.ranges)  # torch.tensor, len=24, in m

    def _ld_adapter(self,ld_data):
        # Input: ld_data is the raw LiDAR data(360°), dtype=tuple, len=360
        # Output: the grouped&clampled ld_data, angle_range = 360°, len=24=360/15 (group=15), in m
        ld_data = torch.tensor(ld_data).clamp_(0, self.ld_range)
        ld_result_grouped, _ = torch.min(ld_data.reshape(self.ld_g_num,self.ld_GN), dim=-1, keepdim=False) # 360 -> 24
        return ld_result_grouped # m

    def run(self):
        rospy.Subscriber("/scan", LaserScan, self._callback, queue_size=1)
        print("----------Subscribing laser data----------")
        rospy.spin()





class Gazebo_fml_robot_env():
    def __init__(self, **params):
        if len(params)==0: self.__dict__.update(**default_cfg) # Use default configration
        else: self.__dict__.update(params) # Use user's configration
        self.ld_range /= 100  # cm -> m
        self.ld_g_num = int(self.ld_num / self.ld_GN) # state中的雷达线数
        self.main_frame = 'map'

        self.target_area = 0.3
        self.car_state_tc = torch.zeros(3)  # (x,y,theta) in m; For state generation

        self.rate = rospy.Rate(self.frequency)

        # 创建gazebo位置设置服务:
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # 创建costmap清空服务
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # Historic trajectory puber:
        self.history_puber = rospy.Publisher("history_traj", Path, queue_size=1)  # topic name: /history_traj

        # Start tf listener, get the world coordinates of AGV
        self.pose_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.pose_buffer)

        # Start lidar thread, subscribing lidar data
        self.sub_laser = LaserThread(self.ld_range, self.ld_num, self.ld_GN)
        self.sub_laser.start()


        # 2D Nav Goal publisher
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        # 创建目标PoseStamped消息
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"  # 设置目标的参考坐标系
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = self.target_point[0]
        self.goal.pose.position.y = self.target_point[1]
        self.goal.pose.position.z = 0.0
        qtn = transformations.quaternion_from_euler(0, 0, self.target_point[2])
        self.goal.pose.orientation.x = qtn[0]
        self.goal.pose.orientation.y = qtn[1]
        self.goal.pose.orientation.z = qtn[2]
        self.goal.pose.orientation.w = qtn[3]
        self.target_point = torch.tensor(self.target_point[0:2]) #只要xy

        # Velocity publisher
        self.pub_vel = rospy.Publisher("/fml_robot_movebase/cmd_vel", Twist, queue_size=1)
        self.action = Twist() # 速度为0
        self.pub_vel.publish(self.action) # enable publisher


    def _Set_object_pose(self, obj_name: str, x: float, y: float, z: float, theta_z: float = None) -> None:
        '''设置gazebo中某个object的x,y,z,theta_z'''
        usv_state = SetModelStateRequest()
        usv_state.model_state.model_name = obj_name

        # 设置位置：
        usv_state.model_state.pose.position.x = x
        usv_state.model_state.pose.position.y = y
        usv_state.model_state.pose.position.z = z

        if theta_z is not None:
            qtn = transformations.quaternion_from_euler(0, 0, theta_z)
            usv_state.model_state.pose.orientation.x = qtn[0]
            usv_state.model_state.pose.orientation.y = qtn[1]
            usv_state.model_state.pose.orientation.z = qtn[2]
            usv_state.model_state.pose.orientation.w = qtn[3]

        self.set_state_service(usv_state)
        # self.rate.sleep()  # 延时，等待位置重置生效



    def _get_pose(self):
        """返回 x,y,theta
        https://github.com/XinJingHao/Sparrow-V2.1/blob/main/Images/coordinate_frames.svg"""
        while True:
            try:
                # 时间戳 rospy.Time(0)---选取时间间隔最近的两个坐标系之间的相对关系
                basefootprint_map = self.pose_buffer.lookup_transform("map", "base_footprint", rospy.Time(0))
                x = basefootprint_map.transform.translation.x # m
                y = basefootprint_map.transform.translation.y # m

                # 把四元数转换成欧拉角
                euler = tf.transformations.euler_from_quaternion([basefootprint_map.transform.rotation.x,
                                                                  basefootprint_map.transform.rotation.y,
                                                                  basefootprint_map.transform.rotation.z,
                                                                  basefootprint_map.transform.rotation.w])
                # theta from ros(-pi,pi) to simulator absolute theta(0,2pi)
                if euler[2] < 0: theta = euler[2] + 2*np.pi #car should orients to the x-axis when init!!!
                else: theta = euler[2]

                # 刷新状态，这里-y将世界坐标Z-up的y轴反向(从而与Sparrow中pygame坐标系一致)
                self.car_state_tc[0], self.car_state_tc[1], self.car_state_tc[2] = x,y,theta # for state generation
                break

            except Exception as e:
                # 打印错误信息
                print(f"Can not self._get_pose because: {e}")

    def _fresh_obs(self):
        '''Fresh the win & collide signal'''
        self._get_pose() # fresh car_state_tc: (x:m, y:m, theta:rad)
        xy_in_target = self.car_state_tc[0:2] - self.target_point # 小车在以target为原点的坐标系下的坐标, m
        D2T = torch.norm(xy_in_target, p=2) # Unnormalized D2T, m
        self.win = D2T < self.target_area


        # LiDAR:
        ld_data = torch.clamp(self.sub_laser.laser_data, 0, self.ld_range)  # (0,3) m
        self.collide = ld_data.min() < 0.13 #小车半径0.09, 雷达最低扫描范围0.11(精度0.01)




    def reset(self):
        self.cnt = 1 # step counter

        # 重置位置
        self._Set_object_pose(obj_name='fml_robot_movebase', x=self.startpose[0], y=self.startpose[1], z=0,theta_z=self.startpose[2])  # 重置被控小车在gazebo中的位置
        self.rate.sleep()  # 延时，等待位置设置生效

        # 清空代价地图，重新评估
        self.clear_costmaps_srv()

        # 清空速度
        for _ in range(5):
            self.pub_vel.publish(self.action)
            self.rate.sleep()  # 延时，等待位置设置生效

        # 重置历史轨迹
        self.history_traj = Path()
        self.history_traj.header.frame_id = self.main_frame




        # 刷新状态
        self._fresh_obs()

        # 重新发布导航目标点
        for _ in range(10):
            # self.goal.header.stamp = rospy.Time.now()
            self.goal_publisher.publish(self.goal)
            self.rate.sleep()  # 延时，等待位置设置生效


    def _pub_historic_trajectory(self) -> None:
        '''将robot当前时刻位置追加到history_traj中，然后发布history_traj'''
        pose = PoseStamped()
        pose.header.frame_id = self.main_frame  # Coordinate frame reference
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.car_state_tc[0].item()
        pose.pose.position.y = self.car_state_tc[1].item()
        self.history_traj.poses.append(pose)
        self.history_puber.publish(self.history_traj)


    def get_traj_length(self) -> float:
        '''计算历史轨迹(self.history_traj)的长度'''
        length = 0.0
        for i in range(len(self.history_traj.poses) - 1):
            p1 = self.history_traj.poses[i].pose.position
            p2 = self.history_traj.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            length += math.sqrt(dx * dx + dy * dy)
        return length


    def step(self):
        '''刷新win和dead'''
        self._fresh_obs()

        '''记录历史轨迹'''
        if self.cnt % 2 == 0: self._pub_historic_trajectory() # 每X步记录一次历史轨迹
        self.cnt += 1

        self.rate.sleep()  # 延时，等待配置生效

        return self.win, self.collide

