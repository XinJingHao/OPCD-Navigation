import numpy as np
import rospy
import torch
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from tf import transformations
from visualization_msgs.msg import Marker
import tf2_ros
import threading
import tf.transformations
from collections import deque

import os

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame

"""使用Gazebo真实数据定位"""

# 雷达线程
class LaserThread(threading.Thread):
    def __init__(self, ld_range, ld_num, ld_GN):
        super(LaserThread, self).__init__()
        # self._ld_adapter = torch.compile(self._ld_adapter)
        self.ld_range = ld_range
        self.ld_num = ld_num # 雷达线数
        self.ld_GN = ld_GN # 雷达归组线数
        self.ld_g_num = int(ld_num/ld_GN) # 传递给agent的雷达状态线数

    def _callback(self, msg):
        # msg is the raw LiDAR data(360°), dtype=tuple, len=72
        self.laser_data = self._ld_adapter(msg.ranges)  # torch.tensor, len=24, in m

    def _ld_adapter(self,ld_data):
        # Input: ld_data is the raw LiDAR data(360°), dtype=tuple, len=240
        # Output: the grouped&clampled ld_data, angle_range = 270°, len=27 in m
        ld_data = torch.tensor(ld_data[25:214]).clamp_(0, self.ld_range) # 240 -> 189 (360° -> 270°)
        ld_result_grouped, _ = torch.min(ld_data.reshape(self.ld_g_num,self.ld_GN), dim=-1, keepdim=False) # 189 -> 27
        return ld_result_grouped # m


    def run(self):
        rospy.Subscriber("/fml_robot_0/scan", LaserScan, self._callback, queue_size=1)
        print("----------Subscribing laser data----------")
        rospy.spin()

# Odom线程: 获取速度
class OdomThread(threading.Thread):
    def __init__(self, ):
        super(OdomThread, self).__init__()
        self.odom_data = torch.zeros(2)

    def _callback(self, msg):
        self.odom_data[0] = msg.twist.twist.linear.x  # v_linear, m/s
        self.odom_data[1] = msg.twist.twist.angular.z # v_angular, rad/s
        # print(self.odom_data)

    def run(self):
        rospy.Subscriber("/fml_robot_0/odom", Odometry, self._callback, queue_size=1)
        print("----------Subscribing odom data----------")
        rospy.spin()


# Marker线程: 订阅GuidePoint
class MarkerThread(threading.Thread):
    def __init__(self, ):
        super(MarkerThread, self).__init__()
        self.guidepoint = torch.zeros(2)

    def _callback(self, msg):
        # 这里-y将世界坐标Z-up的y轴反向(从而与Sparrow中pygame坐标系一致)
        self.guidepoint[0] = msg.pose.position.x  # m
        self.guidepoint[1] = -msg.pose.position.y # m

    def run(self):
        rospy.Subscriber("/GuidePoint", Marker, self._callback, queue_size=1)
        print("----------Subscribing guide point----------")
        rospy.spin()


# 19 editable configurations:
default_cfg = dict(
    ctrl_freq=10, #control frequency
    D=400, # maximal local planning distance
    ld_range=100, # max scanning distance; cm
    ld_num=189, # number of lidar streams in each world
    ld_GN=7, # how many lidar streams are grouped for each group
    startpose = (0,0,0), # x,y,theta of fml_robot when reset
    )


class Gazebo_fml_robot_env():
    def __init__(self, **params):
        '''Robot should orient to the X-axis when init!!!'''
        if len(params)==0: self.__dict__.update(**default_cfg) # Use default configration
        else: self.__dict__.update(params) # Use user's configration
        self.ld_range /= 100  # cm -> m
        self.ld_g_num = int(self.ld_num / self.ld_GN) # state中的雷达线数
        self.main_frame = 'map'

        self.rate = rospy.Rate(self.ctrl_freq)
        self.car_radius = 9 # cm
        self.window_size = 1000 # cm
        self.state_dim = 8 + self.ld_g_num # current_a*2, real_a*2, D2T, α, Vl, Va, ld0, ..., ld23
        self._obs = torch.zeros(self.state_dim)
        self.action_dim = 7 # no stop for agent
        self.delayed_action_state = False # whether show delayed action to the agent

        # 创建gazebo位置设置服务:
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Historic trajectory puber:
        self.history_puber = rospy.Publisher("history_traj", Path, queue_size=1)  # topic name: /history_traj

        # Velocity publisher
        self.pub_vel = rospy.Publisher("/fml_robot_0/cmd_vel", Twist, queue_size=1)
        self.action = Twist()
        self.pub_vel.publish(self.action) # enable publisher

        # Start tf listener, get the world coordinates of AGV
        self.pose_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.pose_buffer)

        # Start odom thread, subscribing velocity
        self.sub_odom = OdomThread()
        self.sub_odom.start()

        # Start lidar thread, subscribing lidar data
        self.sub_laser = LaserThread(self.ld_range, self.ld_num, self.ld_GN)
        self.sub_laser.start()

        # Start marker thread, subscribing guide point
        self.sub_guide = MarkerThread()
        self.sub_guide.start()
        rospy.sleep(5)

        # Navigation:
        self.target_area = 0.3  # m; Enter the central circle (radius=target_area) will be considered win.
        self.v_linear_max = 0.5  # m/s
        self.v_angular_max = 2  # rad/s
        self.a_space = [[0.2*self.v_linear_max, self.v_angular_max], # m/s, rad/s
                        [self.v_linear_max, self.v_angular_max],
                        [self.v_linear_max, 0],
                        [self.v_linear_max, -self.v_angular_max],
                        [0.2*self.v_linear_max, -self.v_angular_max],
                        [-self.v_linear_max, 0],
                        [0.1*self.v_linear_max, 0], # slow down
                        [0,0]] # stop

        self.state_upperbound = self.ld_range * torch.ones(self.state_dim)
        self.state_upperbound[0] = self.v_linear_max # current_a: V_linear
        self.state_upperbound[1] = self.v_angular_max# current_a: V_angular
        self.state_upperbound[2] = self.v_linear_max # real_a: V_linear
        self.state_upperbound[3] = self.v_angular_max# real_a: V_angular
        self.state_upperbound[4] = self.D/100 # Maximal D2T in Sparrow-V2 (cm -> m)
        self.state_upperbound[5] = 1.0 # Alpha will be normalized independently
        self.state_upperbound[6] = self.v_linear_max # V_linear
        self.state_upperbound[7] = self.v_angular_max# V_angular

        self.car_state_tc = torch.zeros(3) # (x,y,theta) in m; For state generation



    def _world_2_grid(self, location_wd):
        '''
        将Sparrow世界坐标(连续，单位为 m) 转化为 栅格坐标(离散，一格=1cm)
        输入 numpy； 格式：[x0,y0] or [[x0,y0],[x1,y1]...[xn,yn]]
        输出 numpy； 格式：[[x0,y0]] or [[x0,y0],[x1,y1]...[xn,yn]]
        '''
        p = 100*location_wd # m -> cm
        if len(p.shape)<2: p = p[np.newaxis,:] # [x0,y0] -> [[x0,y0]]
        return p.astype(int)

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
        # 记得延时，等待gazebo生效

    def _get_pose(self):
        """返回 x,y,theta
        https://github.com/XinJingHao/Sparrow-V2.1/blob/main/Images/coordinate_frames.svg"""
        while True:
            try:
                # 时间戳 rospy.Time(0)---选取时间间隔最近的两个坐标系之间的相对关系
                basefootprint_map = self.pose_buffer.lookup_transform("map", "fml_robot_0/base_footprint", rospy.Time(0))
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
                self.car_state_tc[0], self.car_state_tc[1], self.car_state_tc[2] = x,-y,theta # for state generation
                break

            except:
                print("Can not self._get_pose")

    def _fresh_obs(self, a):
        """self._obs = [current_a*2, real_a*2, D2T, α, Vl, Va, ld0, ..., ld17]"""

        '''Prepare state'''
        # Action state:
        self._obs[0],self._obs[1] = self.a_space[a] # current action from agent; m/s, rad/s
        if self.delayed_action_state:
            self._obs[2],self._obs[3] = self.a_space[self.previous_a] # real (delayed) action; m/s, rad/s
            self.previous_a = a
        else:
            self._obs[2],self._obs[3] = self.a_space[a]

        # D2T:
        self._get_pose() # fresh car_state_tc: (x:m, y:m, theta:rad)
        self.target_point = self.sub_guide.guidepoint # fresh target point
        xy_in_target = self.car_state_tc[0:2] - self.target_point # 小车在以target为原点的坐标系下的坐标, m
        self._obs[4] = torch.norm(xy_in_target, p=2) # Unnormalized D2T, m
        self.win = self._obs[4] < self.target_area

        # α:
        beta = torch.arctan(xy_in_target[0] / (xy_in_target[1]+1e-6)) + torch.pi / 2 + (xy_in_target[1] < 0) * torch.pi
        alpha = (beta - self.car_state_tc[2]) / torch.pi
        self._obs[5] = alpha + ( 2*(alpha < -1) - 2*(alpha > 1) ) # 修复1/2象限、3/4象限

        # V_linear, V_angular:
        self._obs[6:8] = self.sub_odom.odom_data # m/s, rad/s

        # LiDAR:
        ld_data = torch.clamp(self.sub_laser.laser_data, 0, self.ld_range)  # (0,3) m
        self._obs[8:] = ld_data
        self.collide = ld_data.min() < 0.13 #小车半径0.09, 雷达最低扫描范围0.11(精度0.01)



    def reset(self):
        self.cnt = 1 # step counter

        # 重置位置
        self._Set_object_pose(obj_name='fml_robot_0', x=self.startpose[0], y=self.startpose[1], z=0,theta_z=self.startpose[2])  # 重置被控小车位置
        self.rate.sleep()  # 延时，等待位置重置生效

        # 重置速度
        self.action.linear.x = 0
        self.action.angular.z = 0
        self.pub_vel.publish(self.action)
        self.previous_a = len(self.a_space) - 1 # stop

        # 重置历史轨迹
        self.history_traj = Path()
        self.history_traj.header.frame_id = self.main_frame

        # 刷新状态
        self._fresh_obs(self.previous_a)
        return self._obs/self.state_upperbound # Normalize


    def _pub_historic_trajectory(self) -> None:
        '''将robot当前时刻位置追加到history_traj中，然后发布history_traj'''
        pose = PoseStamped()
        pose.header.frame_id = self.main_frame  # Coordinate frame reference
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.car_state_tc[0].item()
        pose.pose.position.y = -self.car_state_tc[1].item()
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


    def step(self, a):
        '''与环境互动'''
        self.action.linear.x, self.action.angular.z = self.a_space[a] # m/s
        self.pub_vel.publish(self.action)
        self.rate.sleep()  # 延时，保证Temporal Window Queue中的时间间隔为0.1s(rostime)

        '''刷新状态'''
        self._fresh_obs(a) # 这里刷新的obs，其实不是a导致的，而是上一个a导致的（系统延迟）

        '''记录历史轨迹'''
        if self.cnt % 2 == 0: self._pub_historic_trajectory() # 每X步记录一次历史轨迹
        self.cnt += 1

        Normalized_state = self._obs/self.state_upperbound
        return Normalized_state, self.win, self.collide



"""--------------------------------------------------------------------------------------------------------"""

def str2bool(v):
    '''Fix the bool BUG for argparse: transfer string to bool'''
    if isinstance(v, bool): return v
    if v.lower() in ('yes', 'True','true','TRUE', 't', 'y', '1', 'T'): return True
    elif v.lower() in ('no', 'False','false','FALSE', 'f', 'n', '0', 'F'): return False
    else: print('Wrong Input Type!')

"""--------------------------------------------------------------------------------------------------------"""