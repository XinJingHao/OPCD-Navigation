import numpy as np
import rospy
import torch
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
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
        # self.laser_data = ld_range * torch.ones(self.ld_g_num)

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

# Target线程: 获取Rviz的2D Nav Goal
class TargetThread(threading.Thread):
    def __init__(self, ):
        super(TargetThread, self).__init__()
        self.target_xy = deque()

    def _callback(self, msg):
        xy = np.array([100*msg.pose.position.x, -100*msg.pose.position.y])
        self.target_xy.append(xy)

    def run(self):
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._callback, queue_size=1)
        print("----------Subscribing Rviz 2D Nav Goal----------")
        rospy.spin()



# 19 editable configurations:
default_cfg = dict(
    py_render = True, # whether to use py_render
    D=400, # maximal local planning distance
    ld_range=300, # max scanning distance; cm
    ld_num=240, # number of lidar streams in each world
    ld_GN=10) # how many lidar streams are grouped for each group


class Gazebo_fml_robot_env():
    def __init__(self, **params):
        '''Robot should orient to the X-axis when init!!!'''
        if len(params)==0: self.__dict__.update(**default_cfg) # Use default configration
        else: self.__dict__.update(params) # Use user's configration
        self.ld_range /= 100  # cm -> m
        self.ld_g_num = int(self.ld_num / self.ld_GN) # state中的雷达线数

        self.car_radius = 9 # cm
        self.window_size = 1000 # cm
        self.state_dim = 8 + self.ld_g_num # current_a*2, real_a*2, D2T, α, Vl, Va, ld0, ..., ld23
        self._obs = torch.zeros(self.state_dim)
        self.action_dim = 7 # no stop for agent
        self.delayed_action_state = False # whether show delayed action to the agent

        # Waypoints publisher
        self.pub_wp = rospy.Publisher('/waypoints', MarkerArray, queue_size=1)

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

        # Start target thread, subscribing rviz 2D Nav Goal
        self.sub_target = TargetThread()
        self.sub_target.start()
        rospy.sleep(1)


        # Navigation:
        self.target_area = 0.3  # m; Enter the central circle (radius=target_area) will be considered win.
        self.waypoints = deque() # cm, pygame coordinate, ndarray
        self.global_path = deque() # cm, pygame coordinate, ndarray


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

        self.car_state_np = np.zeros(3)  # (x,y,theta) in m; For render
        self.car_state_tc = torch.zeros(3) # (x,y,theta) in m; For state generation

        # render init:
        if self.py_render:
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size , self.window_size ))
            self.clock = pygame.time.Clock()
            self.canvas = pygame.Surface((self.window_size , self.window_size ))
            self.ld_angle_interval = np.arange(self.ld_g_num) * 2 * np.pi / (self.ld_g_num-1)  - np.pi #减pi是因为雷达和小车朝向差了pi
            self.grid_x = 100*np.arange(self.window_size/100) # gd
            self.grid_y = 100*np.arange(self.window_size/100) # gd



    def _world_2_grid(self, location_wd):
        '''
        将Sparrow世界坐标(连续，单位为 m) 转化为 栅格坐标(离散，一格=1cm)
        输入 numpy； 格式：[x0,y0] or [[x0,y0],[x1,y1]...[xn,yn]]
        输出 numpy； 格式：[[x0,y0]] or [[x0,y0],[x1,y1]...[xn,yn]]
        '''
        p = 100*location_wd # m -> cm
        if len(p.shape)<2: p = p[np.newaxis,:] # [x0,y0] -> [[x0,y0]]
        return p.astype(int)


    def _get_pose(self):
        """返回结果x,y,theta: https://github.com/XinJingHao/Sparrow-V2.1/blob/main/Images/coordinate_frames.svg"""
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
                self.car_state_np[0], self.car_state_np[1], self.car_state_np[2] = x,-y,theta # for render
                self.car_state_tc[0], self.car_state_tc[1], self.car_state_tc[2] = x,-y,theta # for state generation
                break

            except:
                print("Can not self._get_pose")

    def _fresh_obs(self, a, reset=False):
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
        if reset: self.target_point = self.car_state_tc[0:2] # use current xy as target point when reset; in pygame coordinates; m
        xy_in_target = self.car_state_tc[0:2] - self.target_point # 小车在以target为原点的坐标系下的坐标, m
        self._obs[4] = torch.norm(xy_in_target, p=2) # Unnormalized D2T, m

        # α:
        beta = torch.arctan(xy_in_target[0] / (xy_in_target[1]+1e-6)) + torch.pi / 2 + (xy_in_target[1] < 0) * torch.pi
        alpha = (beta - self.car_state_tc[2]) / torch.pi
        self._obs[5] = alpha + ( 2*(alpha < -1) - 2*(alpha > 1) ) # 修复1/2象限、3/4象限

        # V_linear, V_angular:
        self._obs[6:8] = self.sub_odom.odom_data # m/s, rad/s

        # LiDAR:
        ld_data = torch.clamp(self.sub_laser.laser_data, 0, self.ld_range)  # (0,3) m
        self._obs[8:] = ld_data

        # 在rviz中使用2D Nav Goal设置局部导航点:
        if self.sub_target.target_xy:
            xy = self.sub_target.target_xy.popleft()
            self._add_waypoints(xy)

        # pygame render
        if self.py_render:
            self.ld_data_np = ld_data.numpy()
            self.ld_angle = self.ld_angle_interval + self.car_state_np[2]
            self.ld_vectors_wd = np.stack((np.cos(self.ld_angle),-np.sin(self.ld_angle)),axis=1)
            self._render_frame()


    def reset(self):
        self.action.linear.x = 0
        self.action.angular.z = 0
        self.pub_vel.publish(self.action)
        self.previous_a = len(self.a_space) - 1 # stop

        self._fresh_obs(self.previous_a, reset=True)
        self.win = self._obs[4] < self.target_area

        self.global_path.clear()
        self.waypoints.clear()
        self.global_path.append(100*self.target_point.numpy())
        self.waypoints.append(100*self.target_point.numpy())
        self._pub_waypoints()

        return self._obs/self.state_upperbound # Normalize


    def step(self, a):
        self.action.linear.x, self.action.angular.z = self.a_space[a] # m/s
        self.pub_vel.publish(self.action)

        self._fresh_obs(a) # 这里刷新的obs，其实不是a导致的，而是上一个a导致的（系统延迟）
        self.win = self._obs[4] < self.target_area

        # change target point
        if self.win and len(self.waypoints) > 1:
            self.waypoints.popleft()
            self._pub_waypoints()
            self.target_point = torch.tensor(self.waypoints[0])/100 # m

        return self._obs/self.state_upperbound # Normalize

    def _add_waypoints(self, xy):
        distance = np.linalg.norm(xy - self.waypoints[-1])
        if distance > self.D:  # cm
            print(f"Local Planning Distance exceeds maximal threshold: {self.D}cm")
        else:
            self.global_path.append(xy)
            self.waypoints.append(xy)
            self._pub_waypoints()

    def _pub_waypoints(self):
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"  # 设定坐标系，通常为"map"或"base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i  # 每个点的唯一 ID
            marker.type = Marker.SPHERE  # 设置标记类型为球体
            marker.action = Marker.ADD

            # 设置标记的位置
            marker.pose.position.x = 0.01*waypoint[0] # m -> cm
            marker.pose.position.y = -0.01*waypoint[1] # pygame.coordinate -> ros.coordinate
            marker.pose.orientation.w = 1.0

            # 设置标记的大小和颜色
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            marker.color.r, marker.color.g, marker.color.b = (0.95,0.08,0.52)
            marker.color.a = 1.0  # 设置为不透明

            # 添加到 MarkerArray
            marker_array.markers.append(marker)

        # 发布 MarkerArray
        self.pub_wp.publish(marker_array)


    def _render_frame(self):
        self.canvas.fill((255,255,255))

        # Plot FML grid
        for x in self.grid_x:
            pygame.draw.aaline(self.canvas, (16, 16, 16),(x,0),(x,self.window_size))
        for y in self.grid_y:
            pygame.draw.aaline(self.canvas, (16, 16, 16),(0,y),(self.window_size,y))

        car_center = self._world_2_grid(self.car_state_np[0:2]) # torch.tensor in (1,2)
        # 画雷达射线
        ld_real_sta_gd = car_center
        ld_real_end_gd = self._world_2_grid(self.car_state_np[0:2] + self.ld_data_np[:,np.newaxis] * self.ld_vectors_wd)
        for i in range(self.ld_g_num):
            e = 255*self.ld_data_np[i]/self.ld_range
            pygame.draw.aaline(self.canvas, (255-e, 0, e), ld_real_sta_gd[0], ld_real_end_gd[i])

        # 小车碰撞阈值
        pygame.draw.circle(
            self.canvas,
            (64,64,64),
            car_center[0],
            self.car_radius + 5)

        # 小车车体
        pygame.draw.circle(
            self.canvas,
            (200, 128, 250),
            car_center[0],
            self.car_radius)

        # 小车朝向
        head = self.car_state_np[0:2] + self.car_radius/100 * np.array([np.cos(self.car_state_np[2]), -np.sin(self.car_state_np[2])])
        pygame.draw.line(
            self.canvas,
            (0, 255, 255),
            car_center[0],
            self._world_2_grid(head)[0],
            width=2)

        # draw global path
        if self.global_path:
            for i in range(len(self.global_path)-1):
                pygame.draw.line(self.canvas, (255,200,15), self.global_path[i],self.global_path[i+1], width=4)

        # draw target point
        if self.waypoints:
            pygame.draw.circle(self.canvas,(100, 225, 100),self.waypoints[0],100*self.target_area,2)

        # draw waypoints
        for i in range(1, len(self.waypoints)):
            pygame.draw.circle(self.canvas,(255,200,15),self.waypoints[i],100*self.target_area,2)




        # The following line copies our drawings from `canvas` to the visible window
        self.window.blit(self.canvas, self.canvas.get_rect())
        pygame.event.pump()
        pygame.display.update()


        # 在pygame中使用鼠标点击来增加waypoints(需要开启pygame render):
        ev = pygame.event.get()
        for event in ev:
            if event.type == pygame.MOUSEBUTTONDOWN:
                xy = np.array(pygame.mouse.get_pos())
                self._add_waypoints(xy)


"""--------------------------------------------------------------------------------------------------------"""

def str2bool(v):
    '''Fix the bool BUG for argparse: transfer string to bool'''
    if isinstance(v, bool): return v
    if v.lower() in ('yes', 'True','true','TRUE', 't', 'y', '1', 'T'): return True
    elif v.lower() in ('no', 'False','false','FALSE', 'f', 'n', '0', 'F'): return False
    else: print('Wrong Input Type!')

"""--------------------------------------------------------------------------------------------------------"""