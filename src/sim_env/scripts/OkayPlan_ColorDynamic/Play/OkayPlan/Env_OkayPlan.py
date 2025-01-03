import numpy as np
import torch
import rospy
import tf2_ros
from tf import transformations
import threading
import copy
import matplotlib.pyplot as plt
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Odom线程
class OdomThread(threading.Thread):
    def __init__(self, topic):
        super(OdomThread, self).__init__()
        self.topic = topic
    def _callback(self, msg):
        self.Vl = msg.twist.twist.linear.x # v_linear, m/s
        self.Va = msg.twist.twist.angular.z # v_angular, rad/s
        Rt = msg.pose.pose.orientation  # 四元数： x,y,z,w
        self.theta = transformations.euler_from_quaternion([Rt.x, Rt.y, Rt.z, Rt.w])[2] # 绕Z轴的旋转角度

    def run(self):
        rospy.Subscriber(self.topic, Odometry, self._callback, queue_size=1)
        print(f"----------Subscribing: {self.topic}----------")
        rospy.spin()


# Target线程: 获取Rviz的2D Nav Goal
class TargetThread(threading.Thread):
    def __init__(self, ):
        super(TargetThread, self).__init__()
        self.targetpoint = []

    def _callback(self, msg):
        self.targetpoint.append((msg.pose.position.x, msg.pose.position.y))

    def run(self):
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._callback, queue_size=1)
        print("----------Subscribing Rviz 2D Nav Goal----------")
        rospy.spin()

class Env_4_OkayPlan():
    '''2024.12.09'''
    def __init__(self, opt):
        self.dvc = opt.dvc  # 运算平台
        self.rate = rospy.Rate(10)
        self.main_frame = opt.main_frame

        '''Path Planer相关'''
        self.x_target, self.y_target = opt.startpoint # 初始化时，目标点与起始点重合
        # Kinematics Segments:
        self.Vl_scale = 1
        self.Va_scale = 0.5

        # 创建gazebo位置设置服务:
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        ''' Gazebo相关： '''
        # 障碍物:
        self.obs_list = ['Layer2_0', 'Layer2_1', 'Layer2_2', 'Layer2_3',
                         'Layer2_4', 'Layer2_5', 'Layer2_6', 'Layer2_7',
                         'Layer2_8', 'Layer2_9', 'Layer2_10', 'Layer2_11',
                         'B_1_3_0', 'B_1_3_1', 'B_1_3_2', 'B_1_3_3', 'B_1_3_4', 'B_1_3_5',
                         'B_1_4_0', 'B_1_4_1', 'B_1_4_2', 'B_1_4_3', 'B_1_4_4', 'B_1_4_5', ]
        # obs_init_pose (x,y):
        self.oip = [[4,4],[4,-4],[-4,4],[-4,-4],[0,9],[0,-9],[-9,0],[9,0],[8,8],[8,-8],[-8,8],[-8,-8],
                    [1.99632, 6.07045],[-2.0578, 6.06122],[-2.09356, 0.051228],[1.99973, 0.020044],[-2.08937, -5.9974],[1.94317, -5.96793],
                    [-5.4852, 0.015964],[-5.50992, -5.97184],[5.52612, 6.00951],[5.49347, -0.022705],[5.54818, -5.98559],[-5.44605, 6.01143]]
        self.O = len(self.obs_list) # number of Obstacles
        self.DO = 0  # number of Dynamic Obstacle
        for name in self.obs_list:
            if name[0:6] == 'Layer2': self.DO += 1

        self.CD = opt.CD # whether to detect AGV in bounding box
        self.Collide = False


        # 障碍物基座标系名称初始化:
        self.obs_TF_frame_list = copy.deepcopy(self.obs_list)
        for _ in range(self.DO): self.obs_TF_frame_list[_] += '/base_footprint'  # Layer2_X -> Layer2_X/base_footprint


        # 障碍物Bounding box初始化:
        self.D_FML_AGV = opt.D_FML_AGV # 真实飞马旅小车的直径
        self.dilation = self.D_FML_AGV / 2 + 0.2 # bounding box的膨胀, 0.2为噪声余量
        self.obs_outline = np.array([[self.D_FML_AGV, self.D_FML_AGV]]*self.DO + [[1, 3]]*6 + [[1, 4]]*6) # [width, height] of (AGV, B_1_3, B_1_4)
        self.bounding_box = self.obs_outline +self.dilation # 对障碍物轮廓进行膨胀，得到Bounding Box

        # 创建 话题:Odom 订阅对象:
        self.OdomList = []
        for i in range(self.DO):
            self.OdomList.append(OdomThread(self.obs_list[i]+'/odom'))
            self.OdomList[-1].start()
        # 用于存储障碍物的速度信息
        self.Obs_V = np.zeros((self.O, 1, 1, 2)) # O个障碍物的Vx,Vy

        # 创建 TF 订阅对象
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.tf_maxtry = 25 # 订阅的最大尝试次数

        # Start target thread, subscribing rviz 2D Nav Goal
        self.sub_target = TargetThread()
        self.sub_target.start()

        # 用于存储障碍物的bounding box数据
        self.Grouped_Obs_Segments_np = np.zeros((self.O,4,2,2)) # O个障碍物，4条边，2个端点，2维坐标


    def _Generate_BoundingBox(self, x: float, y: float, w: float, h: float, theta: float) -> np.ndarray:
        '''
        Input: coordinate of the obstacle's centre point (x,y), width(w), height(h), orientation(theta in radius)
        Output: coordinate of the Bounding Box in shape (4,2,2): 4条线段，2个端点，2个坐标
        '''

        theta_br, theta_tr, theta_tl = theta, theta + np.arctan(h / w), theta + np.pi / 2  # 这里应是h/w,老版本的w/h是错的
        diagonal = np.sqrt(w ** 2 + h ** 2)

        # calculate the non-shifted apexes:
        bl = np.array([x, y])  # bottom left: [x,y]
        br = np.array([x + w * np.cos(theta_br), (y + w * np.sin(theta_br))])  # bottom right:
        tr = np.array([x + diagonal * np.cos(theta_tr), (y + diagonal * np.sin(theta_tr))])  # top right
        tl = np.array([x + h * np.cos(theta_tl), (y + h * np.sin(theta_tl))])  # top left

        apexes = np.array([[bl, br], [br, tr], [tr, tl], [tl, bl]])  # (4,2,2): 4条线段，2个端点，2个坐标
        return apexes - (tr - bl) / 2  # shift the coordinates in centre manner


    def _Subscribe_TF(self) -> bool:
        '''订阅被控对象 和 障碍物 的TF变换信息, 并生成他们的bounding box。 成功返回True，不成功返回False'''
        try:
            # 被控对象:
            tfs = self.buffer.lookup_transform(self.main_frame, 'fml_robot_0/base_footprint', rospy.Time(0))
            Ts = tfs.transform.translation  # x,y,z
            self.x_start, self.y_start = Ts.x, Ts.y
            # distance from start to target:
            self.d2target = np.linalg.norm(np.array([self.x_start,self.y_start]) - np.array([self.x_target, self.y_target]))

            # 障碍物:
            for i, obs_name in enumerate(self.obs_TF_frame_list):
                tfs = self.buffer.lookup_transform(self.main_frame,obs_name,rospy.Time(0))
                Ts = tfs.transform.translation # x,y,z
                Rt = tfs.transform.rotation # 四元数： x,y,z,w
                euler_angles = transformations.euler_from_quaternion([Rt.x, Rt.y, Rt.z, Rt.w])
                self.Grouped_Obs_Segments_np[i] = self._Generate_BoundingBox(Ts.x, Ts.y, self.bounding_box[i,0], self.bounding_box[i,1], euler_angles[2]) #膨胀后的bounding box(大)
            return True
        except:
            return False

    def _Subscribe_odom(self):
        ''' 订阅odom,获得障碍物的速度信息,生成预测线段'''
        for i in range(self.DO):
            theta = self.OdomList[i].theta + self.Va_scale * self.OdomList[i].Va
            self.Obs_V[i, 0, 0, 0] = self.OdomList[i].Vl * np.cos(theta)
            self.Obs_V[i, 0, 0, 1] = self.OdomList[i].Vl * np.sin(theta)

        # 生成障碍物预测线段:
        pdct_Grouped_Obs_endpoints = self.Grouped_Obs_Segments_np + self.Vl_scale * self.Obs_V
        self.Grouped_pdct_segments_np = np.stack((self.Grouped_Obs_Segments_np[0:self.DO, :, 0, :],
                                                  pdct_Grouped_Obs_endpoints[0:self.DO, :, 0, :]),
                                                 axis=2)  # (dynamic_obs,4,2,2)

    def _isInsideObs(self, node: torch.tensor, Grouped_Obs_Segments: torch.tensor) -> bool:
        '''
        判断 node 是否在凸四边形的Grouped_Obs_Segments内部
        node = torch.tensor, shape=(2,)
        Grouped_Obs_Segments = torch.tensor, shape=(O,4,2,2)
        '''
        Obs_node_vector = node - Grouped_Obs_Segments[:,:,0,:]  # (N,4,2), 点和四边形顶点组成的向量
        Obs_edge_vector = Grouped_Obs_Segments[:,:,1,:] - Grouped_Obs_Segments[:,:,0,:]  # (N,4,2), 四边形的四条边向量
        cross_prodct = Obs_node_vector[:,:,0] * Obs_edge_vector[:,:,1] - Obs_node_vector[:,:,1] * Obs_edge_vector[:,:,0] #(O,4), 利用交叉积判断点在四边形四边的左侧还是右侧
        inside = (cross_prodct>0).all(dim=-1) + (cross_prodct<0).all(dim=-1) #(O,), 判断node在哪些Obs内 (node在哪些四边形四条边的同侧)
        return inside.any() #node 在任意一个Obs内？


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

    def fresh_targetpoint(self) -> bool:
        '''查看是否有新的targetpoint: 如果有则刷新，并且返回True; 否则返回False'''
        if self.sub_target.targetpoint:
            self.x_target, self.y_target = self.sub_target.targetpoint.pop()
            return True
        else:
            return False

    def get_envInfo(self) -> dict:
        '''抽取路径规划所需要的环境信息'''

        # 订阅 box/car 的TF信息
        for i in range(self.tf_maxtry+1):
            if self._Subscribe_TF(): break
            if i == self.tf_maxtry: print(f'Env.py Warning: Can not subscribe TF from Gazebo after {self.tf_maxtry} tries!!!')
        Grouped_Obs_Segments = torch.from_numpy(self.Grouped_Obs_Segments_np).to(self.dvc) # 用于碰撞检测
        if self.CD:
            self.Collide = self._isInsideObs(torch.tensor([self.x_start, self.y_start], device=self.dvc), Grouped_Obs_Segments)

        # 生成预测线
        self._Subscribe_odom()
        Flat_pdct_segments = torch.from_numpy(self.Grouped_pdct_segments_np).reshape(self.DO * 4, 2, 2).to(self.dvc)

        return dict(start_point = (self.x_start, self.y_start),
                    target_point = (self.x_target, self.y_target),
                    d2target = self.d2target,
                    Obs_Segments = Grouped_Obs_Segments.reshape((self.O*4,2,2)), # 障碍物bounding box线段，(4*O,2,2)
                    Flat_pdct_segments = Flat_pdct_segments, # 障碍物运动信息线段， (dynamic_obs*4,2,2)
                    Arrive = (self.d2target < 2*self.D_FML_AGV), # 是否到达终点，
                    Collide = self.Collide) # AGV与Grouped_Obs_Segments是否发生碰撞

    def reset(self, startpoint: tuple) -> dict:
        for i, name in enumerate(self.obs_list): # 重置L1, L2障碍物的位置
            self._Set_object_pose(obj_name=name, x=self.oip[i][0], y=self.oip[i][1], z = (0 if i <4 else 0.5),
                                  theta_z= (np.pi*np.random.rand() if i <4 else None)) # 随机L2朝向
        self._Set_object_pose(obj_name='fml_robot_0', x=startpoint[0], y=startpoint[1], z=0) # 重置被控小车位置
        self.rate.sleep() # 延时，等待位置重置生效

        return self.get_envInfo()


def str2bool(v):
    '''Fix the bool BUG for argparse: transfer string to bool'''
    if isinstance(v, bool): return v
    if v.lower() in ('yes', 'True','true','TRUE', 't', 'y', '1', 'T'): return True
    elif v.lower() in ('no', 'False','false','FALSE', 'f', 'n', '0', 'F'): return False
    else: print('Wrong Input Type!')