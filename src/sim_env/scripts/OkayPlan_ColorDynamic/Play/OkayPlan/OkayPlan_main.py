import argparse
import rospy
import torch
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
from Planner import OkayPlan
from Env_OkayPlan import Env_4_OkayPlan, str2bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np


'''Configurations:'''
parser = argparse.ArgumentParser()
parser.add_argument('--dvc', type=str, default='cuda', help='Running device of SEPSO_Down, cuda or cpu')
parser.add_argument('--DPI', type=str2bool, default=True, help='True for DPI(from OkayPlan), False for PI(from SEPSO)')
parser.add_argument('--KP', type=str2bool, default=True, help='whether to use Kinematics_Penalty')

# Planner related:
parser.add_argument('--Max_iterations', type=int, default=100, help='maximum number of particle iterations for each planning')
parser.add_argument('--N', type=int, default=1000, help='number of particles in each group')
parser.add_argument('--D', type=int, default=20, help='particle dimension: number of waypoints = D/2')
parser.add_argument('--Quality', type=float, default=0.1, help='planning quality: the smaller, the better quality, and the longer time')
parser.add_argument('--ShakePenalty', type=float, default=0.3, help='Penalty for path shaking, recommend in [0,1]')
parser.add_argument('--compile', type=str2bool, default=True, help='whether to use torch.compile to boost calculation speed')

# Env related:
parser.add_argument('--startpoint', type=tuple, default=(0, 0), help='start point')
parser.add_argument('--map_size', type=int, default=20, help='map size: m')
parser.add_argument('--main_frame', type=str, default='map', help='the main frame of navigation, eg: gazebo_center/map')
parser.add_argument('--D_FML_AGV', type=float, default=0.2, help='diameter of the real FML agv: m')
parser.add_argument('--CD', type=str2bool, default=False, help='Collision Detection: whether to detect AGV in bounding box')

opt = parser.parse_args()
opt.dvc = torch.device(opt.dvc)
opt.NP = int(opt.D / 2)  # 每个粒子所代表的路径的端点数量
opt.Search_range = (-opt.map_size/2, opt.map_size/2)


if __name__ == '__main__':
    # 初始化 ROS 节点
    rospy.init_node("OkayPlan_main")
    rate = rospy.Rate(10)

    # 创建路径发布器
    path_puber = rospy.Publisher("OkayPlan", Path, queue_size=1) # topic name: /OkayPlan
    guide_puber = rospy.Publisher("GuidePoint", Marker, queue_size=1) # topic name: /GuidePoint

    # 加载OkayPlan的超参数
    params = torch.load('Relax0.4_S0_ 2023-09-23 21_38.pt', map_location=opt.dvc)[-1]  # [-1] means the final parameters
    if not opt.KP: params[50] = 0

    # 实例化环境和规划器
    env = Env_4_OkayPlan(opt) # env用于抽取gazebo的环境信息和移动小车
    planner = OkayPlan(opt, params)  # 用于规划全局路径

    # 环境初始化
    env_info = env.reset(opt.startpoint)
    planner.Priori_Path_Init(env_info['start_point'], env_info['target_point'])  # 为第一次规划初始化先验路径

    while not rospy.is_shutdown():
        '''e-greedy随机初始化'''
        if np.random.rand() < 0.001:
            planner.Priori_Path_Init(env_info['start_point'], env_info['target_point'])
            print('OkayPlan: e-greedy Init.')

        '''根据环境信息规划Global Path'''
        path_torch, collision_free = planner.plan(env_info) # collision_free仅代表路径是否无碰撞，并不代表start_point的碰撞情况
        path_np = path_torch.cpu().numpy() # torch to numpy, [x0,x1,...,y0,y1,..], shape=(D,)
        path_np = path_np.reshape(2, -1).T # [[x0,y0], [x1,y1], [x2,y2] ...], shape=(NP,2)

        '''根据Global Path计算局部GuidePoint'''
        guidepoint = planner.get_guidepoint(path_np)

        '''组织&发布路径数据：'''
        Path_msg = Path()  # 创建数据
        # 填写表头：
        current_time = rospy.Time.now()  # Current timestamp
        Path_msg.header.stamp = current_time
        Path_msg.header.frame_id = opt.main_frame
        # 填写路径坐标：
        for i in range(opt.NP):
            pose = PoseStamped()
            pose.header.frame_id = opt.main_frame  # Coordinate frame reference
            pose.header.stamp = current_time
            pose.pose.position.x = path_np[i,0]
            pose.pose.position.y = path_np[i,1]
            Path_msg.poses.append(pose)
        path_puber.publish(Path_msg) # 发布路径

        '''组织&发布GuidePoint'''
        marker = Marker()
        marker.header.frame_id = "map"  # 设定坐标系，通常为"map"或"base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "guidepoint"
        marker.id = 0
        marker.type = Marker.SPHERE  # 设置标记类型为球体
        marker.action = Marker.ADD

        # 设置标记的位置
        marker.pose.position.x = guidepoint[0]
        marker.pose.position.y = guidepoint[1]
        marker.pose.orientation.w = 1.0

        # 设置标记的大小和颜色
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r, marker.color.g, marker.color.b = (0.95, 0.08, 0.52)
        marker.color.a = 1.0  # 设置为不透明
        guide_puber.publish(marker)

        '''刷新'''
        if env.fresh_targetpoint():
            # 刷新环境信息
            env_info = env.get_envInfo()
            planner.Priori_Path_Init(env_info['start_point'], env_info['target_point'])  # 为第一次规划初始化先验路径
            print('OkayPlan: Uniform interpolation for first plan.')
        else:
            env_info = env.get_envInfo()

        # rate.sleep()



















