from collections import deque
import numpy as np
import torch

class OkayPlan():
    '''
    OkayPlan: A real-time path planner for dynamic environment
    Author: Jinghao Xin, Jinwoo Kim, Shengjia Chu, and Ning Li

    Paper Web: https://www.sciencedirect.com/science/article/pii/S002980182401179X
    Code Web: https://github.com/XinJingHao/OkayPlan

    Cite this algorithm:
    @article{XinOkayPlan,
    title = {OkayPlan: Obstacle Kinematics Augmented Dynamic real-time path Planning via particle swarm optimization},
    journal = {Ocean Engineering},
    volume = {303},
    pages = {117841},
    year = {2024},
    issn = {0029-8018},
    doi = {https://doi.org/10.1016/j.oceaneng.2024.117841},
    url = {https://www.sciencedirect.com/science/article/pii/S002980182401179X},
    author = {Jinghao Xin and Jinwoo Kim and Shengjia Chu and Ning Li}}

    Only for non-commercial purposes
    All rights reserved
    '''
    def __init__(self, opt, params):
        self.dvc = opt.dvc  # 运算平台

        '''Hyperparameter Initialization'''
        self.params = params # 该组参数的Group Number=8
        self.G = 8 # Group数量应该严格等于生成params时的种群数量
        # Inertia Initialization for 8 Groups:
        self.w_init = self.params[0:self.G].unsqueeze(-1).unsqueeze(-1)
        self.w_end = (self.params[0:self.G] * self.params[self.G:(2 * self.G)]).unsqueeze(-1).unsqueeze(-1)
        self.Max_iterations = opt.Max_iterations  # 每帧的最大迭代次数
        self.w_delta = (self.w_init - self.w_end) / self.Max_iterations  # (G,1,1)
        # Velocity Initialization for 8 Groups:
        self.v_limit_ratio = self.params[(2 * self.G):(3 * self.G)].unsqueeze(-1).unsqueeze(-1)  # (G,1,1)
        self.v_init_ratio = 0.7 * self.v_limit_ratio  # (G,1,1)
        # H Matrix, (4,G=8,1,1):
        self.Hyper = torch.ones((4, self.G), device=self.dvc)
        self.Hyper[1] = self.params[(3 * self.G):(4 * self.G)]
        self.Hyper[2] = self.params[(4 * self.G):(5 * self.G)]
        self.Hyper[3] = self.params[(5 * self.G):(6 * self.G)]
        self.Hyper.unsqueeze_(-1).unsqueeze_(-1)

        '''Particle Related: 注意本代码的搜索空间单位为m'''
        self.N, self.D = opt.N, opt.D # number of Groups, particles in goups, and particle dimension
        self.arange_idx = torch.arange(self.G, device=self.dvc)  # 索引常量
        self.Search_range = opt.Search_range  # search space of the Particles
        self.Random = torch.zeros((4,self.G,self.N, 1), device=self.dvc)
        self.Kinmtc = torch.zeros((4, self.G, self.N, self.D), device=self.dvc)  # [V,Pbest,Gbest,Tbest]
        self.Locate = torch.zeros((4, self.G, self.N, self.D), device=self.dvc) # [0,X,X,X]
        self.V_offset = 20*opt.map_size/366 # 用于设置粒子速度 (20是针对366的map而言的,这里需要scale一下)

        '''Path Related'''
        self.NP = opt.NP # 每个粒子所代表的路径的端点数量
        self.S = self.NP-1 # 每个粒子所代表的路径的线段数量
        self.P = self.G*self.N*self.S # 所有粒子所代表的路径的线段总数量
        self.rd_area = 0.5 * (self.Search_range[1] - self.Search_range[0]) / (self.S)  # SEPSO中先验知识初始化粒子时的随机范围
        self.ShakePenalty = opt.ShakePenalty

        '''Auto Truncation'''
        # For info, check https://arxiv.org/abs/2308.10169
        self.AT = True
        self.TrucWindow = 20
        self.std_Trsd = opt.Quality  # 自动截断判据中,std的阈值: 越小,每次规划耗时越多,但结果更好更稳定. 应在实时性和规划质量间折衷(原论文中取的10)

        '''Dynamic Prioritized Initialization'''
        self.DPI = opt.DPI

        '''GuidePoint Initialization'''
        self.angles_trsd = 0.2 # 转折角阈值, rad
        self.discounted_D = 0.6*4 # 局部路径规划阈值(这里4为Sparrow训练时的D), m
        self.length_trsd = 0.5 # 选择targetpoint作为guidepoint的阈值, m
        self.waypoints_trsd = 0.2 # waypoints过滤的阈值(删除距离太近的waypoint)

        '''Compile: slow down the first planning, but boost the following planning'''
        if opt.compile:
            self._get_Obs_Penalty = torch.compile(self._get_Obs_Penalty)
            self._get_Shake_Penalty = torch.compile(self._get_Shake_Penalty)
        else:
            print("When use OkayPlan, you can set 'compile=True' to boost the planning speed.")

    def _uniform_random(self, low, high, shape):
        '''Generate uniformly random number in [low, high) in 'shape' on self.dvc'''
        return (high - low)*torch.rand(shape, device=self.dvc) + low

    def _Uniform_interpolation_path(self, start, target):
        '''在起点和终点之间均匀插值形成路径'''
        Path_Xs = (start[0] + (target[0] - start[0]) * torch.arange(self.NP, device=self.dvc) / (self.NP - 1))
        Path_Ys = (start[1] + (target[1] - start[1]) * torch.arange(self.NP, device=self.dvc) / (self.NP - 1))
        return torch.cat((Path_Xs, Path_Ys))

    def Priori_Path_Init(self, start, target):
        '''在起点和终点之间均匀插值生成初始先验路径，每次导航前需要执行一次，导航中无需执行'''
        self.Priori_Path = self._Uniform_interpolation_path(start, target)
        self.shake_cefi = 0 # 最初的规划不考虑shape penalty

    def _fix_apex(self):
        '''固定路径的首末端点, 注意x_start,y_start,x_target,y_target都是标量'''
        self.Locate[1:4, :, :, 0] = self.x_start
        self.Locate[1:4, :, :, self.NP] = self.y_start
        self.Locate[1:4, :, :, self.NP - 1] = self.x_target
        self.Locate[1:4, :, :, 2 * self.NP - 1] = self.y_target

    def _ReLocate(self):
        '''初始化粒子群动力学特征，每次iterate前要执行一次'''
        # Dynamic Prioritized Initialization // Prioritized Initialization
        if self.DPI:
            # OkayPlan Position Init: 根据先验知识Priori_X(上次迭代的Tbest)来初始化粒子群
            Mid_points = torch.tensor([[(self.x_start+self.x_target)/2],[(self.y_start+self.y_target)/2]],device=self.dvc).repeat((1,self.NP)).reshape(self.D) #(D,)
            self.Locate[1:4] = Mid_points + self._uniform_random(low=-self.d2target/2, high=self.d2target/2, shape=(self.G,self.N,self.D)) # DPI区域内随机初始化
            # self.Locate[1:4, :, 0:self.params[53].int()] = Priori_X  # (3,G,RN,D), V0
            self.Locate[1:4, :, 0] = self.Priori_Path  # (3,G,0,D), V1, replace params[53] for 1
            self.Locate[1:4, :, 1] = self._Uniform_interpolation_path((self.x_start,self.y_start), (self.x_target,self.y_target)) # V2, 同时每回和加入均匀插值路径，进一步提供先验信息
        else:
            # SEPSO Position Init: From https://github.com/XinJingHao/Real-time-Path-planning-with-SEPSO
            RN = int(0.25*self.N)
            self.Locate[1:4] = self._uniform_random(low=self.Search_range[0], high=self.Search_range[1], shape=(self.G,self.N,self.D)) #[0,X,X,X]
            self.Locate[1:4, :, 0:RN] = self.Priori_Path + self._uniform_random(-self.rd_area, self.rd_area, (RN,self.D)) # (3,G,RN,D)

        # 限制粒子位置至合法范围
        self.Locate[1:4].clip_(self.Search_range[0], self.Search_range[1])
        self._fix_apex()

        # Velocity Init: (+V_offset防止后期d2target太小时，速度太小)
        self.Kinmtc[0] = self._uniform_random(low=-(self.d2target+self.V_offset),
                                              high=(self.d2target+self.V_offset),
                                              shape=(self.G,self.N,self.D)) * self.v_init_ratio

        # Best Value Init:
        self.Pbest_value = torch.ones((self.G,self.N), device=self.dvc) * torch.inf
        self.Gbest_value = torch.ones(self.G, device=self.dvc) * torch.inf
        self.Tbest_value = torch.inf

        # Auto-Truncation Mechanism
        self.Tbest_Collision_Free = False
        self.Tbest_values_deque = deque(maxlen=self.TrucWindow)

        # Adaptive Velocity range
        self.v_min = -(self.v_limit_ratio * (self.d2target+self.V_offset)) # (G,1,1), (+V_offset防止后期d2target太小时，速度太小)
        self.v_max = (self.v_limit_ratio * (self.d2target+self.V_offset)) # (G,1,1), (+V_offset防止后期d2target太小时，速度太小)

    def _Cross_product_for_VectorSet(self, V_PM, V_PP):
        '''计算 向量集V_PM 和  向量集V_PP 的交叉积 (x0*y1-x1*y0)
            V_PM = torch.tensor((p, m, 2, 2))
            V_PP = torch.tensor((p,2))
            Output = torch.tensor((p, m, 2))'''
        return V_PM[:, :, :, 0] * V_PP[:, 1, None, None] - V_PM[:, :, :, 1] * V_PP[:, 0, None, None]

    def _Is_Seg_Ingersection_PtoM(self, P, M):
        '''利用[交叉积-跨立实验]判断 线段集P 与 线段集M 的相交情况
            P = torch.tensor((p,2,2))
            M = torch.tensor((m,2,2))
            Output = torch.tensor((p,m)), dtype=torch.bool'''

        V_PP = P[:, 1] - P[:, 0]  # (p, 2),自身向量
        V_PM = M - P[:, None, None, 0]  # (p, m, 2, 2), 自身起点与其他线段端点构成的向量
        Flag1 = self._Cross_product_for_VectorSet(V_PM, V_PP).prod(dim=-1) < 0  # (p, m)

        V_MM = M[:, 1] - M[:, 0]  # (m, 2)
        V_MP = P - M[:, None, None, 0]  # (m, p, 2, 2)
        Flag2 = self._Cross_product_for_VectorSet(V_MP, V_MM).prod(dim=-1) < 0  # (m, p)
        return Flag1 * Flag2.T

    def _get_Obs_Penalty(self):
        # 将粒子群转化为线段，并展平为(G*N*S,2,2)
        particles = self.Locate[1].clone()  # (G,N,D)
        start_points = torch.stack((particles[:,:,0:(self.NP-1)], particles[:,:,self.NP:(2*self.NP-1)]), dim=-1) #(G,N,S,2), S条线段的起点坐标
        end_points = torch.stack((particles[:,:,1:self.NP], particles[:,:,(self.NP+1):2*self.NP]), dim=-1) #(G,N,S,2), S条线段的终点坐标
        Segments = torch.stack((start_points, end_points),dim=-2) #(G,N,S,2,2), (G,N,S)条线段的端点坐标
        flatted_Segments = Segments.reshape((self.P,2,2)) # (G*N*S,2,2), 将所有粒子展平为G*N*S条线段

        # 将展平后的粒子群线段 与 地图中的障碍物边界线段 进行跨立实验，得到交叉矩阵
        Intersect_Matrix = self._Is_Seg_Ingersection_PtoM(flatted_Segments, self.Obs_Segments) # (P,M)
        Current_Obs_penalty = Intersect_Matrix.sum(dim=-1).reshape((self.G,self.N,self.S)).sum(dim=-1) #(G,N)

        # 将展平后的粒子群线段 与 障碍物运动轨迹线段 进行跨立实验，得到交叉矩阵
        Pdct_Intersect_Matrix = self._Is_Seg_Ingersection_PtoM(flatted_Segments, self.Flat_pdct_segments) # (P,M)
        Pdct_Obs_penalty = Pdct_Intersect_Matrix.sum(dim=-1).reshape((self.G, self.N, self.S)).sum(dim=-1)  # (G,N)
        return Current_Obs_penalty, Pdct_Obs_penalty

    def _get_Shake_Penalty(self):
        '''t时刻各条路径和t-1最好路径的差异'''
        shake_penalty = torch.linalg.norm(self.Locate[1] - self.Priori_Path, dim=-1)
        return shake_penalty  # (G,N)

    def _get_fitness(self):
        Segments_lenth = torch.sqrt((self.Locate[1,:,:,0:(self.NP-1)] - self.Locate[1,:,:,1:self.NP]).pow(2) +
                             (self.Locate[1,:,:,self.NP:(2*self.NP-1)] - self.Locate[1,:,:,(self.NP+1):(2*self.NP)]).pow(2)) # (G,N,S)
        Path_lenth = Segments_lenth.sum(dim=-1) # (G,N) # 总路径长度

        if self.ShakePenalty > 0:
            shake_penalty = self._get_Shake_Penalty()  # 防止路径抖动; (G,N)
            self.shake_cefi = min(self.ShakePenalty, self.shake_cefi + 0.01)  # 随着规划的进行，逐步增加抖动惩罚: 探索->利用
            shake_penalty = (self.shake_cefi*shake_penalty).pow_(2) # 有助于抑制大抖动(L1)，鼓励小抖动(L2)
        else:
            shake_penalty = 0

        self.Obs_penalty, Pdct_Obs_penalty = self._get_Obs_Penalty()  # both in (G,N)

        # Fitness = Length_Term + shake_penalty + Collision_Penalty + Kinematics_Penalty
        # 用d2target实现Dynamic Normalization,保证不同地图尺寸时, 惩罚相都与目标相的幅值匹配
        return Path_lenth + shake_penalty + \
            self.params[48] * self.d2target * (self.Obs_penalty).pow_(self.params[49].long()) + \
            self.params[50] * self.d2target * (Pdct_Obs_penalty).pow_(self.params[51].long())

    def _iterate(self):
        ''' 粒子迭代, 规划路径( 规划结果=self.Kinmtc[3,0,0] ) '''

        ''' Step 0: 动态先验初始化'''
        self._ReLocate()

        for i in range(self.Max_iterations):
            # if i == (self.Max_iterations - 1): print('Reach maximal iteration!!')
            '''Auto Truncation Mechanism'''
            if self.AT and (i>0.2*self.TrucWindow) and self.Tbest_Collision_Free and (np.std(self.Tbest_values_deque)<self.std_Trsd):
                # if (i>0.2*self.TrucWindow): print(f'迭代次数:{i}, std:{np.std(self.Tbest_values_deque)}')
                break

            ''' Step 1: 计算Fitness (G,N)'''
            fitness = self._get_fitness()

            ''' Step 2: 更新Pbest, Gbest, Tbest 的 value 和 particle '''
            # Pbest
            P_replace = (fitness < self.Pbest_value) # (G,N)
            self.Pbest_value[P_replace] = fitness[P_replace] # 更新Pbest_value
            self.Kinmtc[1, P_replace] = self.Locate[1, P_replace] # 更新Pbest_particles

            # Gbest
            values, indices = fitness.min(dim=-1)
            G_replace = (values < self.Gbest_value) # (G,)
            self.Gbest_value[G_replace] = values[G_replace] # 更新Gbest_value
            self.Kinmtc[2, G_replace] = (self.Locate[2, self.arange_idx, indices][G_replace].unsqueeze(1))

            # Tbest
            flat_idx = fitness.argmin()
            idx_g, idx_n = flat_idx//self.N, flat_idx%self.N
            min_fitness = fitness[idx_g, idx_n]
            if min_fitness < self.Tbest_value:
                self.Tbest_value = min_fitness # 更新Tbest_value
                self.Kinmtc[3] = (self.Locate[3, idx_g, idx_n]).clone() #这里必须clone, 否则数据联动
                # 查看Tbest所代表的路径是否collision free:
                if self.Obs_penalty[idx_g, idx_n] == 0: self.Tbest_Collision_Free = True
                else: self.Tbest_Collision_Free = False
            self.Tbest_values_deque.append(self.Tbest_value.item())

            ''' Step 3: 更新粒子速度 '''
            self.Hyper[0] = self.w_init - self.w_delta*i  # 惯性因子衰减
            self.Random[1:4] = torch.rand((3,self.G,self.N,1), device=self.dvc) #装载随机数
            self.Kinmtc[0] = (self.Hyper * self.Random * (self.Kinmtc - self.Locate)).sum(dim=0) #(G,N,D)
            self.Kinmtc[0].clip_(self.v_min, self.v_max) # 限制速度范围

            ''' Step 4: 更新粒子位置 '''
            self.Locate[1:4] += self.Kinmtc[0] # (3,G,N,D) + (G,N,D) = (3,G,N,D)
            self.Locate[1:4].clip_(self.Search_range[0], self.Search_range[1]) # 限制位置范围
            self._fix_apex()


    def plan(self, env_info):
        ''' 获取环境信息 -> 迭代求解路径 -> 更新下一时刻的先验路径  -> 返回当前时刻路径'''

        self.x_start, self.y_start = env_info['start_point']
        self.x_target, self.y_target = env_info['target_point']
        self.d2target = env_info['d2target']
        self.Obs_Segments = env_info['Obs_Segments'] # (4*O,2,2)
        self.Flat_pdct_segments = env_info['Flat_pdct_segments'] # (dynamic_obs*4,2,2)

        self._iterate() # 优化问题迭代求解
        self.Priori_Path = self.Kinmtc[3,0,0].clone() # 每次规划完成后，继承当前的结果，用于下次DPI

        return self.Kinmtc[3,0,0].clone(), self.Tbest_Collision_Free

    def get_guidepoint(self, waypoints: np.ndarray) -> np.ndarray:
        '''
        功能: 根据waypoints计算Local Planner的局部引导点
        输入: N个waypoints的2维坐标, (N,2)
        输出: guidepoint的2维坐标, (2,)
        '''
        seg_length = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)  # 每条线段的长度
        waypoints_length = np.cumsum(seg_length)  # 第1~NP个waypoints到起点的路径长度
        if waypoints_length[-1] < self.discounted_D:
            return waypoints[-1]  # 总路径长度小于局部规划长度，选择最后一个点作为引导点

        idx = np.argmax(waypoints_length > self.discounted_D)  # 路径长即将大于discounted_D的导航点的索引
        vector = waypoints[idx + 1] - waypoints[idx]
        if idx == 0: return waypoints[idx] + self.discounted_D * vector / np.linalg.norm(vector)
        else: return waypoints[idx] + (self.discounted_D-waypoints_length[idx-1]) * vector / np.linalg.norm(vector)

    def get_guidepoint_old(self, waypoints: np.ndarray) -> np.ndarray:
        '''
        会导致guidepoint抖动，已弃用
        功能: 根据waypoints计算Local Planner的局部引导点
        输入: N个waypoints的2维坐标, (N,2)
        输出: guidepoint的2维坐标, (2,)
        '''
        waypoints = self._waypoints_filter(waypoints) # 过滤掉距离太近的waypoints

        seg_lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1) # 每条线段的长度
        global_path_length = np.sum(seg_lengths) # global path的长度

        if global_path_length < self.length_trsd:
            # Case 1: 如果路径很短(即快到达终点)，则直接用最后一个waypoints(即targetpoint)作为guidepoint
            print('Case 1: 路径很短, 使用最后一个waypoints')
            return waypoints[-1]
        else:
            angles = self._calculate_waypoints_BendingAngles(waypoints) # (N-2,)
            if (angles <= self.angles_trsd).all():
                # Case 2: 路径为大直线：选用朝着waypoints[-1]方向，discounted_D距离内最近的点
                print('Case 2: 大直线')
                vector = waypoints[-1]-waypoints[0]
                return waypoints[0] + min(self.discounted_D,global_path_length)*vector/np.linalg.norm(vector)
            else:
                # 路径有转折，先寻找转折点作为临时导航点
                guide_idx = np.argmax(angles > self.angles_trsd) + 1 # 因为去掉了起始点，这里需要+1
                temp_guide_length = np.sum(seg_lengths[0:guide_idx]) # 起点到临时guidepoint的距离
                if temp_guide_length < self.discounted_D:
                    # Case 3: 起点到临时导航点的距离 < 局部路径规划阈值，选择临时导航点为guidepoint
                    print('Case 3: 转折点在D内', angles)
                    return waypoints[guide_idx]
                else:
                    # Case 4: 起点到临时导航点的距离 >= 局部路径规划阈值，选择朝向临时导航点且距离为discounted_D的点为guidepoint
                    print('Case 4: 转折点在D外')
                    vector = waypoints[guide_idx] - waypoints[0]
                    return waypoints[0] + self.discounted_D * vector / np.linalg.norm(vector)

    def _calculate_waypoints_BendingAngles(self, waypoints: np.ndarray) -> np.ndarray:
        """
        收入: N个waypoints的2维坐标, (N,2)
        输出: 除去起点和终点，剩余点的转折度, (N-2,)
        第K个点的转折度=向量(K-1,K)和向量(K,K+1)的夹角的绝对值
        """
        if waypoints.shape[0] < 3: raise ValueError("Requires at least 3 waypoints!")

        # 计算 (K-1, K) 和 (K, K+1) 的向量
        vectors1 = waypoints[1:-1] - waypoints[:-2]
        vectors2 = waypoints[2:] - waypoints[1:-1]

        # 计算向量的模长
        norms1 = np.linalg.norm(vectors1, axis=1)
        norms2 = np.linalg.norm(vectors2, axis=1)

        # 计算余弦值
        cos_theta = np.sum(vectors1 * vectors2, axis=1) / (norms1 * norms2+1e-6)

        # 返回转折度(夹角)
        return np.arccos(cos_theta)

    def _waypoints_filter(self, waypoints: np.ndarray) -> np.ndarray:
        seg_lengths = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)  # 每条线段的长度
        idx = np.where(seg_lengths > self.waypoints_trsd)[0] # 与后一个点距离大于waypoints_trsd的点的索引

        # 所有waypoints都很近: idx为[],此时返回3个waypoints
        if idx.size == 0: return waypoints[[0, int(self.NP/2), -1]]

        # 补头，补尾，让waypoints数量至少大于3
        if 0 not in idx: idx = np.append(0, idx)
        if self.S not in idx: idx = np.append(idx, self.S) # self.S=最后一个waypints的索引
        return waypoints[idx]

