from Sparrow_Gazebo_single import Gazebo_fml_robot_env,str2bool
from utils.PoE import PositionalEncoding_NTD
from utils.TWQ import TimeWindowQueue_NTD
import torch.nn.functional as F
import torch.nn as nn
import torch, pygame
import argparse
import rospy


parser = argparse.ArgumentParser()
'''Hyperparameter Setting for DRL'''
parser.add_argument('--ModelIdex', type=str, default='4595k', help='which model to load: 4595k / 2041k / 1977k')
parser.add_argument('--T', type=int, default=10, help='length of time window')
parser.add_argument('--H', type=int, default=8, help='Number of Head')
parser.add_argument('--L', type=int, default=3, help='Number of Transformer Encoder Layers')
parser.add_argument('--net_width', type=int, default=64, help='Linear net width')


'''Hyperparameter Setting for SparrowGazebo'''
parser.add_argument('--ctrl_freq', type=int, default=10, help='10Hz是Sparrow的控制频率')
parser.add_argument('--ld_range', type=int, default=300, help='雷达感知距离; cm')
parser.add_argument('--ld_num', type=int, default=240, help='雷达线数')
parser.add_argument('--ld_GN', type=int, default=10, help='雷达分组后的组内线数')
parser.add_argument('--D', type=int, default=400, help='最大局部规划距离(必须和Sparrow训练时一致)， cm')
parser.add_argument('--py_render', type=str2bool, default=False, help='whether to use pygame render')
opt = parser.parse_args()
opt.dvc = torch.device('cuda')

def play_with_pygame():
    # Env init:
    env = Gazebo_fml_robot_env(**vars(opt))
    opt.state_dim = env.state_dim
    opt.action_dim = env.action_dim
    print("\nPress 'Esc' to reset environment.")
    print("\nPress 'Up' to let agent control the robot.")

    # Agent init:
    agent = DDQN_Transformer_Agent_play(opt)
    agent.load(opt.ModelIdex)

    agent.queue.clear()
    s = env.reset()
    agent_ctrl = False
    while True:
        keys = pygame.key.get_pressed()
        if keys[pygame.K_ESCAPE]:
            agent.queue.clear()
            s = env.reset()
            agent_ctrl = False
        elif keys[pygame.K_UP]:
            agent_ctrl = True

        if agent_ctrl:
            if env.win: a = 7 # stop
            else:
                agent.queue.append(s)  # 将s加入时序窗口队列
                TW_s = agent.queue.get()  # 取出队列所有数据
                a = agent.select_action(TW_s, deterministic=False)
        else:
            a = 7 # stop
        s = env.step(a)
        rate.sleep()

def play_with_rviz():
    # Env init:
    env = Gazebo_fml_robot_env(**vars(opt))
    opt.state_dim = env.state_dim
    opt.action_dim = env.action_dim
    print("\nUse 'Rviz: 2D Nav Goal' to set waypoint.")

    # Agent init:
    agent = DDQN_Transformer_Agent_play(opt)
    agent.load(opt.ModelIdex)

    agent.queue.clear()
    s = env.reset()
    while True:
        if env.win: a = 7 # stop
        else:
            agent.queue.append(s)  # 将s加入时序窗口队列
            TW_s = agent.queue.get()  # 取出队列所有数据
            a = agent.select_action(TW_s, deterministic=False)
        s = env.step(a)
        rate.sleep() # 延时，保证Temporal Window Queue中的时间间隔为0.1s(rostime)


def orthogonal_init(layer, gain=1.414):
    for name, param in layer.named_parameters():
        if 'bias' in name:
            nn.init.constant_(param, 0)
        elif 'weight' in name:
            nn.init.orthogonal_(param, gain=gain)
    return layer


class Q_Transformer(nn.Module):
    def __init__(self, opt):
        super(Q_Transformer, self).__init__()
        self.D = opt.state_dim - 8 # s[0:7]是姿态, s[8:]是雷达
        # Define the Transformer Encoder block(note that state_dim should be a even number):
        self.pe = PositionalEncoding_NTD(maxlen=opt.T, emb_size=self.D) # for (N,T,D)
        encoder_layer = nn.TransformerEncoderLayer(d_model=self.D, nhead=opt.H, dropout=0,
                                                   dim_feedforward=opt.net_width, batch_first=True)
        self.transformer_encoder = nn.TransformerEncoder(encoder_layer, num_layers=opt.L)

        self.fc1 = orthogonal_init(nn.Linear(int(self.D + opt.state_dim), opt.net_width))
        self.fc2 = orthogonal_init(nn.Linear(opt.net_width, int(opt.net_width/2)))
        self.fc3 = orthogonal_init(nn.Linear(int(opt.net_width/2), opt.action_dim))

    def forward(self, TW_s):
        '''TW_s.shape = (B,T,D)'''
        temporal_ld = TW_s[:,:,8:]  # s[0:7]是姿态信息, s[8:]是雷达信息
        temporal_ld = self.pe(temporal_ld) # (N,T,D)
        temporal_ld = self.transformer_encoder(temporal_ld) # (N,T,D)
        temporal_ld = temporal_ld.mean(dim=1) # (N,T,D) ->  (N,D)

        aug_s = torch.cat((temporal_ld,TW_s[:,0,:]),dim=-1) # (N,D+S_dim)

        q = F.relu(self.fc1(aug_s)) # (N,256)
        q = F.relu(self.fc2(q)) # (N,128)
        q = self.fc3(q)  # (N,a_dim)
        return q


class DDQN_Transformer_Agent_play(object):
    def __init__(self,opt):
        self.dvc = opt.dvc
        self.action_dim = opt.action_dim
        self.N = 1

        # Build Transformer networks
        self.q_net = Q_Transformer(opt).to(self.dvc)

        # vectorized e-greedy exploration
        self.p = torch.ones(self.N, device=self.dvc) * 0.02

        # temporal window related:
        self.queue = TimeWindowQueue_NTD(self.N, opt.T, opt.state_dim, opt.dvc, padding=0)  # 用于model test


    def select_action(self, TW_s, deterministic):
        '''Input: batched state in (N, T, s_dim) on device
           Output: batched action, (N,), torch.tensor, on device '''
        with torch.no_grad():
            a = self.q_net(TW_s).argmax(dim=-1)
            if deterministic:
                return a
            else:
                replace = torch.rand(self.N, device=self.dvc) < self.p  # [n]
                rd_a = torch.randint(0, self.action_dim, (self.N,), device=self.dvc)
                a[replace] = rd_a[replace]
                return a

    def load(self,idx):
        self.q_net.load_state_dict(torch.load(f"./model/{idx}.pth", map_location=self.dvc, weights_only=True))


if __name__ == '__main__':
    rospy.init_node('Agent', anonymous=True)
    rate = rospy.Rate(opt.ctrl_freq)
    if opt.py_render: play_with_pygame()
    else: play_with_rviz()
