from Env_Color import Gazebo_fml_robot_env,str2bool
import torch.nn as nn
import torch
import argparse
import rospy


parser = argparse.ArgumentParser()
parser.add_argument('--C', type=int, default=100, help='Evaluation Turns')

'''Hyperparameter Setting for DRL'''
parser.add_argument('--ModelIdex', type=str, default='Color', help='from https://github.com/XinJingHao/Sparrow-V1/tree/Sparrow-V1.2/model')
parser.add_argument('--net_width', type=int, default=256, help='Linear net width')


'''Hyperparameter Setting for SparrowGazebo'''
parser.add_argument('--ctrl_freq', type=int, default=10, help='10Hz是Sparrow的控制频率')
parser.add_argument('--ld_range', type=int, default=100, help='雷达感知距离; cm')
parser.add_argument('--ld_num', type=int, default=189, help='雷达线数')
parser.add_argument('--ld_GN', type=int, default=7, help='雷达分组后的组内线数')
parser.add_argument('--D', type=int, default=400, help='最大局部规划距离(必须和Sparrow训练时一致)， cm')
parser.add_argument('--startpose', type=tuple, default=(9,-9,2.3), help='小车reset位姿: x,y,theta')
opt = parser.parse_args()
opt.dvc = torch.device('cpu')


def play_with_rviz():
    # Env init:
    env = Gazebo_fml_robot_env(**vars(opt))
    opt.state_dim = env.state_dim - 4  # 去除Color不需要的状态
    opt.action_dim = env.action_dim - 2  # 去除Color不需要的动作

    # Agent init:
    agent = DDQN_Agent(opt)
    agent.load('Color')
    agent.p *= 0.01  # evaluation时，加上一点e-greedy噪声，防止deadlock


    travel_distance = travel_time = avg_v = win_cnt = 0
    for i in range(1,opt.C+1):
        s = env.reset()

        t_start = rospy.Time.now()
        while True:
            s = s[4:].to(opt.dvc)
            a = agent.select_action(s)
            s, win, collide = env.step(a)

            if win or collide:
                if win: win_cnt += 1

                '''当前回合数据:'''
                temp_travel_distance = round(env.get_traj_length(), 2)
                temp_travel_time = round((rospy.Time.now() - t_start).to_sec(), 2)
                temp_avg_v = round(temp_travel_distance / temp_travel_time, 2)
                print(f'Evaluation {i} >> Travel distance:{temp_travel_distance}m, Travel time:{temp_travel_time}s, Average velocity:{temp_avg_v}m/s, Arrived:{win}')

                # '''所有回合平均数据:'''
                # if win:
                #     travel_distance += temp_travel_distance
                #     travel_time += temp_travel_time
                #     avg_v += temp_avg_v
                #     print(f'Averaged Result >> Travel distance:{round(travel_distance/win_cnt, 2)}m, '
                #           f'Travel time:{round(travel_time/win_cnt, 2)}s, '
                #           f'Average velocity:{round(avg_v/win_cnt, 2)}m/s, '
                #           f'Success rate:{round(win_cnt/i, 2)} \n')

                break



def orthogonal_init(layer, gain=1.414):
    for name, param in layer.named_parameters():
        if 'bias' in name:
            nn.init.constant_(param, 0)
        elif 'weight' in name:
            nn.init.orthogonal_(param, gain=gain)
    return layer

def build_net(layer_shape, activation, output_activation):
    '''build net with for loop'''
    layers = []
    for j in range(len(layer_shape)-1):
        act = activation if j < len(layer_shape)-2 else output_activation
        layers += [orthogonal_init(nn.Linear(layer_shape[j], layer_shape[j+1])), act()]
    return nn.Sequential(*layers)

class Q_Net(nn.Module):
    def __init__(self, state_dim, action_dim, hid_shape):
        super(Q_Net, self).__init__()
        layers = [state_dim] + list(hid_shape) + [action_dim]
        self.Q = build_net(layers, nn.ReLU, nn.Identity)

    def forward(self, s):
        q = self.Q(s)
        return q

class DDQN_Agent(object):
    def __init__(self,opt):
        self.dvc = opt.dvc
        self.q_net = Q_Net(opt.state_dim, opt.action_dim, [opt.net_width,int(opt.net_width/2)]).to(self.dvc)
        self.actor_envs = 1
        self.action_dim = opt.action_dim


        # vectorized e-greedy exploration
        self.p = torch.ones(self.actor_envs, device=self.dvc)


    def select_action(self, s):
        '''Input: batched state, (n,s_dim), torch.tensor, on device
           Output: batched action, (n,), torch.tensor, on device '''
        with torch.no_grad():
            a = self.q_net(s).argmax(dim=-1)
            return a


    def load(self,steps):
        self.q_net.load_state_dict(torch.load("./model/{}.pth".format(steps),weights_only=True,map_location=self.dvc))


if __name__ == '__main__':
    rospy.init_node('Agent', anonymous=True)
    play_with_rviz()
