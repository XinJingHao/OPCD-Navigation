import torch


class TimeWindowQueue_NTD:
    def __init__(self, N, T, D, device, padding):
        self.device = device
        self.padding = padding
        self.T = T

        # 初始化缓冲区，形状为 (N, T, D), 即transformer(batch_first=True)要求的(batch_size, seq_len, emb_dim)
        if padding == -1: self.window = -torch.ones(N, T, D, device=self.device) # 便于可视化
        elif padding == 0: self.window = torch.zeros(N, T, D, device=self.device)
        else: raise ValueError("Wrong padding value")
        self.ptr = 0


    def append(self, batched_transition: torch.tensor):
        """ batched_transition, shape=(B,D): batched transition from vectorized envs """

        # 将数据写入缓冲区
        self.window[:, self.T - 1 - self.ptr, :] = batched_transition # (B,D), 由下往上写入，保证roll的输出顺序

        # 更新写指针和计数器
        self.ptr = (self.ptr + 1) % self.T

    def get(self) -> torch.tensor:
        """
        获取时间窗口buffer中的所有数据, shape=(N, T, D), 使用roll保证数据按时序正确排列
        """
        TimeWindow_data = torch.roll(self.window, shifts=self.ptr, dims=1) # (N, T, D)

        return TimeWindow_data


    def padding_with_done(self, done_flag: torch.tensor):
        """
        根据done_flag，将buffer中对应batch位置置零
        :param done_flag: shape=(N,)
        """
        self.window[done_flag, :, :] = self.padding

    def clear(self):
        self.window.fill_(self.padding)



"""
TimeWindowQueue将vectorized envs(带auto-truncation)的batched_state存入长度为T的时序窗口中。

例子如下：
对应trajectory链接：https://github.com/XinJingHao/Images/blob/main/Sparrow_V1/AutoReset.svg


| t   | TW_s              | TW_s mask   | Action | Reward | dw  | ct  | s_next |
| --- | ----------------- | ----------- | ------ | ------ | --- | --- | ------ |
| 0   | [S0,--,--,--,--]  | [F,T,T,T,T] | A0     | R1     | F1  | T0  | S1     |
| 1   | [S1,S0,--,--,--]  | [F,F,T,T,T] | A1     | R2     | T2  | T1  | S2     |
| 2   | [S2,S1,S0,--,--]  | [F,F,F,T,T] | A2     | 0      | F0  | F2  | S0'    |
| 0   | [S0',S2,S1,S0,--] | [F,T,T,T,T] | A0'    | R1'    | F1' | T0' | S1'    |
"""

# # 参数设置
# N = 2  # batch size
# T = 5  # 时间窗口长度
# D = 3  # 特征大小
#
# # 初始化队列
# queue = TimeWindowQueue_NDT(N, D, T, device='cpu', padding=-1)
#
# # 模拟数据输入和获取
# s = torch.full((N, D), 0, device='cpu')
# ct = torch.tensor([True, True])
# for t in range(0, 9):
#
#     '''加入TWQ，TW_s = get(), 位置编码'''
#     queue.append(s) # valid_length = 1
#     TW_s = queue.get() # [s0, 0, 0, 0]
#
#     '''a = agent.select_action(PE_TW_s, mask)'''
#
#     '''s_next, r, dw, tr, info = envs.step(a)''' # 生成下一时刻数据
#     dw = torch.tensor([t==1, t==5])
#     tr = torch.tensor([False, False])
#     s_next = torch.full((N, D), (t+1)%3, device='cpu')
#     s_next[1] *= 10
#
#     '''buffer.add(PE_TW_s, mask, a, r, dw, ct)'''
#     print(f"时间 {t%3}:")
#     print(f'dw:{dw[0]}, ct:{ct[0]}')
#     print(TW_s.numpy()[0,:,0])
#     print()
#     queue.padding_with_done(~ct)  # 用上一时刻的ct去设置valid_length
#
#
#     ct = ~(dw + tr)
#     s = s_next



