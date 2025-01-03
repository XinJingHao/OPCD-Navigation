import torch

class TimeWindowReplayBuffer_NTD():
	'''Experience replay buffer for vector env with time window state'''
	def __init__(self, opt):
		self.dvc = opt.dvc
		self.max_size = int(opt.buffersize/opt.N)
		self.state_dim = opt.state_dim
		self.N = opt.N
		self.ptr = 0
		self.size = 0
		self.full = False
		self.batch_size = opt.batch_size

		self.s = torch.zeros((self.max_size, opt.N, opt.T, opt.state_dim), device=self.dvc) # TW_s
		self.a = torch.zeros((self.max_size, opt.N, 1), dtype=torch.int64, device=self.dvc)
		self.r = torch.zeros((self.max_size, opt.N, 1), device=self.dvc)
		self.dw = torch.zeros((self.max_size, opt.N, 1), dtype=torch.bool, device=self.dvc)
		self.ct = torch.zeros((self.max_size, opt.N, 1),dtype=torch.bool, device=self.dvc)


	def add(self, s, a, r, dw, ct):
		'''add transitions to buffer'''
		self.s[self.ptr] = s # TW_s, shape=(N, T, state_dim); Batched TimeWindow state
		self.a[self.ptr] = a.unsqueeze(-1)  #(N,) to (N,1)
		self.r[self.ptr] = r.unsqueeze(-1)
		self.dw[self.ptr] = dw.unsqueeze(-1)
		self.ct[self.ptr] = ct.unsqueeze(-1)

		self.ptr = (self.ptr + 1) % self.max_size
		self.size = min(self.size + 1, self.max_size)
		if self.size == self.max_size:
			self.full = True

	def sample(self):
		'''sample batch transitions'''
		if not self.full:
			ind = torch.randint(low=0, high=self.ptr - 1, size=(self.batch_size,), device=self.dvc)  # sample from [0, ptr-2]
		else:
			ind = torch.randint(low=0, high=self.size - 1, size=(self.batch_size,), device=self.dvc)  # sample from [0, size-2]
			if self.ptr - 1 in ind: ind = ind[ind != (self.ptr - 1)] # delate ptr - 1 in [0, size-2]

		env_ind = torch.randint(low=0, high=self.N, size=(len(ind),), device=self.dvc) # [l,h)
		# [N, T, s_dim], [N, 1], [N, 1], [N, T, s_dim], [N, 1], [N, 1]
		return (self.s[ind,env_ind], self.a[ind,env_ind], self.r[ind,env_ind],
				self.s[ind + 1,env_ind], self.dw[ind,env_ind], self.ct[ind,env_ind])

