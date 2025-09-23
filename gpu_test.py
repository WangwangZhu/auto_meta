import torch
if __name__ == '__main__':
	# 测试cuda（在无GPU环境下安全降级到CPU）
	device = 'cuda' if torch.cuda.is_available() else 'cpu'
	print("Support CUDA?:", torch.cuda.is_available())
	x = torch.tensor([10.0], device=device)
	print(x)
	y = torch.randn(2, 3, device=device)
	print(y)
	z = x + y
	print(z)
	# 测试CUDNN（仅在CUDA可用时检查）
	from torch.backends import cudnn
	print("Support cudnn ?:", cudnn.is_available() and device == 'cuda')
