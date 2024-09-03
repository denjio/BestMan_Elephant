'''
TSAI手眼标定法
----------------------------------------------
@作者: 阿凯爱玩机器人
@QQ: 244561792
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@B站: https://space.bilibili.com/40344504
'''
import numpy as np
import cv2
# 自定义库
from transform import Transform

def solve_axb_by_svd(A, b):
	'''求解AX=b
	SVD法-求解最小二乘问题
	参考: https://andreask.cs.illinois.edu/cs357-s15/public/demos/09-svd-applications/Least%20Squares%20using%20the%20SVD.html
	'''
	U, sigma, VT = np.linalg.svd(A)
	Sigma = np.zeros(A.shape)
	Sigma[:3,:3] = np.diag(sigma)
	Sigma_pinv = np.zeros(A.shape).T
	Sigma_pinv[:3,:3] = np.diag(1/sigma[:3])
	X = VT.T.dot(Sigma_pinv).dot(U.T).dot(b)
	return X

def solve_axb(A, b):
	'''求解Ax=b的问题'''
	# 注: 使用下列的方法，结果上没啥差别。
	# Normal Equation法
	# x = np.dot(np.dot(np.linalg.inv(np.dot(A1.T, A1)), A1.T), b1)
	# SVD法求解最小二乘
	x = solve_axb_by_svd(A, b)
	# 使用OpenCV的内置的API: SVD法求解最小二乘
	# ret, x = cv2.solve(A1, b1, flags=cv2.DECOMP_SVD)
	return x
	
def solve_axxb(T_A_list, T_B_list):
	'''求解AX=XB
	其中A,B为已知量, X为未知量。
	'''
	# 获得数据对数
	n_pair = len(T_A_list)
	# ======求解Rx======
	# 构造A1与b1
	A1 = []
	b1 = []
	for i in range(n_pair):
		# 获得空间变换
		T_Ai = T_A_list[i]
		T_Bi = T_B_list[i]
		# 提取旋转矩阵
		R_Ai = T_Ai[:3, :3]
		R_Bi = T_Bi[:3, :3]
		# 将旋转矩阵转换为修正罗德里格斯参数
		N_Ai = Transform.rmat2mrp(R_Ai)
		N_Bi = Transform.rmat2mrp(R_Bi)
		# 计算Skew(N_Ai + N_Bi)
		A1.append(Transform.skew(N_Ai+N_Bi))
		# 计算N_Ai-N_Bi
		b1.append((N_Bi - N_Ai).reshape(-1, 1))
	A1 = np.float64(np.vstack(A1))
	b1 = np.float64(np.vstack(b1))
	# 最小二乘法求解Nx^{'}
	# Nx^{'} = (A1^{T}*A1)^{-1}*A1.T*b1
	Nx_dot = solve_axb(A1, b1)
	# Nx^{'}的模长 |Nx^{'}|
	Nx_dot_norm = np.linalg.norm(Nx_dot)
	# 求解Nx
	Nx = 2.0*(Nx_dot / np.sqrt(1 + Nx_dot_norm*Nx_dot_norm))
	# 将Nx转换为旋转矩阵
	R_X = Transform.mrp2rmat(Nx)
	# ======求解tx======
	# 构造A2与b2
	A2 = []
	b2 = []
	for i in range(n_pair):
		# 获得空间变换
		T_Ai = T_A_list[i]
		T_Bi = T_B_list[i]
		# 提取旋转矩阵
		R_Ai = T_Ai[:3, :3]
		R_Bi = T_Bi[:3, :3]
		# 提取平移向量
		t_Ai = T_Ai[:3, 3].reshape((-1, 1))
		t_Bi = T_Bi[:3, 3].reshape((-1, 1))
		A2.append(R_Ai - np.eye(3))
		b2.append(np.dot(R_X, t_Bi) - t_Ai)
	A2 = np.float64(np.vstack(A2))
	b2 = np.float64(np.vstack(b2))
	# 最小二乘法求解t_X
	t_X = solve_axb(A2, b2)
	# ======拼接得到Tx======
	T_X = np.eye(4)
	T_X[:3, :3] = R_X
	T_X[:3, 3] = t_X.reshape(-1)
	return T_X

def calibration_eye_in_hand(T_arm2wrist_list, T_cam2board_list):
	'''手眼标定-眼在手上
	[输入参数]
	@T_arm2wrist_list: 机械臂腕关节/工具坐标系在机械臂基坐标系下的位姿， 通过机械臂SDK/示教器可以获得. 
	@T_cam2board_list: 相机坐标系到标定板坐标系之间的空间变换
	[返回参数]
	@T_wrist2cam: 腕关节/工具坐标系到相机坐标系空间变换
	'''
	# 样本个数
	n_sample = len(T_arm2wrist_list)
	# 生成T_A_list
	T_A_list = []
	T_B_list = []
	for i in range(n_sample-1):
		for j in range(i+1, n_sample):
			# 获取i,j时刻，腕关节在机械臂基坐标系下的位姿
			T_arm2wristi = T_arm2wrist_list[i]
			T_arm2wristj = T_arm2wrist_list[j]
			# 获取i,j时刻，标定板在相机坐标系下的位姿
			T_cami2board = T_cam2board_list[i]
			T_camj2board = T_cam2board_list[j]
			# 求解腕关节坐标系i到腕关节坐标系j的空间变换
			T_wristi2wristj = np.dot(Transform.inverse(T_arm2wristi), T_arm2wristj)
			# 求解相机坐标系i到相机坐标系j的空间变换
			T_cami2camj = np.dot(T_cami2board, Transform.inverse(T_camj2board))
			# 将T_wristi2wristj添加到A列表里面
			T_A_list.append(T_wristi2wristj)
			# 将T_cami2camj添加到B列表里面
			T_B_list.append(T_cami2camj)
	# 求解AX=XB
	T_wrist2cam = solve_axxb(T_A_list, T_B_list)
	return T_wrist2cam

def calibration_eye_to_hand(T_arm2wrist_list, T_cam2board_list):
	'''手眼标定-眼在手外
	[输入参数]
	@T_arm2wrist_list: 机械臂腕关节/工具坐标系在机械臂基坐标系下的位姿， 通过机械臂SDK/示教器可以获得. 
	@T_cam2board_list: 相机坐标系到标定板坐标系之间的空间变换
	[返回参数]
	@T_arm2cam: 机械臂基坐标系到相机坐标系下的空间变换。
	'''
	# 样本个数
	n_sample = len(T_arm2wrist_list)
	# 生成T_A_list
	T_A_list = []
	T_B_list = []
	for i in range(n_sample-1):
		for j in range(i+1, n_sample):
			# 获取i,j时刻，腕关节在机械臂基坐标系下的位姿
			T_arm2wristi = T_arm2wrist_list[i]
			T_arm2wristj = T_arm2wrist_list[j]
			# 获取i,j时刻，标定板在相机坐标系下的位姿
			T_cam2boardi = T_cam2board_list[i]
			T_cam2boardj = T_cam2board_list[j]
			# 求解 T_Aij
			# T_Aij = ^{arm}_{wrist_j}T * (^{arm}_{wrist_i}T)^{-1}
			T_A = np.dot(T_arm2wristj, Transform.inverse(T_arm2wristi))
			# 求解 T_Bij
			# T_Bij = ^{cam}_{board_j}T * (^{cam}_{board_i}T)^{-1}
			T_B = np.dot(T_cam2boardj, Transform.inverse(T_cam2boardi))
			# 将T_wristi2wristj添加到A列表里面
			T_A_list.append(T_A)
			# 将T_cami2camj添加到B列表里面
			T_B_list.append(T_B)
	# 求解AX=XB
	T_arm2cam = solve_axxb(T_A_list, T_B_list)
	return T_arm2cam
