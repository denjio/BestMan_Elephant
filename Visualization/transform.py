'''
刚体空间变换 工具库
----------------------------------------------
@作者: 阿凯爱玩机器人
@QQ: 244561792
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@B站: https://space.bilibili.com/40344504
'''
import math
import numpy as np
from numpy import sin, cos, pi

class Transform:
	@staticmethod
	def dxmat(dx):
		'''沿X轴平移'''
		return np.float64([
			[1, 0, 0, dx],
			[0, 1, 0, 0],
			[0, 0, 1, 0],
			[0, 0, 0, 1]])
	
	@staticmethod
	def dymat(dy):
		'''沿Y轴平移'''
		return np.float64([
			[1, 0, 0, 0],
			[0, 1, 0, dy],
			[0, 0, 1, 0],
			[0, 0, 0, 1]])
	
	@staticmethod
	def dzmat(dz):
		'''沿Z轴平移'''
		return np.float64([
			[1, 0, 0, 0],
			[0, 1, 0, 0],
			[0, 0, 1, dz],
			[0, 0, 0, 1]])
	
	@staticmethod
	def rxmat(gamma):
		'''绕X轴旋转'''
		return np.array([
			[1, 0,           0,          0],
			[0, cos(gamma), -sin(gamma), 0],
			[0, sin(gamma),  cos(gamma), 0],
			[0, 0,           0,          1]])

	@staticmethod
	def rymat(beta):
		'''绕Y轴旋转'''
		return np.array([
			[cos(beta),  0,  sin(beta), 0],
			[0,          1,  0,         0],
			[-sin(beta), 0,  cos(beta), 0],
			[0,           0,  0,         1]])
	@staticmethod
	def rzmat(alpha):
		return np.array([
			[cos(alpha), -sin(alpha), 0, 0],
			[sin(alpha),  cos(alpha), 0, 0],
			[0,           0,          1, 0],
			[0,           0,          0, 1]])
	
	@staticmethod
	def dhmat(alpha, a, theta, d):
		'''DH变换矩阵'''
		dhmat = Transform.rxmat(alpha)
		dhmat = dhmat.dot(Transform.dxmat(a))
		dhmat = dhmat.dot(Transform.rzmat(theta))
		dhmat = dhmat.dot(Transform.dzmat(d))
		return dhmat
	
	@staticmethod
	def inverse(T):
		'''齐次变换矩阵求逆'''
		R = T[:3, :3]
		t = T[:3, 3].reshape((-1, 1))
		R_T = R.T
		T_inv = np.eye(4)
		T_inv[:3, :3] = R_T
		T_inv[:3, 3] = -R_T.dot(t).reshape(-1)
		return T_inv
	
	@staticmethod
	def euler2rmat(roll=0, pitch=0, yaw=0):
		'''欧拉角转换为旋转矩阵''' 
		alpha, beta, gamma = yaw, pitch, roll
		cos_gamma = np.cos(gamma)
		sin_gamma = np.sin(gamma)
		cos_beta = np.cos(beta)
		sin_beta = np.sin(beta)
		cos_alpha = np.cos(alpha)
		sin_alpha = np.sin(alpha)

		r11 = cos_alpha*cos_beta
		r12 = -sin_alpha*cos_gamma + sin_beta*sin_gamma*cos_alpha
		r13 = sin_alpha*sin_gamma + sin_beta*cos_alpha*cos_gamma
		r21 = sin_alpha*cos_beta
		r22 = sin_alpha*sin_beta*sin_gamma + cos_alpha*cos_gamma
		r23 = sin_alpha*sin_beta*cos_gamma - sin_gamma*cos_alpha
		r31 = -sin_beta
		r32 = sin_gamma*cos_beta
		r33 = cos_beta*cos_gamma
		return np.array([
			[r11, r12, r13],
			[r21, r22, r23],
			[r31, r32, r33]])
	
	@staticmethod
	def rmat2euler(rmat):
		'''旋转矩阵转换为欧拉角'''
		alpha = None # 偏航角
		beta = None  # 俯仰角
		gamma = None # 横滚角
		
		r11, r12, r13, r21, r22, r23, r31, r32, r33 = rmat.reshape(-1)
		if abs(r31) >= (1 - 0.000001):
			# 出现万向锁的问题
			if r31 < 0:
				gamma = 0
				beta = np.pi/2
				alpha = math.atan2(r23, r22)
				return [[gamma, beta, alpha]]
			else:
				gamma = 0
				beta = -np.pi/2
				alpha = math.atan2(-r23, r22)
				return [[gamma, beta, alpha]]
		else:
			# 正常求解
			cos_beta = np.sqrt(r32*r32 +r33*r33)
			cos_beta_list = [cos_beta, -cos_beta]
			rpy_list = []
			for cos_beta in cos_beta_list:
				beta = math.atan2(-r31, cos_beta)
				alpha = math.atan2(r21/cos_beta, r11/cos_beta)
				gamma = math.atan2(r32/cos_beta, r33/cos_beta)
				rpy_list.append([gamma, beta, alpha])
			return rpy_list
	
	@staticmethod
	def skew(n):
		'''生成Skew矩阵'''
		xn, yn, zn = n.reshape(-1)
		skew_mat = np.float64([
			[0, -zn, yn],
			[zn, 0, -xn],
			[-yn, xn, 0]])
		return skew_mat
	
	@staticmethod
	def rvect2rmat(u, theta=None):
		'''旋转向量转换为旋转矩阵'''
		# 转轴
		u = np.float64(u).reshape(-1)
		# 转轴模长
		u_norm = np.linalg.norm(u)
		if theta is None:
			# 如果没有指定theta则将旋转向量模长作为角度
			theta = u_norm
		# 转轴归一化
		u_unit = u / u_norm
		n1, n2, n3 = u_unit
		# 为了减少计算量，预先计算好
		a = np.sin(theta)
		b = 1 - np.cos(theta)
		an1 = a*n1
		an2 = a*n2
		an3 = a*n3
		bn1n1 = b*n1*n1
		bn2n2 = b*n2*n2
		bn3n3 = b*n3*n3
		bn1n2 = b*n1*n2
		bn2n3 = b*n2*n3
		bn1n3 = b*n1*n3
		# 计算旋转矩阵R
		R = np.float64([
			[1-bn2n2-bn3n3, -an3+bn1n2, an2+bn1n3],
			[an3+bn1n2, 1-bn1n1-bn3n3, -an1+bn2n3],
			[-an2+bn1n3, an1+bn2n3, 1-bn1n1-bn2n2]
		])
		return R
	
	@staticmethod
	def rvect2rmat2(u, theta=None):
		'''旋转向量转换为旋转矩阵'''
		# 转轴
		u = np.float64(u).reshape(-1)
		# 转轴模长
		u_norm = np.linalg.norm(u)
		if theta is None:
			# 如果没有指定theta则将旋转向量模长作为角度
			theta = u_norm
		# 转轴归一化
		u_unit = u / u_norm
		n1, n2, n3 = u_unit
		# 构造矩阵 n*n^{T}
		mat_n_n_trans = np.float64([
			[n1**2, n1*n2, n1*n3],
			[n1*n2, n2**2, n2*n3],
			[n1*n3, n2*n3, n3**2]])
		# 构造矩阵 n_hat
		mat_n_hat = np.float64([
			[0, -n3, n2],
			[n3, 0, -n1],
			[-n2, n1, 0]])
		# 罗德里格斯公式
		I = np.eye(3)
		cos_theta = cos(theta)
		sin_theta = sin(theta)
		R = cos_theta*I + (1-cos_theta)*mat_n_n_trans + sin_theta*mat_n_hat
		return R
	
	@staticmethod
	def rmat2rvect(rmat):
		'''旋转矩阵转换为旋转向量'''
		# 提取rmat中的元素 
		r11, r12, r13, r21, r22, r23, r31, r32, r33 = rmat.reshape(-1)
		# 计算旋转矩阵R的迹
		trace_rmat = r11 + r22 + r33
		# 根据迹的取值来分流 
		if trace_rmat >=3:
			# 单位矩阵
			# 转轴任意制定，旋转角度为0
			return [0, 0, 1], 0
		elif trace_rmat <= -1:
			# 绕X轴/Y轴/Z轴定轴旋转的情况
			# 转角为pi, 转轴为X, Y, Z基向量中的一个
			n = [1 if rii == 1 else 0 for rii in [r11, r22, r33]]
			return n, np.pi
		else:
			# arccos(( trace(R) - 1 ) / 2)
			theta = np.arccos(0.5*(trace_rmat - 1))
			# 1 / ( 2 * sin(theta))
			ratio = 0.5 / np.sin(theta)
			n = ratio * np.float64([r32-r23, r13-r31, r21-r12])
			return n, theta
	
	@staticmethod
	def rmat2rvect2(rmat):
		'''旋转矩阵转换为旋转向量'''
		# 先将旋转矩阵转换为四元数
		rmat = np.float64(rmat)
		m11, m12, m13, m21, m22, m23, m31, m32, m33 = rmat.reshape(-1)
		# trace是矩阵的迹, 是矩阵主对角元素之和
		# trace(rmat) = m11 + m22 + m33
		trace_rmat = m11 + m22 + m33
		if  trace_rmat > 0:
			# 注: q0不能是0， 否则就变成了纯四元数了
			# 就不是旋转四元数了
			# S = 4q0
			s = np.sqrt(trace_rmat+1) * 2.0 # S = 4*qw
			inv_s = 1.0 / s
			qw = 0.25 * s
			qx = (m32 - m23) * inv_s
			qy = (m13 - m31) * inv_s
			qz = (m21 - m12) * inv_s
		elif m11 > m22 and m11 > m33:
			s = np.sqrt(1.0 + m11 - m22 - m33) * 2 # S = 4*qx
			inv_s = 1.0 / s
			qw = (m32 - m23) * inv_s
			qx = 0.25 * s
			qy = (m12 + m21) * inv_s
			qz = (m13 + m31) * inv_s
		elif m22 > m33:
			s = np.sqrt(1.0 - m11 + m22 - m33) * 2 # S = 4*qy
			inv_s = 1.0 / s
			qw = (m13 - m31) * inv_s
			qx = (m12 + m21) * inv_s
			qy = 0.25 * s
			qz = (m23 + m32) * inv_s
		else:
			s = np.sqrt(1.0 - m11 - m22 + m33) * 2 # S = 4*qz
			inv_s = 1.0 / s
			qw = (m21 - m12)
			qx = (m13 + m31)
			qy = (m23 + m32)
			qz = 0.25 * s
		# 角度的1/2
		theta_d2 = np.arccos(qw)
		theta = theta_d2 * 2
		sin_theta_d2 = sin(theta_d2)
		ux = qx / sin_theta_d2
		uy = qy / sin_theta_d2
		uz = qz / sin_theta_d2
		return np.float64([ux, uy, uz]), theta
	
	@staticmethod
	def mrp2rvect(N):
		'''修正罗德里格斯公式转换为旋转向量'''
		# 求解theta
		theta = np.arcsin(0.5*np.linalg.norm(N))*2
		n = N / (2*np.sin(0.5*theta))
		return n, theta
	
	@staticmethod
	def rvect2mrp(n, theta):
		'''旋转向量转换为修正罗德里格斯参数'''
		N = 2*np.sin(0.5*theta)*n
		return N
		
	@staticmethod
	def mrp2rmat(N):
		'''修正罗德里格斯参数转换为旋转矩阵'''
		# 强制将N变为列向量
		N = N.reshape((-1, 1))
		# N的模长
		N_norm = np.linalg.norm(N)
		# N的模长平方
		N_norm_pw2 = N_norm*N_norm
		# 修正罗德里格斯公式
		I = np.eye(3)
		R = (1-0.5*N_norm_pw2)*I + 0.5*(N*N.T + \
			np.sqrt(4-N_norm_pw2)*Transform.skew(N))
		return R
	
	@staticmethod
	def rmat2mrp(rmat):
		'''旋转矩阵转换为修正罗德里格斯参数'''
		# 旋转矩阵转换为旋转向量
		n, theta = Transform.rmat2rvect(rmat)
		# 通过旋转向量构造旋转矩阵
		N = 2*np.sin(0.5*theta)*n
		return N