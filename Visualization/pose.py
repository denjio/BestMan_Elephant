
'''
位姿描述 
----------------------------------------------
@作者: 阿凯爱玩机器人
@QQ: 244561792
@微信: xingshunkai
@邮箱: xingshunkai@qq.com
@B站: https://space.bilibili.com/40344504
'''
import math
import numpy as np
from transform import Transform
from quaternion import Quaternion

class Pose:
	'''位姿
	'''
	# 坐标
	x = 0
	y = 0
	z = 0
	# 旋转矩阵
	rmat = np.eye(3)
	# 欧拉角
	roll = 0
	pitch = 0.0
	yaw = 0.0
	
	def set_position(self, x, y, z):
		'''设置位置'''
		self.x = x
		self.y = y
		self.z = z
	
	def get_position(self):
		'''获取位置'''
		return [self.x, self.y, self.z]

	def set_euler_angle(self, roll, pitch, yaw):
		'''设置欧拉角'''
		# 赋值欧拉角
		self.roll = roll
		self.pitch = pitch
		self.yaw = yaw
		# 更新旋转矩阵
		self.rmat = Transform.euler2rmat(\
	  		roll=self.roll, pitch=self.pitch, yaw=self.yaw)
	
	def get_euler_angle(self):
		'''获取欧拉角'''
		return [self.roll, self.pitch, self.yaw]
	
	def set_rotation_matrix(self, rmat):
		'''设置旋转矩阵'''
		self.rmat = np.copy(rmat)
		# 同步更新欧拉角
		self.roll, self.pitch, self.yaw = Transform.rmat2euler(rmat)[0]
	
	def get_rotation_matrix(self):
		'''获取旋转矩阵'''
		return Transform.euler2rmat(\
	  		roll=self.roll, pitch=self.pitch, yaw=self.yaw)
	
	def set_transform_matrix(self, tmat):
		'''设置变换矩阵'''
		x, y, z = tmat[:3, 3].reshape(-1)
		self.set_position(x, y, z)
		rmat = tmat[:3, :3]
		self.set_rotation_matrix(rmat)
	
	def get_transform_matrix(self):
		'''获取变换矩阵'''
		tmat = np.float64(np.eye(4))
		tmat[0,3] = self.x
		tmat[1,3] = self.y
		tmat[2,3] = self.z
		tmat[:3, :3] = self.rmat
		return tmat
	
	def set_quaternion(self, q):
		'''设置四元数'''
		self.set_rotation_matrix(q.to_rmat())
	
	def get_quaternion(self):
		'''获取当前的四元数'''
		q = Quaternion()
		q.from_rmat(self.rmat)
		return q
		
	def distance(self, pose):
		'''返回笛卡尔空间下的距离'''
		x1, y1, z1 = self.get_position()
		x2, y2, z2 = pose.get_position()
		return math.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
	
	def from_bullet_pose(self, posi, q_xyzw, unit="m"):
		'''从Bullet位姿描述中构造Pose'''
		# 位置单位的转换
		if unit == "m":
			self.set_position(*[v*1000 for v in posi])
		elif unit == "mm":
			self.set_position(*posi)
		# 创建四元数对象
		q = Quaternion()
		q.from_xyzw(*q_xyzw)
		self.set_quaternion(q)

	def __str__(self):
		params = [self.x, self.y, self.z, np.degrees(self.roll), \
	  		np.degrees(self.pitch), np.degrees(self.yaw)]
		return "Pose x={:.3f}, y={:.3f}, z={:.3f}, roll={:.3f}, pitch={:.3f}, yaw={:.3f}]".format(*params)
