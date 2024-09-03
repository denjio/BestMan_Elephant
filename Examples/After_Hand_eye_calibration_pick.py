""" 
# @Author: Youbin Yao 
# @Date: 2024-08-27 20:45:49
# @Last Modified by:   Youbin Yao 
# @Last Modified time: 2024-08-27 20:45:49  
""" 
import sys
import os
sys.path.append(os.getcwd())
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
sys.path.append(os.path.join(parent_dir, 'Visualization'))
import numpy as np
import cv2
import json
from RoboticsToolBox import Bestman_Real_Elephant
from Visualization import calibration_eye_to_hand, Transform
from urdf_parser_py.urdf import URDF
from scipy.spatial.transform import Rotation as R
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

# 示例：将单个末端执行器姿态转换为变换矩阵
def pose_to_transform(pose):
    # 提取位置和方向
    x, y, z = pose[:3]
    roll, pitch, yaw = pose[3:]
    euler_angles_deg = pose[3:6]
    # 将欧拉角从度转换为弧度
    euler_angles_rad = np.radians(euler_angles_deg)
    # 将滚转、俯仰、偏航角转换为旋转矩阵
    # rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()
     # 将滚转、俯仰、偏航角转换为旋转矩阵
    rotation_matrix = R.from_euler('xyz', euler_angles_rad, degrees=True).as_matrix()
    # 将旋转矩阵和位置向量组合成齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = [x, y, z]
    
    return T
    
# 读取 JSON 文件中的相机参数
json_file_path = '/home/robot/Desktop/BestMan_Elephant/camera_params.json'
with open(json_file_path, 'r') as file:
    camera_params = json.load(file)

# 提取相机参数
mtx = np.array(camera_params['mtx']).reshape((3, 3))
dist = np.array(camera_params['dist'])
rvecs = np.array(camera_params['rvecs'])
tvecs = np.array(camera_params['tvecs'])

print(rvecs.shape, tvecs.shape)
# 假设图像中目标点的像素坐标 (u, v) 和深度 Z
u, v= 0, 0 # 假设中心点
Z = 0.659 # 假设深度为 2 米

# 去畸变并将图像坐标转换为摄像机坐标
undistorted_points = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), mtx, dist, None, mtx)
print(undistorted_points)
X = (undistorted_points[0][0][0] ) * Z 
Y = (undistorted_points[0][0][1] ) * Z 
P_camera = np.array([X, Y, Z, 1.0])
print(P_camera)

T_arm2wrist_list = []
# 加载URDF文件
robot = URDF.from_xml_file('/home/robot/Desktop/BestMan_Elephant/Asset/mycobot_pro_630.urdf')
# print(robot)
end_effort_list=[
[150.406863,-300.654075,382.096107,-177.676689,5.572467,159.336867],
[150.407462,-300.653712,382.095800,-179.283289,5.993681,143.497378],
[150.407348,-300.653822,382.096137,-175.566427,4.100455,-176.019982],
[128.392039,-185.521140,435.728308,173.265773,-0.353037,156.213436],
[125.047732,-168.030146,415.873348,161.583123,-8.294159,157.099148],
# [133.546688,-186.049137,438.382623,170.061507,-0.427645,124.413527],
# [132.659960,-223.343589,278.419321,-178.480797,4.358319,124.792952],
[134.146132,-223.056410,278.451196,-177.385015,4.288113,155.949345],
[134.159191,-223.054450,278.452652,-175.915139,2.933319,179.056362],
[132.552309,-201.377201,180.220957,172.661602,0.983511,158.620444],
[91.416854,-467.880907,64.410011,175.428578,-0.325647,148.219759],
[91.416876,-467.881368,64.406842,175.639343,1.412869,170.204870],
[163.067205,-367.055664,403.499433,176.898837,6.507234,-175.940183],
[204.131855,-360.312454,380.283649,171.325642,1.815091,157.948987],
[194.868783,-328.787527,378.754947,170.574362,0.770461,154.969308],
[189.400143,-310.172666,361.129625,159.002388,-9.297069,155.836381],
[269.138800,-239.861041,370.468033,175.072492,8.191020,-172.164264],
[271.783917,-238.197987,371.106780,168.719349,6.471380,156.765850],
# [-41.021715,-285.111151,307.367087,175.565275,12.530018,144.698381],
# [-114.802253,-264.180277,307.369436,175.565762,12.530002,129.397922],
]
# 将 end_effort_list 中的所有姿态转换为变换矩阵
T_arm2wrist_list = [pose_to_transform(pose) for pose in end_effort_list]

# 创建列表存储T_cam2board
T_cam2board_list = []
# 遍历每个视角的rvecs和tvecs
for rvec, tvec in zip(rvecs, tvecs):
    # 将旋转向量转换为旋转矩阵
    R, _ = cv2.Rodrigues(rvec)
    
     # 将tvec转换为列向量
    tvec = tvec.reshape(-1, 1)
    # 构建齐次变换矩阵
    T_cam2board = np.hstack((R, tvec))
  
    T_cam2board = np.vstack((T_cam2board, [0, 0, 0, 1]))
    
    # 添加到列表
    T_cam2board_list.append(T_cam2board)
# print(T_arm2wrist_list, T_cam2board_list)

T_arm2cam = calibration_eye_to_hand(T_arm2wrist_list, T_cam2board_list)
 
# 计算点在机械臂基座坐标系下的位置
point_base = np.dot(T_arm2cam, P_camera)
print('point_base', point_base)
# T_cam2arm = np.linalg.inv(T_arm2cam)
# point_base = np.dot(T_cam2arm, P_camera)
# print('point_base', point_base)
 
# # 计算点在机械臂基座坐标系下的位置
# point_base = np.dot(T_arm2cam, P_camera)
# print(point_base)
x_base, y_base, z_base = point_base[:3]

print(f"目标点在机械臂基座坐标系下的位置: ({x_base}, {y_base}, {z_base})")
 
bestman = Bestman_Real_Elephant("192.168.43.38", 5001)
# bestman.state_on()
bestman.get_current_cartesian()
bestman.get_current_joint_values()
# 定义垂直向下的欧拉角
bestman.set_arm_coords([300.731933,y_base,307.369918,175.565980,12.529990,-172.698050], speed=500)
# bestman.set_arm_joint_values([0.0, -120.0, 120.0, -90.0, -90.0, -0.0],speed=500)

bestman.get_current_cartesian()
bestman.get_current_joint_values()
# bestman.set_arm_coords([x_base, 124, 350.841710, 0, -90, 0], speed=200)
# # 发布目标位置到机器人
# pose_pub = rospy.Publisher('/robot_arm/goal_pose', Pose, queue_size=10)
# goal_pose = Pose()
# goal_pose.position.x = P_base[0]
# goal_pose.position.y = P_base[1]
# goal_pose.position.z = P_base[2]
# pose_pub.publish(goal_pose)
