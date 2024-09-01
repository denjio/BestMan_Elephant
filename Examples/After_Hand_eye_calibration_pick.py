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

    # 将滚转、俯仰、偏航角转换为旋转矩阵
    rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_matrix()

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
u, v= 210, 120 # 假设中心点
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
[240.712679,9.248801,212.482240,-172.830392,5.296393,75.100849],
[335.795279,-21.471282,225.889988,-175.640753,5.436961,74.837881],
[378.083953,-35.134275,226.980209,-177.597721,5.527260,74.651056],
[425.343435,-50.403095,195.770261,-176.239918,5.465218,74.781119],
[412.554531,-119.542458,195.539676,-176.710241,-1.576748,71.953654],
[359.200694,-103.622610,238.091197,-179.039080,1.580000,70.701836],
[323.969766,-76.018262,240.532579,-178.202566,1.483139,69.975320],
[323.987944,-76.029838,240.543581,-178.454504,1.742036,61.125765],
[292.524264,-59.120198,234.492561,-176.952303,1.757542,61.171618],
[293.878582,-59.848014,202.971447,-170.482091,0.257614,70.691542],
[348.796888,-89.362845,220.694458,-175.671029,1.072112,70.630979],
[392.877077,-113.052851,212.610265,-177.141573,1.301605,70.600573],
[444.974773,-142.783362,209.703691,178.910287,3.564923,69.847746],
[356.301430,124.595652,239.140334,-178.962464,-5.269219,68.445625],
[302.497164,14.873241,226.601324,-171.473388,-7.233345,63.601711],
[369.354653,73.148309,238.939037,178.605093,-1.032620,85.253348],
[375.511970,-15.940784,238.969151,178.373988,2.035370,69.121279],
[375.512559,-15.931280,238.969529,177.960103,1.620950,56.211062],
# [372.494568,-4.742098,229.503567,-174.798854,-8.342979,70.693422],
[302.475687,14.879355,226.596277,-172.377703,-8.182523,70.344853],
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
T_cam2arm = np.linalg.inv(T_arm2cam)
point_base = np.dot(T_cam2arm, P_camera)
print('point_base', point_base)
# T_arm2cam = np.array([
#     [   0.7647718 ,    0.33233856  , -0.55197388, -125.11242469],
#  [  -0.45201427 ,   0.88724616 ,  -0.09207257 ,-421.6110077 ],
#  [   0.45913744 ,  0.31991458 ,   0.82876262 , 212.9457449 ],
#  [   0.     ,       0.    ,       0.    ,        1.        ]])
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
bestman.set_arm_coords([300,29,226.544752,-171.458040,-7.242349,63.614122], speed=500)
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
