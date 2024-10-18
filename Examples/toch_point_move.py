""" 
# @Author: Youbin Yao 
# @Date: 2024-09-01 13:24:30
# @Last Modified by:   Youbin Yao 
# @Last Modified time: 2024-09-01 13:24:30  
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
 
# 读取 JSON 文件中的相机参数
json_file_path = '/home/robot/Desktop/BestMan_Elephant/camera_params.json'
with open(json_file_path, 'r') as file:
    camera_params = json.load(file)

# 提取相机参数
mtx = np.array(camera_params['mtx']).reshape((3, 3))
dist = np.array(camera_params['dist'])
rvecs = np.array(camera_params['rvecs'])
tvecs = np.array(camera_params['tvecs'])

# 假设图像中目标点的像素坐标 (u, v) 和深度 Z
u, v= 620, 435 # 假设中心点
Z = 761 # 深度
 
# 提取内参矩阵(中的参数
mtx = np.array(
[[644.7208252,    0.,         631.20788574],
 [  0. ,        643.23901367, 372.19293213],
 [  0.,           0.,           1.        ]])
fx = mtx[0, 0]
fy = mtx[1, 1]
cx = mtx[0, 2]
cy = mtx[1, 2]
print(mtx)
# 将图像坐标 (u, v, z) 转换为相机坐标 (X, Y, Z)
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = Z
P_camera = np.array([X, Y, Z, 1.0])
print(P_camera)
 # arm 1
T_Cam2Robot_arm1 =np.array(
[[-9.99810248e-01,  5.95333441e-03, -1.85479259e-02,  2.41222153e+02],
 [ 4.60860048e-03,  9.97414322e-01,  7.17177231e-02, -2.68426413e+02],
 [ 1.89269265e-02,  7.16186346e-02, -9.97252497e-01,  8.90397554e+02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
 
# arm 2
T_Cam2Robot_arm2 =np.array(
[[ 9.99517264e-01, -3.00053157e-02, -8.05725336e-03,  2.32466261e+02],
 [-3.03055063e-02, -9.98733683e-01, -4.01572709e-02, -3.05379448e+02],
 [-6.84211874e-03,  4.03820647e-02, -9.99160885e-01,  8.85636510e+02],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
point_base = np.dot(T_Cam2Robot_arm1, P_camera)
x_base, y_base, z_base = point_base[0], point_base[1], point_base[2]

print(f"目标点在机械臂基座坐标系下的位置: ({x_base}, {y_base}, {z_base})")
bestman = Bestman_Real_Elephant("192.168.43.243", 5001)
# bestman.state_on()
bestman.get_current_cartesian()
bestman.get_current_joint_values()
# 定义垂直向下的欧拉角
# bestman.set_arm_joint_values([0.0, -120.0, 120.0, -90.0, -90.0, -0.0],speed=500)
bestman.set_arm_coords([x_base,y_base,180, 175,0,120],speed=800)
bestman.get_current_cartesian()
bestman.get_current_joint_values() 
 