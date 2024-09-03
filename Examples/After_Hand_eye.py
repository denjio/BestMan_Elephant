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
u, v= 300, 619 # 假设中心点
Z = 0.750 # 深度
# 去畸变
undistorted_points = cv2.undistortPoints(np.array([[u, v]], dtype=np.float32), mtx, dist, None, mtx)
# 将图像坐标转换为摄像机坐标
# 给定像素坐标 (u, v) 和深度值 Z 计算相机坐标系下的三维坐标
X = (undistorted_points[0][0][0] ) * Z 
Y = (undistorted_points[0][0][1] ) * Z 
P_camera = np.array([X, Y, Z, 1.0])
print(P_camera)

# 加载URDF文件
robot = URDF.from_xml_file('/home/robot/Desktop/BestMan_Elephant/Asset/mycobot_pro_630.urdf')

end_effort_list=[
[158.275733,-268.295108,326.504688,173.855638,6.861038,155.282662],
[-23.999754,-344.573772,325.315364,177.128403,8.868653,152.056668],
[-24.000348,-344.572734,325.313939,176.693728,8.717101,149.215357],
[322.357460,-347.636103,324.561948,175.321800,8.197706,167.744456],
[322.357059,-347.634370,324.570380,177.044301,8.959944,179.248145],
[377.873125,-115.163099,299.938397,167.884756,10.492300,147.208553],
[377.872409,-115.162643,299.939212,166.916067,9.248039,141.562652],
[108.406980,-98.529370,327.939086,171.782615,5.571919,129.559933],
]
# 将 end_effort_list 中的所有姿态转换为变换矩阵
R_gripper2base_list = []
t_gripper2base_list = []

for pose in end_effort_list:
    xyz = pose[0:3]
    euler_angles_deg = pose[3:6]
    
    # 将欧拉角从度转换为弧度
    euler_angles_rad = np.radians(euler_angles_deg)
    
    # 使用scipy将欧拉角转换为旋转矩阵
    rotation_matrix = R.from_euler('xyz', euler_angles_rad).as_matrix()
    
    # 将xyz作为平移向量
    translation_vector = np.array(xyz)
    
    # 添加到列表
    R_gripper2base_list.append(rotation_matrix)
    t_gripper2base_list.append(translation_vector)
# print(R_gripper2base_list, t_gripper2base_list)

# 遍历每个视角的rvecs和tvecs
R_target2cam_list = []
t_target2cam_list = []

for i in range(len(rvecs)):
    # 将旋转向量转换为旋转矩阵
    R_cam2target, _ = cv2.Rodrigues(rvecs[i])
    
    # # 计算标定板相对于相机的姿态
    R_target2cam = R_cam2target
    t_target2cam = tvecs[i]
    R_target2cam_list.append(R_target2cam)
    t_target2cam_list.append(t_target2cam)
R_base2gripper = []
t_base2gripper = []
R_cam2target = []
t_cam2target = []
# print(R_target2cam_list, t_target2cam_list)
 # 开始为各个参数赋值
for i in range(len(rvecs)):
    # 处理R_base2gripper和t_base2gripper
    pose = end_effort_list[i]
    xyz = pose[0:3]
    euler_angles_deg = pose[3:6]
    
    # 将欧拉角从度转换为弧度
    euler_angles_rad = np.radians(euler_angles_deg)
    
    # 使用scipy将欧拉角转换为旋转矩阵
    rotation_matrix = R.from_euler('xyz', euler_angles_rad).as_matrix()
    
    # 将xyz作为平移向量
    translation_vector = np.array(xyz)
    # 创建新的4*4的矩阵用来存放R和T拼接矩阵
    rtarray = np.zeros((4, 4), np.double)
    # 旋转向量转旋转矩阵,dst是是旋转矩阵,jacobian是雅可比矩阵
    dst, jacobian = cv2.Rodrigues(euler_angles_rad)
    # 存入旋转矩阵
    rtarray[:3, :3] = dst
    # 传入平移向量
    rtarray[:3, 3] = xyz 
    rtarray[3, 3] = 1
    # 求逆矩阵
    rb2e = np.linalg.inv(rtarray)
    # 放入用来传给calibrateHandEye的参数中
    R_base2gripper.append(rb2e[:3, :3].T)
    t_base2gripper.append(np.array(xyz))
     
    # 处理R_target2cam和t_target2cam
    # 获标定板在相机坐标系下的旋转矩阵,把旋转向量转成旋转矩阵
    dst1, jacobian1 = cv2.Rodrigues(rvecs[i])
    # 相机坐标系旋转矩阵转置
    R_cam2target.append(dst1.T)
    # 写入相机坐标系平移向量,平移向量需要乘原本的负的旋转矩阵
    t_cam2target.append(np.matmul(-dst1.T, tvecs[i]))
# 使用calibrateHandEye进行手眼标定
R_cam2base, t_cam2base = cv2.calibrateHandEye(
    R_gripper2base_list, t_gripper2base_list,
    R_target2cam_list, t_target2cam_list, method=cv2.CALIB_HAND_EYE_TSAI
)
# print('R_cam2base', R_cam2base,'t_cam2base', t_cam2base)
# 构造齐次变换矩阵 T_cam2base
T_cam2base = np.eye(4)
T_cam2base[:3, :3] = R_cam2base
T_cam2base[:3, 3] = t_cam2base.flatten()
# 计算点在机械臂基座坐标系下的位置, 将相机坐标系下的点转换到基座坐标系

# print(T_cam2base)
point_base = np.dot(T_cam2base, P_camera)
print('point_base', point_base)
T_base2cam = np.linalg.inv(T_cam2base)
# point_base = np.dot(T_base2cam, P_camera)
# print('point_base', point_base)
# 计算点在机械臂基座坐标系下的位置
x_base, y_base, z_base = point_base[0], point_base[1],point_base[2]

print(f"目标点在机械臂基座坐标系下的位置: ({x_base}, {y_base}, {z_base})")
bestman = Bestman_Real_Elephant("192.168.43.38", 5001)
# bestman.state_on()
bestman.get_current_cartesian()
bestman.get_current_joint_values()
# 定义垂直向下的欧拉角
bestman.set_arm_coords([187,-170.029450,364.804202,179.288396,1.539904,133.136652],  speed=500)
# bestman.set_arm_joint_values([0.0, -120.0, 120.0, -90.0, -90.0, -0.0],speed=500)

bestman.get_current_cartesian()
bestman.get_current_joint_values()
# 将基座坐标系中的点转换回相机坐标系
P_cam = np.dot(T_cam2base, [0,0,0,1])
print(P_cam) 
#结果验证，原则上来说，每次结果相差较小
for i in range(len(tvecs)):

    RT_end_to_base=np.column_stack((R_gripper2base_list[i],t_gripper2base_list[i]))
    # print('RT_end_to_base', RT_end_to_base)
    RT_end_to_base=np.row_stack((RT_end_to_base,np.array([0,0,0,1])))
    

    RT_chess_to_cam=np.column_stack((R_target2cam_list[i],t_target2cam_list[i]))
    # print('RT_chess_to_cam', RT_chess_to_cam)
    RT_chess_to_cam=np.row_stack((RT_chess_to_cam,np.array([0,0,0,1])))
    # print(RT_chess_to_cam)

    RT_cam_to_end=np.column_stack((R_cam2base,t_cam2base))
    RT_cam_to_end=np.row_stack((RT_cam_to_end,np.array([0,0,0,1])))
    # print(RT_cam_to_end)

    RT_chess_to_base=RT_end_to_base@RT_cam_to_end@RT_chess_to_cam#即为固定的棋盘格相对于机器人基坐标系位姿
    RT_chess_to_base=np.linalg.inv(RT_chess_to_base)
    print('第',i,'次')
    print(RT_chess_to_base[:3,:])
    print('')
 