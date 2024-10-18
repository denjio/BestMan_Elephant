'''
Author: hyuRen
Date: 2024-10-09 09:39:00
LastEditors: hyuRen
LastEditTime: 2024-10-11 21:02:31
'''
import pyrealsense2 as rs
import numpy as np
import cv2
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import sys
import os
sys.path.append(os.getcwd())
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
from RoboticsToolBox.Bestman_Elephant import Bestman_Real_Elephant
# 实例化Bestman_Real_Elephant对象
bestman1 = Bestman_Real_Elephant("192.168.43.38", 5001)
bestman2 = Bestman_Real_Elephant("192.168.43.243", 5001)
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

def get_base_coordinate_arm(T_Cam2Robot_arm, u, v, z):
    #  提取内参矩阵(中的参数
    mtx = np.array(
    [[644.7208252,    0.,         631.20788574],
    [  0. ,        643.23901367, 372.19293213],
    [  0.,           0.,           1.        ]])
    fx = mtx[0, 0]
    fy = mtx[1, 1]
    cx = mtx[0, 2]
    cy = mtx[1, 2]
    # print(mtx)
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    Z = z
    P_camera = np.array([X, Y, Z, 1.0])
    point_base = np.dot(T_Cam2Robot_arm, P_camera)
    return point_base[0], point_base[1], point_base[2]


# # 机械臂使能，执行一次即可
# bestman.state_on()
# bestman2.state_on()
 
# 初始化 RealSense 流程
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

# 创建对齐对象 (将深度图像对齐到颜色图像)
align_to = rs.stream.color
align = rs.align(align_to)

# 存储点击的坐标
click_points = []

# 操作类型
GRAB = 0
RELEASE = 1
turn = 0
operation = GRAB

# 创建进程池， 并发控制两个机械臂
executor = ProcessPoolExecutor(max_workers=2)

# 移动机械臂到指定位置抓取
def move_arm(arm:Bestman_Real_Elephant, x_base, y_base, z_base, operation):
    print(f"{arm}-operation:{operation}-moveing to coordinates:({x_base, y_base, z_base})")
    if operation == GRAB:
        arm.set_arm_coords([x_base, y_base, 300, 175, 0, 120], speed=800)
        arm.open_gripper(open_scale=50)
        arm.set_arm_coords([x_base, y_base, 165, 175, 0, 120], speed=800)
        arm.close_gripper()
        arm.set_arm_coords([x_base, y_base, 300, 175, 0, 120], speed=800)
    else:
        arm.set_arm_coords([x_base, y_base, 300, 175, 0, 120], speed=800)
        arm.set_arm_coords([x_base, y_base, 180, 175, 0, 120], speed=800)
        arm.open_gripper(open_scale=50)
        arm.set_arm_coords([x_base, y_base, 300, 175, 0, 120], speed=800)
        arm.close_gripper()


# 鼠标回调函数
# 保持turn逻辑不变，但直接让两个机械臂一起执行
def get_mouse_click(event, x, y, flags, param):
    global click_points, operation, turn
    if event == cv2.EVENT_LBUTTONDOWN:
        aligned_depth_frame, color_image = param
        
        # 获取点击位置的深度值 z
        z = aligned_depth_frame.get_distance(x, y) * 1000  # 将 z 值从米转换为毫米
        
        print(f"点击位置 (u, v): ({x}, {y}), 深度值 z: {z} 毫米")
        # 图像坐标系转换到机械臂坐标系
        if turn == 0:
            x_base, y_base, z_base = get_base_coordinate_arm(T_Cam2Robot_arm1, x, y, z)
            turn = 1
        else:
            x_base, y_base, z_base = get_base_coordinate_arm(T_Cam2Robot_arm2, x, y, z)
            turn = 0
        click_points.append((x_base, y_base, z_base))

        # 一旦有两个点击点，启动两个机械臂的任务
        if len(click_points) == 2:
            # 使用并发执行两个机械臂动作
            executor.submit(move_arm, bestman1, *click_points[0], operation)
            executor.submit(move_arm, bestman2, *click_points[1], operation)
            
            # 操作切换
            if operation == GRAB:
                operation = RELEASE
            else:
                operation = GRAB

            # 清空点击点以准备下次输入
            click_points.clear()

            

if __name__ == '__main__':
    try:
        while True:
            # 获取一帧数据
            frames = pipeline.wait_for_frames()

            # 对齐深度帧到颜色帧
            aligned_frames = align.process(frames)

            # 获取对齐后的深度帧和颜色帧
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                continue

            # 将颜色帧转换为 NumPy 数组 
            color_image = np.asanyarray(color_frame.get_data())

            # 显示颜色图像
            cv2.imshow('Aligned RGB Image', color_image)

            # 设置鼠标回调函数
            cv2.setMouseCallback('Aligned RGB Image', get_mouse_click, (aligned_depth_frame, color_image))

            # 等待按键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()