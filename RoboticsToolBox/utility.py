""" 
# @Author: Youbin Yao 
# @Date: 2024-08-25 15:32:48
# @Last Modified by:   Youbin Yao 
# @Last Modified time: 2024-08-25 15:32:48  
""" 
from scipy.spatial.transform import Rotation as R
import numpy as np
from scipy.optimize import minimize

def pose_to_euler(pose):
    '''
    Convert robot pose from a list [x, y, z, qw, qx, qy, qz] to [x, y, z] and Euler angles.
    
    Parameters:
    pose: list of 7 floats - [x, y, z, qw, qx, qy, qz]
    
    Returns:
    tuple: (x, y, z, roll, pitch, yaw) where (x, y, z) is the position and (roll, pitch, yaw) are the Euler angles in radians.
    '''
    x, y, z, qw, qx, qy, qz = pose
    r = R.from_quat([qx, qy, qz, qw])  # Reordering to match scipy's [qx, qy, qz, qw]
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return [x, y, z, roll, pitch, yaw]

def euler_to_pose(position_euler):
    '''
    Convert robot pose from [x, y, z, roll, pitch, yaw] to [x, y, z, qw, qx, qy, qz].
    
    Parameters:
    position_euler: list of 6 floats - [x, y, z, roll, pitch, yaw]
    
    Returns:
    list: [x, y, z, qw, qx, qy, qz]
    '''
    x, y, z, roll, pitch, yaw = position_euler
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    qx, qy, qz, qw = r.as_quat()  # Getting [qx, qy, qz, qw] from scipy
    return [x, y, z, qw, qx, qy, qz]  # Reordering to match [qw, qx, qy, qz]

# 定义正运动学模型 (用于数值逆运动学求解)
def forward_kinematics(joint_angles):
    # 这里你需要实现你机器人特定的正运动学方程
    # 返回一个 [x, y, z, roll, pitch, yaw] 的列表
    # 这只是一个示例，实际需要根据你的机器人参数来定义
    x, y, z = 0.0, 0.0, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.0
    return [x, y, z, roll, pitch, yaw]

# 定义目标函数 (最小化末端执行器位置与目标位置的差异)
def objective_function(joint_angles, target_pose):
    fk_pose = forward_kinematics(joint_angles)
    error = np.linalg.norm(np.array(fk_pose) - np.array(target_pose))
    return error

# 逆运动学函数
def inverse_kinematics(target_pose, initial_guess):
    result = minimize(objective_function, initial_guess, args=(target_pose,), method='BFGS')
    return result.x  # 返回求解的关节角度

# # 示例目标姿态 [x, y, z, roll, pitch, yaw]
# target_pose = [0.5, 0.3, 0.2, np.pi/4, np.pi/6, np.pi/3]

# # 初始关节角度猜测 (用于迭代求解)
# initial_guess = [0, 0, 0, 0, 0, 0]

# # 求解逆运动学
# joint_angles = inverse_kinematics(target_pose, initial_guess)

# print("Calculated joint angles:", joint_angles)
