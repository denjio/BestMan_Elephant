# !/usr/bin/env python
# -*- encoding: utf-8 -*-
""" 
# @Author: Youbin Yao 
# @Date: 2024-08-21 19:59:22
# @Last Modified by:   Youbin Yao 
# @Last Modified time: 2024-08-21 19:59:22  
""" 
from pymycobot import ElephantRobot
import time
import sys
import os
from scipy.spatial.transform import Rotation as R
from utility import *

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
print(os.path.join(parent_dir, 'RoboticsToolBox'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
class Bestman_Real_Elephant:
    def __init__(self, robot_ip, frequency=10):
        # Initialize the robot and gripper with the provided IPs and frequency
        self.robot = ElephantRobot(robot_ip, 5001)
        # Necessary instructions to start the robot
        self.robot.start_client()
        self.gripper = None
        self.frequency = frequency
         
    # ----------------------------------------------------------------
    # functions for switch
    # ----------------------------------------------------------------
    def state_on(self,):
        # Turn off robot enable first
        self.robot.state_off()
        time.sleep(3)
        # Power up the robot
        self.robot.power_on()
        time.sleep(3)
        # Enabling robots 
        self.robot.state_on()
        time.sleep(3)

    def state_off(self,):
        self.robot.state_off()
        time.sleep(3)

    def power_on(self,):
        self.robot.power_on()
        time.sleep(3)
       
    def power_off(self,):
        self.robot.state_off()
        self.robot.power_off()
        time.sleep(3)
    
    
    # ----------------------------------------------------------------
    # functions for arm
    # ----------------------------------------------------------------
    # def sim_get_arm_id(self):
    #     return self.arm_id

    # def get_DOF(self):
    #     self.DOF = 6
    #     return self.DOF

    # def sim_get_arm_joint_idx(self):
    #     return self.arm_joints_idx

    # def sim_get_arm_all_joint_idx(self):
    #     return list(range(p.getNumJoints(self.arm_id, physicsClientId=self.client_id)))

    # def sim_get_tcp_link(self):
    #     return self.tcp_link

    # def sim_get_tcp_link_height(self):
    #     return self.tcp_height

    # def sim_get_end_effector_link(self):
    #     return self.end_effector_index
    
    # def sim_get_arm_all_jointInfo(self):
    #     return  
    
    # def sim_get_joint_bounds(self):
    #     return joint_bounds

    def check_state(self):
        return self.robot.state_check()
    
    def check_running(self):
        return self.robot.check_running()
    
    def check_error(self):
        # 功能：机器人错误检测
        return self.robot.read_next_error()

        
    def get_current_joint_values(self):
        return self.robot.get_angles()
    
    def get_current_cartesian(self):
        return self.robot.get_coords()
    
    # def sim_get_current_end_effector_pose(self):
    #     return Pose(end_effector_info[0], end_effector_info[1])

    def set_single_joint_value(self, joint=None, joint_value=None, speed=500):
        self.robot.write_angle(joint, joint_value, speed=500)
        self.robot.command_wait_done()
 
    # def sim_debug_set_arm_to_joint_values(self):
    #     pass

    def set_arm_joint_values(self, joint_values=None, speed=500):
        self.robot.write_angles(joint_values, speed=500)
        self.robot.command_wait_done()
    
    def move_arm_follow_target_trajectory(self, target_trajectory,trajectory_type='euler', target_vel=None, target_acc=None, MAX_VEL=None, MAX_ACC=None):
        '''
        Move arm to a few set of joint angles, considering physics.

        Args:
            target_trajectory: A list of desired joint angles (in radians) for each joint of the arm.
            trajectory: 'pose' or 'euler'
            target_vel: Optional. A list of target velocities for each joint.
            target_acc: Optional. A list of target accelerations for each joint.
            MAX_VEL: Optional. A list of maximum velocities for each joint.
            MAX_ACC: Optional. A list of maximum accelerations for each joint.
        '''
        period = 1.0 / self.frequency
        # DOF = len(self.robot_states.q)
        # if target_vel is None:
        #     target_vel = [0.0] * DOF
        # if target_acc is None:
        #     target_acc = [0.0] * DOF
        # if MAX_VEL is None:
        #     MAX_VEL = [3.0] * DOF
        # if MAX_ACC is None:
        #     MAX_ACC = [1.0] * DOF
        
        for target_pos in target_trajectory:
            
            # Monitor fault on robot server
            # if self.robot.isFault():
            #     raise Exception("Fault occurred on robot server, exiting ...")
            if trajectory_type == 'eurler':
                target_pos = target_pos
            elif trajectory_type == 'pose':
                target_pos = pose_to_euler(target_pos)
            
            # 初始关节角度猜测 (用于迭代求解)
            initial_guess = self.get_current_joint_values()
            # 求解逆运动学
            joint_angles = inverse_kinematics(target_pos, initial_guess)
            print("Calculated joint angles:", joint_angles)
            # Send command
            self.set_arm_joint_values(joint_angles)
    
            # Use sleep to control loop period
            time.sleep(period)
 
    def set_single_coord(self, axis, value, speed):
        # 功能：发送单个坐标值给机械臂进行移动
        # 参数：机器人笛卡尔位置[0代表x,1代表y,2代表z,3代表rx,4代表ry,5代表rz]，要到达的坐标值，机械臂运动的速度:[0-6000]
        self.robot.write_coord(axis, value, speed=1000)

    def set_arm_coords(self,coords,speed):
        # 功能：发送所有角度给机械臂所有关节
        # 参数：关节角度(列表类型)，机械臂运动的速度:[0-5999]
        self.robot.write_coords(coords,speed=1000)

    def set_jog_angel(self,joint_str, direction, ):
        # 功能： 控制机器人按照指定的角度持续移动
        # 参数：机械臂的关节[J1/J2/J3/J4/J5/J6]，主要控制机器臂移动的方向[-1=负方向 ，0=停止，1=正方向]，机器人运动的速度
        self.robot.jog_angle(joint_str, direction, speed=1)
        self.robot.command_wait_done()
    
    def set_jog_coord(self, axis_str, direction):
        # 功能： 控制机器人按照指定的坐标轴方向持续移动
        # 参数：笛卡尔的方向[x/y/z/rx/ry/rz],主要控制机器臂移动的方向[-1=负方向 ，0=停止，1=正方向],机器人运动的速度
        self.robot.jog_coord(axis_str, direction, speed=1)
        self.robot.command_wait_done()

    def set_jog_relative(self, joint_id, angle, speed, mode):
        # 功能：以当前位置往某个坐标轴方向进行相对运动，或是以当前关节角度往某个关节的角度进行相对运动
        # 参数：相对运动的方向或角度['J1'——'J6', 'X', 'Y', 'Z', 'RX', 'RY', 'RZ'],相对移动的距离或角度,移动速度,运动模式[0 或 1 ]
        self.robot.jog_relative(joint_id, angle, speed, mode)
        self.robot.command_wait_done()

    # def joints_to_cartesian(self, joint_values):
    #     return Pose(position, orientation)

    # def cartesian_to_joints(self, pose, max_iterations=1000, threshold=1e-4):
    #     return joint_values

    # def sim_rotate_end_effector(self, angle):
    #     print("[BestMan_Sim][Arm] \033[34mInfo\033[0m: Rotate end effector completed!")

    # def sim_move_end_effector_to_goal_pose(
    #     print("[BestMan_Sim][Arm] \033[34mInfo\033[0m: Move end effector to goal pose finished!")

    # def sim_execute_trajectory(self, trajectory, threshold=0.1, enable_plot=False):
    #     print("[BestMan_Sim][Arm] \033[34mInfo\033[0m: Excite trajectory finished!")

    # def sim_calculate_IK_error(self, goal_pose):
    #     return distance

    
    # ----------------------------------------------------------------
    # Functions for gripper
    # ----------------------------------------------------------------
    def open_gripper(self, speed=100, open_scale=None):
        """open gripper
        Args:
            state (int): open_scale, 0-100
            speed (int): speed, 1-100

        """
        # Gripper settings transparent transmission mode
        self.robot.set_gripper_mode(0)
        time.sleep(1)
        if open_scale == None:
            # gripper fully open
            self.robot.set_gripper_state(0,speed)
            time.sleep(1)
        else:
            # gripper opening specified position
            self.robot.set_gripper_value(open_scale, speed)
        self.robot.command_wait_done()
        print(
                "[BestMan_Sim][Gripper] \033[34mInfo\033[0m: Gripper open!"
            )

    def close_gripper(self, speed=100, close_scale=None):
        """open gripper
        Args:
            state (int): open_scale, 0-100
            speed (int): speed, 1-100

        """
        # Gripper settings transparent transmission mode
        self.robot.set_gripper_mode(0)
        time.sleep(1)
        if close_scale == None:
            # gripper fully open
            self.robot.set_gripper_state(1,speed)
            time.sleep(1)
        else:
            # gripper opening specified position
            self.robot.set_gripper_value(close_scale, speed)
        self.robot.command_wait_done()
        print(
                "[BestMan_Sim][Gripper] \033[34mInfo\033[0m: Gripper close!"
            )

# if __name__=='__main__':
#     bestman = Bestman_Real_Elephant(robot_ip='192.168.43.38')
#     bestman.power_off()
#     # bestman.move_arm_to_joint_values(([0.0, -120.0, 120.0, -90.0, -90.0, -0.0]), 1000) # ([0,-90,0,-90,0,0]) ([0.0, -120.0, 120.0, -90.0, -90.0, -0.0])
#     # bestman.close_gripper(20)
#     # bestman.open_gripper()
#     # bestman.close_gripper()
