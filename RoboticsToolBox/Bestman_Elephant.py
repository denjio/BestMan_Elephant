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


class Bestman_Real_Elephant:
    def __init__(self, robot_ip, frequency):
        # Initialize the robot and gripper with the provided IPs and frequency
        self.robot = ElephantRobot(robot_ip, 5001)
        self.gripper = None
        self.frequency = frequency
         

    # ----------------------------------------------------------------
    # functions for arm
    # ----------------------------------------------------------------

    def sim_get_arm_id(self):
        return self.arm_id

    def sim_get_DOF(self):
        return self.DOF

    def sim_get_arm_joint_idx(self):
        return self.arm_joints_idx

    def sim_get_arm_all_joint_idx(self):
        return list(range(p.getNumJoints(self.arm_id, physicsClientId=self.client_id)))

    def sim_get_tcp_link(self):
        return self.tcp_link

    def sim_get_tcp_link_height(self):
        return self.tcp_height

    def sim_get_end_effector_link(self):
        return self.end_effector_index
    
    def sim_get_arm_all_jointInfo(self):
        return arm_jointInfo
    
    def sim_get_joint_bounds(self):
        return joint_bounds

    def sim_get_current_joint_values(self):
        return current_joint_values

    def sim_get_current_end_effector_pose(self):
        return Pose(end_effector_info[0], end_effector_info[1])

    def sim_set_arm_to_joint_values(self, joint_values):
        self.client.run(10)

    def sim_debug_set_arm_to_joint_values(self):
        )

    def sim_move_arm_to_joint_values(self, joint_values, threshold=0.015, timeout=0.05):
        self.client.run(40)

    def sim_joints_to_cartesian(self, joint_values):
        return Pose(position, orientation)

    def sim_cartesian_to_joints(self, pose, max_iterations=1000, threshold=1e-4):
        return joint_values

    def sim_rotate_end_effector(self, angle):
        print("[BestMan_Sim][Arm] \033[34mInfo\033[0m: Rotate end effector completed!")

    def sim_move_end_effector_to_goal_pose(
        print("[BestMan_Sim][Arm] \033[34mInfo\033[0m: Move end effector to goal pose finished!")

    def sim_execute_trajectory(self, trajectory, threshold=0.1, enable_plot=False):
        print("[BestMan_Sim][Arm] \033[34mInfo\033[0m: Excite trajectory finished!")

    def sim_calculate_IK_error(self, goal_pose):
        return distance

    # ----------------------------------------------------------------
    # functions between base and arms
    # ----------------------------------------------------------------

    def sim_get_robot_size(self):
        return robot_size

    def sim_sync_base_arm_pose(self):
            )
