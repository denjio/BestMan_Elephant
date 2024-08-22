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
    def __init__(self, robot_ip, frequency=1):
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
    
    def get_current_joint_values(self):
        return self.robot.get_angles()
    
    def get_current_cartesian(self):
        return self.robot.get_coords()
    # def sim_get_current_end_effector_pose(self):
    #     return Pose(end_effector_info[0], end_effector_info[1])

    def set_arm_to_joint_value(self, joint=None, joint_value=None, speed=500):
        self.robot.write_angle(joint, joint_value, speed=500)
        
 
    # def sim_debug_set_arm_to_joint_values(self):
    #     pass

    def move_arm_to_joint_values(self, joint_values=None, speed=500):
        self.robot.write_angles(joint_values, speed=500)

    def joints_to_cartesian(self, joint_values):
        return Pose(position, orientation)

    def cartesian_to_joints(self, pose, max_iterations=1000, threshold=1e-4):
        return joint_values

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
        print(
                "[BestMan_Sim][Gripper] \033[34mInfo\033[0m: Gripper close!"
            )

if __name__=='__main__':
    bestman = Bestman_Real_Elephant(robot_ip='192.168.113.130')
    # bestman.state_on()
    bestman.get_current_cartesian()
    bestman.get_current_joint_values()
    bestman.check_running()
    bestman.check_state()
    