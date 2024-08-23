""" 
# @Author: Youbin Yao 
# @Date: 2024-08-23 11:31:25
# @Last Modified by:   Youbin Yao 
# @Last Modified time: 2024-08-23 11:31:25  
""" 
import os
import sys
sys.path.append(os.getcwd())

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))

from pymycobot import ElephantRobot
from RoboticsToolBox import Bestman_Real_Elephant


if __name__=='__main__':
    # 标准姿态 [0.0, -120.0, 120.0, -90.0, -90.0, -0.0]
    # 零位 [0,-90,0,-90,0,0]
    bestman = Bestman_Real_Elephant("192.168.43.38", 5001)
    bestman.state_on()
    bestman.open_gripper()
    bestman.set_arm_joint_values([0.0, -120.0, 120.0, -90.0, -90.0, -0.0])
    bestman.close_gripper()
    bestman.power_off()
    # for i in range(1):
    #     "机器人关节运动到安全点,设定关节角度"
    #     bestman.set_arm_joint_values([94.828,-143.513,135.283,-82.969,-87.257,-44.033])

    #     "机器人笛卡尔运动到码垛抓取过渡点,设定位姿"
    #     bestman.set_arm_coords([-130.824,256.262,321.533,176.891,-0.774,-128.700], 3000)

    #     "机器人以当前坐标位置往Z轴负方向整体运动100mm,到达抓取位置"
    #     bestman.set_jog_relative("Z",-100,1500,1)

    #     "控制夹爪闭合到30mm"
    #     bestman.close_gripper(30,100)

    #     "机器人以当前坐标位置往Z轴正方向整体运动100mm,到达抓取过渡点"
    #     bestman.set_jog_relative("Z",100,1500,1)

    #     "机器人以当前坐标位置往Y轴正方向整体运动300mm,到达放置过渡点"
    #     bestman.set_jog_relative("Y",300,1500,1)

    #     "机器人以当前坐标位置往Z轴负方向整体运动100mm,到达放置位置"
    #     bestman.set_jog_relative("Z",-100,1500,1)

    #     "控制夹爪完全张开"
    #     bestman.open_gripper(100,100)
        

    #     "机器人以当前坐标位置往Z轴正方向整体运动100mm,到达放置过渡点"
    #     bestman.set_jog_relative("Z",100,1500,1)

    #     "机器人关节运动到安全点"
    #     bestman.set_arm_joint_values([94.828,-143.513,135.283,-82.969,-87.257,-44.033])