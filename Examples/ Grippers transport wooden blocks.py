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

from RoboticsToolBox import Bestman_Real_Elephant, pose_to_euler, euler_to_pose
if __name__=='__main__':
    # 标准姿态 [0.0, -120.0, 120.0, -90.0, -90.0, -0.0]
    # 零位 [0,-90,0,-90,0,0]
    bestman = Bestman_Real_Elephant("192.168.43.38", 5001)
    # bestman.state_on ()
    bestman.power_off ()
    bestman.check_error()
    bestman.get_current_cartesian()
    # 四元组目前不能运行
    target_trajectory = [
        [0.5, 0.5, 0.5, 0.707, 0.0, 0.707, 0.0],
        [0.51, 0.52, 0.515, 0.707, 0.0, 0.653281, 0.270598],
        [0.52, 0.54, 0.53, 0.707, 0.0, 0.55557, 0.382683],
        [0.53, 0.56, 0.545, 0.707, 0.0, 0.382683, 0.55557],
        [0.54, 0.58, 0.56, 0.707, 0.0, 0.270598, 0.653281],
        [0.55, 0.6, 0.575, 0.707, 0.0, 0.0, 0.707],
        [0.56, 0.62, 0.59, 0.653281, 0.0, -0.270598, 0.707],
        [0.57, 0.64, 0.605, 0.55557, 0.0, -0.382683, 0.707],
        [0.58, 0.66, 0.62, 0.382683, 0.0, -0.55557, 0.707],
        [0.59, 0.68, 0.635, 0.270598, 0.0, -0.653281, 0.707],
        ]
    target_trajectory = [
         [150.911058, 180.385238, 300.841710, -160.972194, 0.518498, 110.999205],  # 大幅度移动 + 大角度变化
    [50.911058, 250.385238, 400.841710, -170.972194, -0.518498, 140.999205],  # X,Y,Z 大幅度变化 + 角度微调
    [97.911058, 200.385238, 360.841710, -179.972194, -0.018498, 100.999205],  # Z 方向变化较大，Yaw 大幅调整
    [120.911058, 200.385238, 380.841710, -180.972194, 0.028498, 145.999205],  # X 增加 + Yaw 增加
    [80.911058, 240.385238, 350.841710, -179.972194, -0.028498, 125.999205]   # Y, Z 减少 + Roll, Yaw 减少
        ]
     
    pose_list = []
    # for tt in target_trajectory:
    #     t = pose_to_euler(tt)
    #     pose_list.append(t)
    # print(pose_list)
    bestman.move_arm_follow_target_trajectory(target_trajectory,trajectory_type='eurler')
    # bestman.get_current_joint_values()
    # # bestman.open_gripper()
    # bestman.set_arm_joint_values([0.0, -120.0, 120.0, -90.0, -90.0, -0.0])
    # bestman.robot.get_coords()
    # bestman.get_current_joint_values()
    # # bestman.close_gripper()
    # bestman.power_off()
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