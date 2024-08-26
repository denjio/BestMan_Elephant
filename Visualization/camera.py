import pyrealsense2 as rs  
import numpy as np
import time 
import cv2 
import os

class CameraRoot:
    def __init__(self, camera_width=1280, camera_height=720, camera_fps=30, base_path='.'):

        # #################################################################
        # camera params
        # #################################################################
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps
        self.base_path = base_path

        self.camera_id_list = None
        self.serial_number_dict = None # {0: '241122072950', 1: '241222073328'}
        self.serial_number_list = None
        self.pipeline_list = None
        self.config_list = None
        self.frame_list = None

        self.align = rs.align(rs.stream.color)
        self.__getCameraDeviceInfo()
        self.connect_camera()

    def __getCameraDeviceInfo(self, ):
        """
            {"id":serial number}
        """
        # 创建上下文对象  
        ctx = rs.context()  
        # 查询所有已连接的RealSense设备  
        devs = ctx.query_devices()  
        # 获取设备数量  
        self.camera_ids = range(len(devs))
        self.serial_number_dict = {}
        # 遍历设备  
        for i, dev in enumerate(devs):  
            # 打印设备信息，如序列号等（这里以序列号为例）
            serial_number = dev.get_info(rs.camera_info.serial_number)
            self.serial_number_dict[i] = serial_number
        
        # get camera inter params
        

    def connect_camera(self, ):
        # config realsense
        self.serial_number_list = list(self.serial_number_dict.values())
        self.camera_id_list = list(self.serial_number_dict.keys())

        self.pipeline_list = [rs.pipeline() for i in range(len(self.camera_id_list))]
        self.config_list = [rs.config() for i in range(len(self.camera_id_list))]

        # enable stream
        for n, config in enumerate(self.config_list):
            config.enable_device(self.serial_number_list[n])
            config.enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16,
                                    self.camera_fps)
            config.enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8,
                                    self.camera_fps)
            # config.enable_stream(rs.stream.infrared, n+1 ,self.camera_width, self.camera_height, rs.format.y8, 
            #                         self.camera_fps)  # open 红外
            
           
        # start pipeline
        for n, pipeline in enumerate(self.pipeline_list):
            pipeline.start(self.config_list[n])
    
    def wait_frames(self, frame_id=None):
        '''
            等待进数据
            frame_id:
                camera number , get all frame
                id , get specific id camera frame
        '''
        self.frame_list = [None for i in range(len(self.camera_id_list))]
        if frame_id != None:  # give a frame id
            for n, camera_id in enumerate(self.camera_id_list):
                self.frame_list[n] = self.pipeline_list[frame_id].wait_for_frames()
        else:  # get all frame
            if len(self.camera_id_list) == 1:
                self.frame_list.append(self.pipeline_list[0].wait_for_frames())
            else:
                for n, camera_id in enumerate(self.camera_id_list):
                    self.frame_list[n] = self.pipeline_list[n].wait_for_frames()

        # align RGBD
        for i in range(len(self.frame_list)):      
            self.align.process(self.frame_list[i])
 
    def __rgb_image(self, camera_id=0):
        color_frame = self.frame_list[camera_id].get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image
 
    def __depth_frame(self, camera_id=0):

        depth_frame = self.frame_list[camera_id].get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        colorizer_depth_image = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        return depth_image, colorizer_depth_image

    def __infrared_frame(self, camera_id=0):
            ir_frame_left = self.frame_list[camera_id].get_infrared_frame(1)
            ir_frame_right = self.frame_list[camera_id].get_infrared_frame(2)
            
            ir_left_image = np.asanyarray(ir_frame_left.get_data())
            ir_right_image = np.asanyarray(ir_frame_right.get_data())

            return ir_left_image, ir_right_image

    def get_visual_data(self, camera_id=0):
        """
            rgb 
            depth
            @ cloud point
        """
        rgb = self.__rgb_image(camera_id)
        depth, colorizer_depth = self.__depth_frame(camera_id)

        # ir_l_image, ir_r_image = self.__infrared_frame(camera_id)

        return rgb, depth, colorizer_depth

    def get_video_data(self, ):

        pass

    def get_calibration_image_by_user(self, image_num=20):
        Count = 0
        while Count < image_num:
            self.wait_frames()
            rgb, depth, colorizer_depth = self.get_visual_data(camera_id=1)

            cv2.imshow("rgb", rgb)
            cv2.imshow("depth", depth)
            
            key = cv2.waitKey(1) & 0xFF  

            if key == ord('q'):
                break
            if key == ord("w"):
                self.save_image(rgb, 'test' + '_' + str(Count))
                Count += 1
    
        self.stop()

    def get_3d_points(self, filter_dist=[0, 1]):
        """
        Convert depth image to 3D point cloud.

        Returns:
            np.ndarray: 3D point cloud.
        """
        xmap, ymap = np.arange(self.depths.shape[1]), np.arange(self.depths.shape[0])
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = self.depths
        points_x = (xmap - self.cx) / self.fx * points_z
        points_y = (ymap - self.cy) / self.fy * points_z
        mask = (points_z > filter_dist[0]) & (points_z < filter_dist[1])
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)
        colors = (self.colors / 255.0)[mask].astype(np.float32)
        return points, colors

    def save_image(self, data, save_name):
        # process path
        save_path = self.base_path + os.sep + save_name + '.png'

        # save
        cv2.imwrite(save_path, data)
        print("Save Image at: ", save_path)

    def stop(self, ):
        for pipeline in self.pipeline_list:
            pipeline.stop()
        print("camera exit sucessfully.")


if __name__ == "__main__":
    cap = CameraRoot()
    print(cap.serial_number_dict)
    cap.get_calibration_image_by_user(image_num=2)
    cap.get_3d_points()
    # # 视频保存路径
    # video_path = 'a.mp4'
    # video_depth_path = 'b.mp4'
    # video_depthcolor_path = 'c.mp4'
    # video_depthcolor_camera_path = 'd.mp4'

    # fps = cap.camera_fps
    # w,h = cap.camera_width, cap.camera_height

    # mp4 = cv2.VideoWriter_fourcc(*'mp4v') # 视频格式
    # wr  = cv2.VideoWriter(video_path, mp4, fps, (w, h), isColor=True) # 视频保存而建立对象
    # wr_depth = cv2.VideoWriter(video_depth_path, mp4, fps, (w, h), isColor=False)
    # wr_depthcolor = cv2.VideoWriter(video_depthcolor_path, mp4, fps, (w, h), isColor=True)
    # wr_camera_colordepth = cv2.VideoWriter(video_depthcolor_camera_path, mp4, fps, (w, h), isColor=True)

    # flag_V = 0
    # while True:
    #         color_image, depth_image, colorizer_depth = cap.get_visual_data() # 读取图像帧，包括RGB图和深度图
	# 	    #深度图数据格式转换，uint16→uint8
    #         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

 
    #         # print('ll')
    #         key = cv2.waitKey(1)
    #         if key & 0xFF == ord('s') :
    #             flag_V = 1
    #         if flag_V == 1:
    #             wr.write(color_image)                # 保存RGB图像帧
    #             wr_depth.write(depth_image)          # 保存基于灰度深度图
    #             wr_depthcolor.write(depth_colormap)  # 保存计算所得着色深度图
    #             wr_camera_colordepth.write(colorizer_depth)  # 保存相机自行计算的着色深度图
    #             print('...录制视频中...')
    #         if key & 0xFF == ord('q') or key == 27:
    #             cv2.destroyAllWindows()
    #             print('...录制结束/直接退出...')
    #             break

    # cap.stop()
    # count = 0
    # while True:
    #     start = time.time()
    #     cap.wait_frames()
    #     rgb, depth, = cap.get_visual_data(camera_id=1)

    #     cv2.imshow("rgb", rgb)
    #     cv2.imshow("depth", depth)

    #     print("FPS:", 1 / (time.time() - start), "f/s")
        
    #     key = cv2.waitKey(1) & 0xFF  

    #     if key == ord('q'):
    #         break
    #     if key == ord("w"):
    #         cap.save_image(rgb, 'test' + '_' + str(count))
    #     count += 1
 
    # cap.stop()