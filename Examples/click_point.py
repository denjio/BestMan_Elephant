
import pyrealsense2 as rs
import numpy as np
import cv2

# 初始化 RealSense 流程
pipeline = rs.pipeline()
config = rs.config()

# 启用深度流和颜色流
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# 开始流
pipeline.start(config)

# 创建对齐对象 (将深度图像对齐到颜色图像)
align_to = rs.stream.color
align = rs.align(align_to)

# 鼠标回调函数
def get_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        aligned_depth_frame, color_image = param
        
        # 获取点击位置的深度值 z
        z = aligned_depth_frame.get_distance(x, y) * 1000  # 将 z 值从米转换为毫米
        
        print(f"点击位置 (u, v): ({x}, {y}), 深度值 z: {z} 毫米")

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