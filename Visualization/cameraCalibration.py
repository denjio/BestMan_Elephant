import cv2
import numpy as np
import glob
import os
import argparse
import sys
import json
sys.path.append(os.getcwd())
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))

from RoboticsToolBox import Bestman_Real_Elephant


def main():
    try:
        # # Instantiate robot interface
        # bestman =  Bestman_Real_Realman(args.robot_ip, args.frequency) # TODO:re Connect
        
        # # Initialize logging
        # bestman.clear_fault()
        # bestman.go_home()

        # # get calibration image 
        # bestman.get_calibration_image_by_user(args.base_name, args.collect_image_num, args.type)

        # ###################################################################
        # Calibration
        # ###################################################################

        # 标定板的大小，标定板内角点的个数
        # CHECKERBOARD = (6,9)
        CHECKERBOARD = (7,9)
        square_size = 2
        # 角点优化，迭代的终止条件
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # step1:定义标定板在真实世界的坐标
        # 创建一个向量来保存每张图片中角点的3D坐标
        objpoints = []
        # 创建一个向量来保存每张图片中角点的2D坐标
        imgpoints = []

        # 定义3D坐标:[row,col,z]
        objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
      

        # step2:提取不同角度拍摄的图片
        images = glob.glob('/home/robot/Desktop/BestMan_Elephant/Visualization/calibration_image/*.png')
        images = sorted(images)
        # print('images',images)
        for i,fname in enumerate(images):
            # print(fname)
            img = cv2.imread(fname) # 读取图片
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # RGB转换成灰度图

            # step3:计算标定板的角点的2D坐标
            # 寻找角点坐标，如果找到ret返回True, corners:[col, row]，原点在左上角
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+
                cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)

            if ret == True:
                objpoints.append(objp)
                # 调用cornerSubpix对2D角点坐标位置进行优化
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # 绘制寻找到的角点，从红色开始绘制，紫色结束
                img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2,ret)
                cv2.imshow(fname+" succeed", img)
            else:
                print(f"第{i}张图，{fname}未发现足够角点")
                cv2.imshow(fname + " failed", img)
        # while True:
        #     pass
            cv2.waitKey(1)
        cv2.destroyAllWindows()

        h,w = img.shape[:2]

        # step4:相加标定
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        print(rvecs,tvecs)
        # objpoints = np.array(objpoints).squeeze(1)
        # imgpoints = np.array(imgpoints).squeeze(2)
        # rvecs = []
        # tvecs = []
        # # 遍历每个视角
        # for i in range(8):
        #     # 从每个视角中提取 3D 点和 2D 点
        #     obj_pts = objpoints[i]
        #     img_pts = imgpoints[i]

        #     # 调用 solvePnP 计算姿态
        #     success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, mtx, dist)

        #     # 存储计算得到的旋转和平移向量
        #     if success:
        #         rvecs.append(rvec)
        #         tvecs.append(tvec)
        # # 调用 solvePnP 进行姿态估计
         
        mtx_list = mtx.flatten().tolist()  
        dist_list = dist.flatten().tolist()  
        rvecs_list = [vec.flatten().tolist() for vec in rvecs]  
        tvecs_list = [vec.flatten().tolist() for vec in tvecs]  

        print("相机内参:") # [[fx,0,cx],[0,fy,cy],[0,0,1]]
        print(mtx_list,type(mtx_list),"\n")
        print("畸变参数:") # k1,k2,p1,p2,k3
        print(dist_list,"\n")
        print("旋转矩阵:")
        print(rvecs_list,"\n")
        print("平移矩阵:")
        print(tvecs_list,"\n")

        # 创建一个字典来存储所有结果  
        camera_params = {  
            "ret": ret,  
            "mtx": mtx_list,  
            "dist": dist_list,  
            "rvecs": rvecs_list,  
            "tvecs": tvecs_list  
        }  
     

        # print(camera_params)
        # save camera params to file 
        # 我们要将这个数据写入一个名为'data.json'的文件  
        filename = 'camera_params.json'  

        # 使用json.dump()方法将数据写入文件  
        with open(filename, 'w', encoding='utf-8') as f:  
            json.dump(camera_params, f, ensure_ascii=False, indent=4)  

        print(f"数据已成功写入{filename}")
        
        # step5:去畸变
        img = cv2.imread('/home/robot/Desktop/BestMan_Elephant/Visualization/calibration_image/test_0.png')
        h, w = img.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

        # 使用undistort矫正图像
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        # 图片裁剪
        x, y, w, h = roi
        dst2 = dst[y:y+h, x:x+w]
        print(f"ROI: x:{x},y:{y},w:{w},h:{h}")
        cv2.imshow("original image", img)
        cv2.imshow("image undistorted1", dst)
        cv2.imshow("image undistorted1 ROI", dst2)
        cv2.imwrite('/home/robot/Desktop/BestMan_Elephant/Visualization/undistorted/img_undistort.png', dst2)
        # 使用remapping
        mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
        dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)
        

        # crop the image
        x, y, w, h = roi
        dst2 = dst[y:y+h, x:x+w] # x，宽，y，高
        cv2.imshow("image undistorted2", dst)
        cv2.imshow("image undistorted2 ROI", dst2)
        cv2.imwrite('/home/robot/Desktop/BestMan_Elephant/Visualization/undistorted/img_remap.png', dst2)
        cv2.waitKey(1)
        cv2.destroyAllWindows()
        # print(1)
        # step6:计算重投影误差：
        mean_error = 0
        for i in range(len(objpoints)):
            # 使用内外参和畸变参数对点进行重投影
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            # print(imgpoints[i][0], imgpoints2[0])
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2) # L2范数，均方误差
            mean_error += error
        # print(1)
        mean_error /= len(objpoints)
        print("total error: {}".format(mean_error))

    except Exception as e:
        # Log any exceptions that occur
        print(f"Error {str(e)}")

# def image_process(img, mtx, dist, ):
#     # img = cv2.imread('./calibration_images_fold/left12.png')
#     h, w = img.shape[:2]
#     newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

#     # 使用undistort矫正图像
#     dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
#     # 图片裁剪
#     x, y, w, h = roi
#     dst2 = dst[y:y+h, x:x+w]
#     print(f"ROI: x:{x},y:{y},w:{w},h:{h}")
#     cv2.imshow("original image", img)
#     cv2.imshow("image undistorted1", dst)
#     cv2.imshow("image undistorted1 ROI", dst2)
#     cv2.imwrite('./undistorted/img_undistort.png', dst2)

#     # 使用remapping
#     mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
#     dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

#     # crop the image
#     x, y, w, h = roi
#     dst2 = dst[y:y+h, x:x+w] # x，宽，y，高

#     return dst2

# def world_coordinates(image_x, image_y, ):
#     pass

if __name__ == "__main__":
    main()




