import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
from skimage import measure
from skimage.filters import threshold_otsu


image_points_left1=[]
def mouse_callback(event, x, y, flags, param):
    image = param
    if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键按下时
        # 绘制绿色圆点标记选择的点
        cv2.circle(image, (x, y), 3, (0, 255, 0), -1) #第三个参数为选点尺寸大小，第四个参数为选点标记颜色，第五个参数表示颜色填充整个圆
        # 显示带有标记的图像
        cv2.imshow('RGB_Image', image)
        # 输出点击位置的坐标
        print(f"Clicked on position ({x}, {y})")
        image_points_left1.append([x,y])

def pca_function(xyz_src, xyz_dst):
    src_mean = np.mean(xyz_src, axis=1, keepdims=True)
    dst_mean = np.mean(xyz_dst, axis=1, keepdims=True)

    src_centered = xyz_src - src_mean
    dst_centered = xyz_dst - dst_mean

    H = src_centered @ dst_centered.T
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = dst_mean - R @ src_mean
    return R, t

XYZInRobot = np.array([
    [230.949, -360.680, 154.075],
    [107.448, -360.681, 154.076],
    [0.446, -359.432, 154.076],
    [0.447, -260.680, 159.074],
    [182.458, -260.675, 159.066],
    [294.951, -260.674, 159.065],
    [370.691, -260.684, 159.064],
    [431.448, -224.180, 159.090],
    [322.449, -224.181, 159.084],
    [188.448, -224.183, 159.084]
])

# Camera intrinsic parameters
ppx = 631.20788574
ppy = 372.19293213
fx = 644.7208252
fy = 643.23901367
K_Color = np.array([[fx, 0, ppx], [0, fy, ppy], [0, 0, 1]])

WindowsR = 15
xyzInColorCamAll = []
import re
def natural_sort_key(s):
    # 将字符串中的数字部分提取出来，并转换为整数
    return [int(text) if text.isdigit() else text for text in re.split(r'(\d+)', s)]
# Load images
img_paths = sorted(glob.glob('data/bd/*c.png'))
img_paths = sorted(img_paths, key=natural_sort_key)
for i, img_path in enumerate(img_paths):
    img_color = cv2.imread(img_path) / 255.0
    img_depth = cv2.imread(img_path.replace('c.png', 'd.png'), cv2.IMREAD_UNCHANGED).astype(np.float64)
    image_points_left1=[]
    cv2.namedWindow('RGB_Image')
    # 绑定鼠标回调函数
    cv2.setMouseCallback('RGB_Image', mouse_callback,img_color)
    # 显示RGB图像
    cv2.imshow('RGB_Image', img_color)
    while True:
        if cv2.waitKey(1) == 27:
            break
    uv=image_points_left1[-1]
    
    uv = np.round(uv).astype(int)
    heigth, width, _ = img_color.shape
    z_depth = img_depth[round(uv[1]), round(uv[0])]
    uv1 = np.array([uv[0], uv[1], 1])
    xyz_in_color_cam = np.linalg.inv(K_Color) @ (z_depth * uv1)
    xyzInColorCamAll.append(xyz_in_color_cam)

xyzInColorCamAll = np.array(xyzInColorCamAll)

# Visualization
fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.scatter(XYZInRobot[:, 0], XYZInRobot[:, 1], XYZInRobot[:, 2], c='r', s=100)
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(xyzInColorCamAll[:, 0], xyzInColorCamAll[:, 1], xyzInColorCamAll[:, 2], c='r', s=100)
plt.show()

# Coarse registration
Index = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
RCam2Robot, tCam2Robot = pca_function(xyzInColorCamAll[Index].T, XYZInRobot[Index].T)
XYZInRobotFromCam = (RCam2Robot @ xyzInColorCamAll.T + tCam2Robot).T
T_Cam2Robot = np.vstack([np.hstack([RCam2Robot, tCam2Robot]), np.array([0, 0, 0, 1])])
print("T_Cam2Robot:")
print(T_Cam2Robot)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(XYZInRobotFromCam[:, 0], XYZInRobotFromCam[:, 1], XYZInRobotFromCam[:, 2], c='r', s=100)
ax.scatter(XYZInRobot[:, 0], XYZInRobot[:, 1], XYZInRobot[:, 2], c='b', s=100)
plt.show()

# Error calculation
ErrorX = np.abs(XYZInRobotFromCam[:, 0] - XYZInRobot[:, 0])
ErrorY = np.abs(XYZInRobotFromCam[:, 1] - XYZInRobot[:, 1])
ErrorZ = np.abs(XYZInRobotFromCam[:, 2] - XYZInRobot[:, 2])
Error = np.linalg.norm(XYZInRobotFromCam - XYZInRobot, axis=1)

plt.plot(Error, label='Euclidean Distance')
plt.plot(ErrorX, label='X Error')
plt.plot(ErrorY, label='Y Error')
plt.plot(ErrorZ, label='Z Error')
plt.legend()
plt.xlabel('Index')
plt.ylabel('Error (mm)')
plt.show()

print('Mean Error X:', np.mean(ErrorX))
print('Mean Error Y:', np.mean(ErrorY))
print('Mean Error Z:', np.mean(ErrorZ))
