'''
遥操控项目的第一个模块，实现视觉实时分析
核心功能函数:get_transform(side)
输出:相机坐标系下，末端执行器(手柄)坐标系的转换矩阵表示
'''
import cv2
import numpy as np
from pathlib import Path
import glob
import yaml
from yaml.loader import SafeLoader
import platform
from pytransform3d import transformations as pt
from pytransform3d import rotations as pr
from time import sleep

class VisionTracker:
    def __init__(self, camera_device="/dev/video0", calibration_dir="../calibration"):
        """
        初始化视觉跟踪器
        
        Args:
            camera_device: 摄像头设备路径
            calibration_dir: 标定文件目录
        """
        self.camera_device = camera_device
        self.calibration_dir = calibration_dir
        
        # 初始化摄像头
        self.cam = self._init_camera()
        
        # 加载标定参数
        self.camera_matrix, self.distortion_coeffs = self._load_calibration()
        
        # 初始化ArUco检测器
        self.detector = self._init_aruco_detector()
        
        # ArUco标记的3D坐标定义
        self.obj_points_map = self._init_aruco_coordinates()
        
        # 低通滤波状态
        self.transform_left_last = None
        self.transform_right_last = None
    
    def _init_camera(self):
        """初始化摄像头，返回摄像头对象"""
        # 跨平台兼容的摄像头打开方式
        device = self.camera_device
        if platform.system() == "Windows":
            # windows系统使用数字索引
            cam = cv2.VideoCapture(device)
        else:
            # linux系统使用V4L2,稳定性更高
            cam = cv2.VideoCapture(device, cv2.CAP_V4L2)
        
        # 检查摄像头是否成功打开
        if not cam.isOpened():
            raise RuntimeError(f"无法打开摄像头设备: {device}")
        
        # 设置缓冲区大小
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 尝试设置MJPG格式（如果支持）
        try:
            fourcc_value = cv2.VideoWriter_fourcc(*'MJPG')
            cam.set(cv2.CAP_PROP_FOURCC, fourcc_value)
        except:
            print("警告: 无法设置MJPG格式，使用默认格式")
        
        # 获取相机参数
        width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)   
        height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cam.get(cv2.CAP_PROP_FPS)
        
        print(f"camera: {width}x{height}@{fps}")
        
        return cam
    
    def _load_calibration(self):
        """加载摄像头标定参数，返回相机内参矩阵和畸变系数"""
        file_names = glob.glob(str(Path(self.calibration_dir) / 'calibration_results_*.yaml'))
        print(Path(self.calibration_dir))
        print(file_names)
        
        with open(file_names[-1]) as f:
            calibration = yaml.load(f, Loader=SafeLoader)
        
        camera_matrix = np.array(calibration['camera_matrix'], dtype=np.float32)
        distortion_coeffs = np.array(calibration['distortion_coefficients'], dtype=np.float32)
        
        return camera_matrix, distortion_coeffs
    
    def _init_aruco_detector(self):
        """初始化ArUco检测器，返回检测器对象"""
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco__detect_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco__detect_params)
        return detector
    
    def _init_aruco_coordinates(self):
        """定义ArUco标记的3D坐标，返回3D坐标映射字典"""
        obj_points_map = {
            0:  [(-15.00, -15.00,  48.28), (-15.00,  15.00,  48.28), ( 15.00,  15.00,  48.28), ( 15.00, -15.00,  48.28)],
            1:  [( 23.54, -15.00,  44.75), ( 23.54,  15.00,  44.75), ( 44.75,  15.00,  23.54), ( 44.75, -15.00,  23.54)],
            2:  [(-15.00, -23.54,  44.75), ( 15.00, -23.54,  44.75), ( 15.00, -44.75,  23.54), (-15.00, -44.75,  23.54)],
            3:  [(-23.54,  15.00,  44.75), (-23.54, -15.00,  44.75), (-44.75, -15.00,  23.54), (-44.75,  15.00,  23.54)],
            4:  [( 15.00,  23.54,  44.75), (-15.00,  23.54,  44.75), (-15.00,  44.75,  23.54), ( 15.00,  44.75,  23.54)],
            5:  [( 48.28, -15.00,  15.00), ( 48.28,  15.00,  15.00), ( 48.28,  15.00, -15.00), ( 48.28, -15.00, -15.00)],
            6:  [( 23.54, -44.75,  15.00), ( 44.75, -23.54,  15.00), ( 44.75, -23.54, -15.00), ( 23.54, -44.75, -15.00)],
            7:  [(-15.00, -48.28,  15.00), ( 15.00, -48.28,  15.00), ( 15.00, -48.28, -15.00), (-15.00, -48.28, -15.00)],
            8:  [(-44.75, -23.54,  15.00), (-23.54, -44.75,  15.00), (-23.54, -44.75, -15.00), (-44.75, -23.54, -15.00)],
            9:  [(-48.28,  15.00,  15.00), (-48.28, -15.00,  15.00), (-48.28, -15.00, -15.00), (-48.28,  15.00, -15.00)],
            10: [(-23.54,  44.75,  15.00), (-44.75,  23.54,  15.00), (-44.75,  23.54, -15.00), (-23.54,  44.75, -15.00)],
            11: [( 15.00,  48.28,  15.00), (-15.00,  48.28,  15.00), (-15.00,  48.28, -15.00), ( 15.00,  48.28, -15.00)],
            12: [( 44.75,  23.54,  15.00), ( 23.54,  44.75,  15.00), ( 23.54,  44.75, -15.00), ( 44.75,  23.54, -15.00)],
            13: [( 44.75, -15.00, -23.54), ( 44.75,  15.00, -23.54), ( 23.54,  15.00, -44.75), ( 23.54, -15.00, -44.75)],
            14: [(-15.00, -44.75, -23.54), ( 15.00, -44.75, -23.54), ( 15.00, -23.54, -44.75), (-15.00, -23.54, -44.75)],
            15: [(-44.75,  15.00, -23.54), (-44.75, -15.00, -23.54), (-23.54, -15.00, -44.75), (-23.54,  15.00, -44.75)],
            16: [( 15.00,  44.75, -23.54), (-15.00,  44.75, -23.54), (-15.00,  23.54, -44.75), ( 15.00,  23.54, -44.75)],
        }
        
        # 复制右手标记到左手 (ID 18-34)
        for i in range(17):
            obj_points_map[i + 18] = obj_points_map[i]
        
        return obj_points_map
    
    def detect_markers(self, image, debug=True):
        """
        检测ArUco标记
        Args:
            image: 输入图像
            debug: 是否实时显示相机检测到的标记图像
        Returns:
            ids: 识别到的ID列表
            corners: 识别到的id对应的角点(2D坐标)列表
        """
        corners, ids, _ = self.detector.detectMarkers(image)
        if debug:
            debug_image = image.copy()
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
            cv2.imshow('ArUco Detection', debug_image)
        cv2.waitKey(1)
    
        return ids, corners
    
    def solve_pose(self, ids, corners, is_left_hand):
        """
        求解标记姿态，返回4x4变换矩阵
        Args:
            ids: 识别到的ID列表
            corners: 识别到的id对应的角点(2D坐标)列表
            is_left_hand: True表示左手，False表示右手
            
        Returns:
            np.ndarray: 相机坐标系下，末端执行器(手柄)坐标系的转换矩阵表示
        """
        if ids is None or len(ids) == 0:
            return None
        
        # 只保留指定手部的标记
        tags = []
        for id, corner in zip(ids, corners):
            id = id.item()
            corner = corner.squeeze()
            marker_is_left = id >= 18
            
            if is_left_hand == marker_is_left:
                tags.append((id, corner))
        
        # 检查标记数量
        if len(tags) < 4:
            return None
        
        # 按面积排序，选择最大的4个标记
        tags.sort(key=lambda tag: cv2.contourArea(tag[1]), reverse=True)
        tags = tags[:4]
        
        # 构建3D-2D对应点
        obj_points = []
        img_points = []
        for id, corner in tags:
            if id in self.obj_points_map:
                obj_points.extend(self.obj_points_map[id])
                img_points.extend(corner)
        
        # PnP求解
        _, rvec, tvec = cv2.solvePnP(
            np.array(obj_points) / 1000,  # 转换为米
            np.array(img_points),
            self.camera_matrix, 
            self.distortion_coeffs
        )
        
        # 转换为4x4变换矩阵
        transform = pt.transform_from(
            pr.matrix_from_compact_axis_angle(rvec.squeeze()), 
            tvec.squeeze()
        )
        
        return transform
    
    def get_transform(self):
        """获取左右手的变换矩阵"""
        ret, image = self.cam.read()
        if not ret:
            return None, None
        
        ids, corners = self.detect_markers(image)
        transform_left = self.solve_pose(ids, corners, True)
        transform_right = self.solve_pose(ids, corners, False)
        
        # 低通滤波
        low_pass_coff = 0.4
        if transform_left is not None:
            if self.transform_left_last is None:
                self.transform_left_last = transform_left
            else:
                transform_left = pt.transform_from_pq(pt.pq_slerp(
                    pt.pq_from_transform(self.transform_left_last),
                    pt.pq_from_transform(transform_left),
                    low_pass_coff
                ))
                self.transform_left_last = transform_left
        
        if transform_right is not None:
            if self.transform_right_last is None:
                self.transform_right_last = transform_right
            else:
                transform_right = pt.transform_from_pq(pt.pq_slerp(
                    pt.pq_from_transform(self.transform_right_last),
                    pt.pq_from_transform(transform_right),
                    low_pass_coff
                ))
                self.transform_right_last = transform_right
        
        return transform_left, transform_right
    
    def release(self):
        """释放资源"""
        if hasattr(self, 'cam'):
            self.cam.release()

if __name__ == "__main__":
    vision_tracker = VisionTracker(0)
    cnt = 0
    while True:
        transform_l, transform_r = vision_tracker.get_transform()
        print('*'*10,cnt, '*'*10,sep='')
        cnt+=1
        print(transform_l)
        sleep(0.2)

