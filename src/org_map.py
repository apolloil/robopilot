'''
遥操控的第二个模块,坐标映射模块,实现real2sim的坐标转换
核心功能: 获取mujoco坐标系下,末端执行器(手柄)坐标系的转换矩阵表示
注意:mujoco_to_cam需要根据实际摄像机情况计算
'''
import numpy as np
from pytransform3d import transformations as pt

class OrgMap:
    def __init__(self, side="right"):
        """初始化坐标映射器"""
        self.side = side
        # mujoco坐标系（sim）下相机坐标系（real）的转换矩阵表示
        # 计算方法：根据零位时，cam_to_tag和mujoco_to_cam，逆解出mujoco_to_tag
        if side =="left":
            self.mujoco_to_cam = np.array([
                [0, 0, -1, 1], 
                [1, 0, 0, 0.43], 
                [0, -1, 0, 0.3], 
                [0, 0, 0, 1], 
            ])
        else:
            self.mujoco_to_cam = np.array([
                [0, 0, -1, 1], 
                [1, 0, 0, -0.43], 
                [0, -1, 0, 0.3], 
                [0, 0, 0, 1], 
            ])
    
    def get_target_position(self, cam_to_tag):
        """
        获取末端执行器位置
        
        Args:
            cam_to_tag: 相机坐标系下，末端执行器(手柄)坐标系的转换矩阵表示

        Returns:
            np.ndarray: mujoco坐标系下,末端执行器(手柄)坐标系的转换矩阵表示
        """
        if cam_to_tag is None:
            return None
            
        # 转换为MuJoCo坐标系并提取位置
        return self.mujoco_to_cam @ cam_to_tag
    

if __name__ == "__main__":
    pass
