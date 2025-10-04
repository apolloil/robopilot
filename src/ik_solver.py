'''
遥操控的第三个模块,逆运动学求解器
核心功能:给定末端执行器位姿，获取关节信息
注意：平移关节的调节需要根据实际摄像机情况计算
'''
import numpy as np
from pytransform3d import transformations as pt
from pathlib import Path
import math
import modern_robotics as mr
from mr_urdf_loader import loadURDF

class IKSolver:
    def __init__(self, model_path="../model/astra.urdf", arm_side="right"):
        """
        初始化逆运动学求解器

        Args:
            model_path: 机器人模型文件路径 (URDF)
            arm_side: 手臂选择，"right" 或 "left"
        """
        self.model_path = model_path
        self.arm_side = arm_side  # "right" 或 "left"

        # 根据手臂选择设置末端执行器名称
        if arm_side == "right":
            self.eef_link_name = "link_ree_teleop"
        else:
            self.eef_link_name = "link_lee_teleop"

        # 关节名称定义
        self.joint_names = self._init_joint_names()

        # 加载机器人模型
        self.M, self.Slist, self.Blist, self.Mlist, self.Glist, self.robot = loadURDF(
            self.model_path,
            self.eef_link_name,
            self.joint_names
        )

        # 初始化关节限制
        self.joint_limits_lower, self.joint_limits_upper = self._init_joint_limits()
        
        # 初始化上次求解的关节角度
        self.last_theta = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    def _init_joint_names(self):
        """初始化关节名称列表"""
        if self.arm_side == "right":
            # 右臂6自由度: joint_r1(升降), joint_r2-r6(5个旋转关节)
            joint_names = [
                'joint_r1',  # 升降关节 (slide)
                'joint_r2',  # 肩关节1 (hinge)
                'joint_r3',  # 肩关节2 (hinge)
                'joint_r4',  # 肘关节 (hinge)
                'joint_r5',  # 腕关节1 (hinge)
                'joint_r6',  # 腕关节2 (hinge)
            ]
        else:
            # 左臂6自由度: joint_l1(升降), joint_l2-l6(5个旋转关节)
            joint_names = [
                'joint_l1',  # 升降关节 (slide)
                'joint_l2',  # 肩关节1 (hinge)
                'joint_l3',  # 肩关节2 (hinge)
                'joint_l4',  # 肘关节 (hinge)
                'joint_l5',  # 腕关节1 (hinge)
                'joint_l6',  # 腕关节2 (hinge)
            ]

        return joint_names

    def _init_joint_limits(self):
        """设置关节限制,用于验证解的合法性"""
        joint_limit_lower = []
        joint_limit_upper = []
        for joint_name in self.joint_names:
            joint = self.robot.joint_map[joint_name]
            joint_limit_lower.append(joint.limit.lower)
            joint_limit_upper.append(joint.limit.upper)
        return joint_limit_lower, joint_limit_upper
    
    def solve(self, target_pose):
        """
        求解逆运动学

        Args:
            target_pose: 4x4目标位姿矩阵

        Returns:
            tuple: (joint_msg, success) 关节信息和求解成功标志
        """
        if target_pose is None:
            return None, False
        # 多初值优化
        for initial_guess in [
            self.last_theta,  # 优先使用上次求解结果，连续性优先
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 使用零角度作为初值
        ]:
            theta_list, success = mr.IKinSpace(
                Slist=self.Slist,
                M=self.M,
                T=target_pose,
                thetalist0=initial_guess,
                eomg=0.005,  # 位置误差容差
                ev=0.005     # 姿态误差容差
            )
            # 无法求解
            if not success:
                # print("IK求解失败")
                continue
            # 归一化旋转关节到[-pi,pi] 
            for i in range(1, len(theta_list)):
                theta_list[i] = math.fmod(math.fmod(theta_list[i] + math.pi, 2*math.pi) + 2*math.pi, 2*math.pi) - math.pi
            
            # 检查关节限制
            ok = True
            for i, (p, mn, mx) in enumerate(zip(theta_list, self.joint_limits_lower, self.joint_limits_upper)):
                if not (mn <= p <= mx):
                    print(f"Joint #{i+1} reach limit, min: {mn}, max: {mx}, current pos: {p}")
                    ok = False
            if not ok:
                continue
            self.last_theta = theta_list
            return theta_list, True
        return None, False


if __name__ == "__main__":
    ik_solver_r = IKSolver(arm_side="right")
    print(ik_solver_r.M)
    print("*"*30)
    ik_solver_l = IKSolver(arm_side="left")
    print(ik_solver_l.M)
