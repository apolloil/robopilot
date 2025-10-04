'''
遥操控项目的主程序,实现视觉实时分析,坐标转换,逆运动学求解,MuJoCo控制
'''
import time
from cv2 import rotate
import numpy as np
from vision_tracker import VisionTracker
from org_map import OrgMap
from ik_solver import IKSolver
import mujoco
from mujoco import viewer

def clip(value,max_value):
    return np.clip(value/max_value,-1,1)

if __name__ == "__main__":
    vision_tracker = VisionTracker(0)
    org_map_r = OrgMap(side="right")
    org_map_l = OrgMap(side="left")
    ik_solver_r = IKSolver(arm_side="right")
    ik_solver_l = IKSolver(arm_side="left")
    # 加载模型
    model = mujoco.MjModel.from_xml_path("../model/astra.xml")
    data = mujoco.MjData(model)

    # 平移关节参数
    prim_Kp = 70
    prim_Kd = 2*np.sqrt(prim_Kp)-3
    prim_max_torque = 30
    # 旋转关节参数
    rotate_Kp = 40
    rotate_Kd = 1
    rotate_max_torque = 50

    # 要进行低通滤波平滑
    low_pass_coff = 0.6
    ctrl_low_pass_coff = 0.6
    
    # 低通滤波状态变量
    target_r_last = None
    target_l_last = None
    
    # ctrl低通滤波状态变量
    ctrl_r_last = np.array([0, 0, 0, 0, 0])
    ctrl_l_last = np.array([0, 0, 0, 0, 0])
    ctrl_prim_r_last = 0
    ctrl_prim_l_last = 0
    ctrl_r = np.array([0, 0, 0, 0, 0])
    ctrl_l = np.array([0, 0, 0, 0, 0])
    ctrl_prim_r = 0
    ctrl_prim_l = 0
    
    with viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            cam_to_tag_l, cam_to_tag_r = vision_tracker.get_transform()
            mujoco_to_eef_l = org_map_l.get_target_position(cam_to_tag_l)
            mujoco_to_eef_r = org_map_r.get_target_position(cam_to_tag_r)
            target_r, success_r = ik_solver_r.solve(mujoco_to_eef_r)
            target_l, success_l = ik_solver_l.solve(mujoco_to_eef_l)
            # print(target_r)
            
            # 对target（关节信息数组）进行低通滤波
            if success_r and target_r is not None:
                if target_r_last is None:
                    target_r_last = target_r
                else:
                    target_r = low_pass_coff * target_r + (1 - low_pass_coff) * target_r_last
                    target_r_last = target_r
            else:
                target_r_last = None
                
            if success_l and target_l is not None:
                if target_l_last is None:
                    target_l_last = target_l
                else:
                    target_l = low_pass_coff * target_l + (1 - low_pass_coff) * target_l_last
                    target_l_last = target_l
            else:
                target_l_last = None
            
            rotate_r_list = [1,2,3,4,5]
            rotate_l_list = [9,10,11,12,13]
            
            if success_r:
                # 计算右手臂控制信号
                rotate_torque = rotate_Kp * (target_r[1:] - data.qpos[rotate_r_list]) - rotate_Kd * (data.qvel[rotate_r_list])
                ctrl_r = clip(rotate_torque, rotate_max_torque)
                prim_torque = prim_Kp * (target_r[0] - data.qpos[0]) - prim_Kd * (data.qvel[0])
                ctrl_prim_r = clip(prim_torque, prim_max_torque)
                # 对ctrl进行低通滤波
                ctrl_r = ctrl_low_pass_coff * ctrl_r + (1 - ctrl_low_pass_coff) * ctrl_r_last
                ctrl_prim_r = ctrl_low_pass_coff * ctrl_prim_r + (1 - ctrl_low_pass_coff) * ctrl_prim_r_last
            else:
                # 失败时设置当前ctrl为平衡状态值,减少抖动
                ctrl_r = np.array([0, 0, 0, -0.02, 0])
                ctrl_prim_r = 0.11
                # 对ctrl进行平均
                ctrl_r = ctrl_low_pass_coff/5 * ctrl_r + (1 - ctrl_low_pass_coff/5) * ctrl_r_last
                ctrl_prim_r = ctrl_low_pass_coff/5 * ctrl_prim_r + (1 - ctrl_low_pass_coff/5) * ctrl_prim_r_last
            # 设置ctrl历史值
            ctrl_r_last = ctrl_r
            ctrl_prim_r_last = ctrl_prim_r
            # 应用控制信号
            data.ctrl[rotate_r_list] = ctrl_r
            data.ctrl[0] = ctrl_prim_r

            if success_l:
                # 计算左手臂控制信号
                rotate_torque = rotate_Kp * (target_l[1:] - data.qpos[rotate_l_list]) - rotate_Kd * (data.qvel[rotate_l_list])
                ctrl_l = clip(rotate_torque, rotate_max_torque)
                prim_torque = prim_Kp * (target_l[0] - data.qpos[8]) - prim_Kd * (data.qvel[8])
                ctrl_prim_l = clip(prim_torque, prim_max_torque)
                # 对ctrl进行低通滤波
                ctrl_l = ctrl_low_pass_coff * ctrl_l + (1 - ctrl_low_pass_coff) * ctrl_l_last
                ctrl_prim_l = ctrl_low_pass_coff * ctrl_prim_l + (1 - ctrl_low_pass_coff) * ctrl_prim_l_last
            else:
                # 失败时设置当前ctrl为平衡状态值,减少抖动
                ctrl_l = np.array([0, 0, 0, 0.02, 0])
                ctrl_prim_l = 0.11
                # 对ctrl进行平均
                ctrl_l = ctrl_low_pass_coff/5 * ctrl_l + (1 - ctrl_low_pass_coff/5) * ctrl_l_last
                ctrl_prim_l = ctrl_low_pass_coff/5 * ctrl_prim_l + (1 - ctrl_low_pass_coff/5) * ctrl_prim_l_last
            # 设置ctrl历史值
            ctrl_l_last = ctrl_l
            ctrl_prim_l_last = ctrl_prim_l
            # 应用控制信号
            data.ctrl[rotate_l_list] = ctrl_l
            data.ctrl[8] = ctrl_prim_l

            fixed_list = [6,7,14,15,16,17]
            data.ctrl[fixed_list] = 0

            mujoco.mj_step(model, data)
            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - start_time)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            