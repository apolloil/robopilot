快速开始
========

本指南将帮助您快速上手 AhaRobot 机器人遥操作系统。

基本使用
--------

1. 启动主程序
~~~~~~~~~~~~~~

.. code-block:: bash

   cd main_pr
   python src/main_teleop.py

2. 系统初始化
~~~~~~~~~~~~~~

程序启动后会自动：

- 初始化摄像头（索引0）
- 加载机器人模型（astra.xml）
- 设置左右臂的逆运动学求解器
- 配置控制参数

3. 控制界面
~~~~~~~~~~~~

系统提供以下控制功能：

- **视觉跟踪**: 自动检测和跟踪目标对象
- **双臂控制**: 同时控制左右两个机械臂
- **实时反馈**: 提供实时的位置和姿态反馈

基本配置
--------

摄像头设置
~~~~~~~~~~

.. code-block:: python

   # 修改摄像头索引
   vision_tracker = VisionTracker(0)  # 0 为默认摄像头

模型路径
~~~~~~~~

.. code-block:: python

   # 修改模型文件路径
   model = mujoco.MjModel.from_xml_path("../model/astra.xml")

控制参数
~~~~~~~~

.. code-block:: python

   # 平移关节参数
   prim_Kp = 70          # 比例增益
   prim_Kd = 2*np.sqrt(prim_Kp)-3  # 微分增益
   prim_max_torque = 30  # 最大扭矩

   # 旋转关节参数
   rotate_Kp = 40        # 比例增益
   rotate_Kd = 1         # 微分增益
   rotate_max_torque = 50  # 最大扭矩

   # 低通滤波参数
   low_pass_coff = 0.6   # 位置滤波系数
   ctrl_low_pass_coff = 0.6  # 控制滤波系数

示例代码
--------

简单的遥操作示例
~~~~~~~~~~~~~~~~

.. code-block:: python

   import numpy as np
   from vision_tracker import VisionTracker
   from org_map import OrgMap
   from ik_solver import IKSolver
   import mujoco
   
   # 初始化组件
   vision_tracker = VisionTracker(0)
   org_map_r = OrgMap(side="right")
   ik_solver_r = IKSolver(arm_side="right")
   
   # 加载模型
   model = mujoco.MjModel.from_xml_path("model/astra.xml")
   data = mujoco.MjData(model)
   
   # 主循环
   while True:
       # 获取视觉信息
       target_pos = vision_tracker.get_target_position()
       
       # 坐标映射
       mapped_pos = org_map_r.map_position(target_pos)
       
       # 逆运动学求解
       joint_angles = ik_solver_r.solve_ik(mapped_pos)
       
       # 应用控制
       data.ctrl[:] = joint_angles
       mujoco.mj_step(model, data)

自定义控制逻辑
~~~~~~~~~~~~~~

.. code-block:: python

   class CustomController:
       def __init__(self):
           self.ik_solver = IKSolver()
           self.target_position = np.array([0, 0, 0])
           
       def update_target(self, new_position):
           """更新目标位置"""
           self.target_position = new_position
           
       def compute_control(self):
           """计算控制输出"""
           return self.ik_solver.solve_ik(self.target_position)

高级功能
--------

双臂协调控制
~~~~~~~~~~~~

.. code-block:: python

   # 初始化双臂控制器
   ik_solver_r = IKSolver(arm_side="right")
   ik_solver_l = IKSolver(arm_side="left")
   
   # 协调控制
   def coordinated_control(target_r, target_l):
       angles_r = ik_solver_r.solve_ik(target_r)
       angles_l = ik_solver_l.solve_ik(target_l)
       return angles_r, angles_l

视觉跟踪集成
~~~~~~~~~~~~

.. code-block:: python

   # 高级视觉跟踪
   class AdvancedVisionTracker(VisionTracker):
       def __init__(self, camera_id):
           super().__init__(camera_id)
           self.tracking_history = []
           
       def get_smoothed_position(self):
           """获取平滑后的位置"""
           current_pos = self.get_target_position()
           self.tracking_history.append(current_pos)
           
           # 保持历史记录长度
           if len(self.tracking_history) > 10:
               self.tracking_history.pop(0)
               
           # 计算平均值
           return np.mean(self.tracking_history, axis=0)

故障排除
--------

常见问题
~~~~~~~~

1. **摄像头无法打开**
   - 检查摄像头是否被其他程序占用
   - 尝试不同的摄像头索引

2. **模型加载失败**
   - 确认模型文件路径正确
   - 检查文件权限

3. **逆运动学求解失败**
   - 检查目标位置是否在可达范围内
   - 调整求解器参数

性能优化
--------

1. **降低计算频率**
   - 调整主循环的更新频率
   - 使用多线程处理

2. **优化视觉处理**
   - 降低图像分辨率
   - 使用更高效的算法

下一步
------

- 阅读 :doc:`user_guide/index` 了解详细功能
- 查看 :doc:`api/index` 了解 API 接口
- 参考 :doc:`development/index` 进行开发
