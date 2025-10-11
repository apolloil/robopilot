API 参考
========

本部分提供了 AhaRobot 机器人遥操作系统的完整 API 文档。

.. toctree::
   :maxdepth: 2

   vision_tracker
   ik_solver
   org_map
   main_teleop

概述
----

API 文档涵盖了系统中所有主要模块的接口说明，包括类、方法和参数。

模块概览
--------

VisionTracker
~~~~~~~~~~~~~

视觉跟踪模块，负责实时目标检测和跟踪。

.. code-block:: python

   from vision_tracker import VisionTracker
   
   # 创建视觉跟踪器
   tracker = VisionTracker(camera_id=0)
   
   # 获取目标位置
   position = tracker.get_target_position()

IKSolver
~~~~~~~~

逆运动学求解器，将笛卡尔空间位置转换为关节角度。

.. code-block:: python

   from ik_solver import IKSolver
   
   # 创建逆运动学求解器
   ik_solver = IKSolver(arm_side="right")
   
   # 求解逆运动学
   joint_angles = ik_solver.solve_ik(target_position)

OrgMap
~~~~~~

组织映射模块，处理坐标系转换和映射。

.. code-block:: python

   from org_map import OrgMap
   
   # 创建映射器
   mapper = OrgMap(side="right")
   
   # 映射位置
   mapped_position = mapper.map_position(image_position)

主程序
~~~~~~

主控制程序，集成所有模块并提供完整的遥操作功能。

.. code-block:: python

   # 运行主程序
   python src/main_teleop.py

数据类型
--------

位置和姿态
~~~~~~~~~~~

系统使用 NumPy 数组表示位置和姿态：

.. code-block:: python

   import numpy as np
   
   # 3D 位置 [x, y, z]
   position = np.array([0.5, 0.2, 0.3])
   
   # 4x4 变换矩阵
   transform = np.eye(4)
   transform[:3, 3] = position

关节角度
~~~~~~~~

关节角度以弧度为单位：

.. code-block:: python

   # 5个关节的角度
   joint_angles = np.array([0.1, -0.5, 0.8, -0.3, 0.2])

错误处理
--------

系统使用标准的 Python 异常处理机制：

.. code-block:: python

   try:
       position = tracker.get_target_position()
   except Exception as e:
       print(f"获取位置失败: {e}")

配置参数
--------

系统支持通过配置文件或代码参数进行配置：

.. code-block:: python

   # 控制参数
   prim_Kp = 70          # 比例增益
   prim_Kd = 2*np.sqrt(prim_Kp)-3  # 微分增益
   prim_max_torque = 30  # 最大扭矩
   
   # 滤波参数
   low_pass_coff = 0.6   # 低通滤波系数

性能考虑
--------

- 视觉处理频率建议不超过 30 FPS
- 逆运动学求解时间通常在 1-5ms
- 建议使用多线程处理以提高响应性

版本兼容性
----------

- Python 3.8+
- NumPy 2.0+
- OpenCV 4.12+
- MuJoCo 3.3+

下一步
------

- 查看具体的模块文档了解详细接口
- 参考示例代码学习使用方法
- 阅读开发文档了解扩展方法
