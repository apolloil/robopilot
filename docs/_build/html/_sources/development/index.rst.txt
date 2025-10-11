开发文档
========

本部分为开发者提供了详细的开发指南和扩展方法。

.. toctree::
   :maxdepth: 2

   architecture
   extending
   testing
   contributing

概述
----

AhaRobot 机器人遥操作系统采用模块化设计，便于扩展和维护。本部分将帮助开发者：

- 理解系统架构
- 学习如何扩展功能
- 了解测试方法
- 参与项目贡献

开发环境设置
------------

1. 克隆仓库
~~~~~~~~~~~

.. code-block:: bash

   git clone https://github.com/apolloil/robopilot.git
   cd robopilot/main_pr

2. 安装开发依赖
~~~~~~~~~~~~~~~

.. code-block:: bash

   pip install -r requirements.txt
   pip install -e ./modified_pkg/mr_urdf_loader/
   pip install -e ./modified_pkg/urchin/
   
   # 开发工具
   pip install pytest black flake8 mypy

3. 配置开发环境
~~~~~~~~~~~~~~~

.. code-block:: bash

   # 代码格式化
   black src/
   
   # 代码检查
   flake8 src/
   
   # 类型检查
   mypy src/

项目结构
--------

::

   main_pr/
   ├── src/                    # 源代码
   │   ├── main_teleop.py     # 主程序
   │   ├── ik_solver.py       # 逆运动学
   │   ├── vision_tracker.py  # 视觉跟踪
   │   └── org_map.py         # 坐标映射
   ├── model/                  # 机器人模型
   ├── calibration/           # 标定数据
   ├── modified_pkg/          # 修改的第三方包
   ├── docs/                  # 文档
   └── tests/                 # 测试文件

代码规范
--------

命名约定
~~~~~~~~

- 类名使用 PascalCase: ``VisionTracker``
- 函数和变量使用 snake_case: ``get_target_position``
- 常量使用 UPPER_CASE: ``MAX_TORQUE``

文档字符串
~~~~~~~~~~

使用 Google 风格的文档字符串：

.. code-block:: python

   def solve_ik(self, target_position):
       """求解逆运动学
       
       Args:
           target_position (np.ndarray): 目标位置 [x, y, z]
           
       Returns:
           np.ndarray: 关节角度数组
           
       Raises:
           ValueError: 当目标位置超出工作空间时
       """
       pass

类型注解
~~~~~~~~

使用类型注解提高代码可读性：

.. code-block:: python

   from typing import Optional, Tuple
   import numpy as np
   
   def get_target_position(self) -> Optional[np.ndarray]:
       """获取目标位置"""
       pass

扩展指南
--------

添加新的视觉算法
~~~~~~~~~~~~~~~~

1. 继承 VisionTracker 基类
2. 实现必要的接口方法
3. 添加配置参数

.. code-block:: python

   class CustomVisionTracker(VisionTracker):
       def __init__(self, camera_id: int, custom_param: float = 1.0):
           super().__init__(camera_id)
           self.custom_param = custom_param
           
       def get_target_position(self) -> Optional[np.ndarray]:
           # 实现自定义算法
           pass

添加新的逆运动学算法
~~~~~~~~~~~~~~~~~~~~

1. 继承 IKSolver 基类
2. 实现求解方法
3. 添加参数验证

.. code-block:: python

   class CustomIKSolver(IKSolver):
       def solve_ik(self, target_position: np.ndarray) -> np.ndarray:
           # 实现自定义求解算法
           pass

测试指南
--------

单元测试
~~~~~~~~

使用 pytest 编写单元测试：

.. code-block:: python

   import pytest
   import numpy as np
   from src.ik_solver import IKSolver
   
   def test_ik_solver_initialization():
       solver = IKSolver(arm_side="right")
       assert solver.arm_side == "right"
   
   def test_ik_solver_solve():
       solver = IKSolver(arm_side="right")
       target = np.array([0.5, 0.2, 0.3])
       result = solver.solve_ik(target)
       assert len(result) == 5

集成测试
~~~~~~~~

测试模块间的交互：

.. code-block:: python

   def test_vision_to_control_pipeline():
       tracker = VisionTracker(0)
       mapper = OrgMap(side="right")
       solver = IKSolver(arm_side="right")
       
       # 模拟完整的控制流程
       position = tracker.get_target_position()
       mapped = mapper.map_position(position)
       angles = solver.solve_ik(mapped)
       
       assert angles is not None

性能测试
~~~~~~~~

使用 timeit 进行性能测试：

.. code-block:: python

   import time
   
   def test_ik_solver_performance():
       solver = IKSolver(arm_side="right")
       target = np.array([0.5, 0.2, 0.3])
       
       start_time = time.time()
       for _ in range(1000):
           solver.solve_ik(target)
       end_time = time.time()
       
       avg_time = (end_time - start_time) / 1000
       assert avg_time < 0.01  # 平均时间应小于10ms

调试技巧
--------

日志记录
~~~~~~~~

使用 Python logging 模块：

.. code-block:: python

   import logging
   
   logging.basicConfig(level=logging.DEBUG)
   logger = logging.getLogger(__name__)
   
   def solve_ik(self, target_position):
       logger.debug(f"求解逆运动学，目标位置: {target_position}")
       # 求解逻辑
       logger.debug(f"求解结果: {result}")

可视化调试
~~~~~~~~~~

使用 matplotlib 进行数据可视化：

.. code-block:: python

   import matplotlib.pyplot as plt
   
   def visualize_trajectory(positions):
       fig, ax = plt.subplots()
       positions = np.array(positions)
       ax.plot(positions[:, 0], positions[:, 1])
       ax.set_xlabel('X')
       ax.set_ylabel('Y')
       plt.show()

贡献指南
--------

提交代码
~~~~~~~~

1. Fork 仓库
2. 创建功能分支
3. 提交更改
4. 创建 Pull Request

代码审查
~~~~~~~~

- 确保代码符合项目规范
- 添加必要的测试
- 更新文档
- 通过所有测试

发布流程
~~~~~~~~

1. 更新版本号
2. 更新 CHANGELOG
3. 创建发布标签
4. 构建文档

下一步
------

- 了解 :doc:`architecture` 系统架构
- 学习 :doc:`extending` 扩展方法
- 查看 :doc:`testing` 测试指南
- 阅读 :doc:`contributing` 贡献指南
