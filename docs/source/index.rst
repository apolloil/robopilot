AhaRobot 机器人遥操作系统
=====================================

欢迎使用 AhaRobot 机器人遥操作系统文档！

这是一个基于 MuJoCo 物理仿真的 Aharobot 机器人遥操作系统项目，集成了计算机视觉、逆运动学求解和实时机器人控制功能。

.. toctree::
   :maxdepth: 2
   :caption: 用户指南

   installation
   quickstart
   user_guide/index

.. toctree::
   :maxdepth: 2
   :caption: 开发者文档

   api/index
   development/index

.. toctree::
   :maxdepth: 1
   :caption: 其他

   changelog
   contributing

功能特性
--------

🤖 **机器人仿真**
   基于 MuJoCo 的高精度物理仿真，支持完整的 Aharobot 机器人模型

🎮 **遥操作控制**
   实时视觉反馈和精确的机器人控制接口

👁️ **计算机视觉**
   集成 OpenCV 的实时视觉跟踪和目标识别

🔧 **逆运动学求解**
   高效的逆运动学算法，支持双臂协调控制

📐 **机器人标定**
   完整的机器人标定和校准工具

快速开始
--------

安装依赖：

.. code-block:: bash

   pip install -r requirements.txt
   pip install ./modified_pkg/mr_urdf_loader/
   pip install ./modified_pkg/urchin/

运行主程序：

.. code-block:: bash

   python src/main_teleop.py

项目结构
--------

::

   main_pr/
   ├── src/                    # 源代码目录
   │   ├── main_teleop.py     # 主遥操作程序
   │   ├── ik_solver.py       # 逆运动学求解器
   │   ├── vision_tracker.py  # 视觉跟踪模块
   │   └── org_map.py         # 组织映射模块
   ├── model/                  # 机器人模型文件
   │   ├── astra.xml          # MuJoCo XML 模型文件
   │   ├── astra.urdf         # URDF 模型文件
   │   ├── astra.csv          # 关节参数文件
   │   └── meshes/            # 3D 网格文件
   ├── calibration/           # 标定文件
   ├── modified_pkg/          # 修改的第三方包
   └── requirements.txt       # 项目依赖

更多信息
--------

- `MuJoCo RoboPilot 项目页面 <https://aha-robot.notion.site/MuJoCo-RoboPilot-28333900bc8780c486f5d114ae5a719d>`_
- `GitHub 仓库 <https://github.com/apolloil/robopilot>`_

索引和表格
==========

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
