安装指南
========

本指南将帮助您在系统上安装和配置 AhaRobot 机器人遥操作系统。

系统要求
--------

操作系统
~~~~~~~~

- Windows 10/11
- Ubuntu 18.04+ 
- macOS 10.15+

Python 版本
~~~~~~~~~~~

- Python 3.8 或更高版本

硬件要求
~~~~~~~~

- 至少 8GB RAM
- 支持 OpenGL 3.3+ 的显卡
- USB 摄像头（用于视觉跟踪）

依赖安装
--------

1. 克隆仓库
~~~~~~~~~~~

.. code-block:: bash

   git clone https://github.com/apolloil/robopilot.git
   cd robopilot/main_pr

2. 创建虚拟环境（推荐）
~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # 使用 venv
   python -m venv venv
   source venv/bin/activate  # Linux/macOS
   # 或
   venv\Scripts\activate     # Windows

   # 使用 conda
   conda create -n aharobot python=3.11
   conda activate aharobot

3. 安装基础依赖
~~~~~~~~~~~~~~~

.. code-block:: bash

   pip install --upgrade pip
   pip install -r requirements.txt

4. 安装修改的第三方包
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # 安装 URDF 加载器
   pip install ./modified_pkg/mr_urdf_loader/
   
   # 安装 URDF 处理库
   pip install ./modified_pkg/urchin/

系统特定安装
------------

Windows
~~~~~~~

1. 安装 Visual Studio Build Tools（如果遇到编译错误）
2. 确保安装了 Microsoft Visual C++ 14.0 或更高版本

.. code-block:: bash

   # 如果遇到 OpenCV 安装问题
   pip install opencv-python-headless

Ubuntu/Debian
~~~~~~~~~~~~~

.. code-block:: bash

   # 安装系统依赖
   sudo apt-get update
   sudo apt-get install -y \
       libgl1-mesa-glx \
       libglib2.0-0 \
       libsm6 \
       libxext6 \
       libxrender-dev \
       libgomp1 \
       libglu1-mesa-dev

macOS
~~~~~

.. code-block:: bash

   # 使用 Homebrew 安装依赖
   brew install opencv
   brew install glfw

验证安装
--------

运行以下命令验证安装是否成功：

.. code-block:: python

   import mujoco
   import cv2
   import numpy as np
   from pytransform3d import transformations as pt
   import modern_robotics as mr
   
   print("所有依赖安装成功！")

运行测试
--------

.. code-block:: bash

   # 测试 MuJoCo 模型加载
   python -c "import mujoco; model = mujoco.MjModel.from_xml_path('model/astra.xml'); print('模型加载成功！')"
   
   # 测试摄像头
   python -c "import cv2; cap = cv2.VideoCapture(0); print('摄像头可用！' if cap.isOpened() else '摄像头不可用！')"

故障排除
--------

常见问题
~~~~~~~~

1. **MuJoCo 许可证问题**
   - 确保已正确安装 MuJoCo 许可证
   - 检查环境变量 MUJOCO_LICENSE_PATH

2. **OpenCV 导入错误**
   - 尝试安装 opencv-python-headless 版本
   - 检查系统图形库依赖

3. **编译错误**
   - 确保安装了完整的编译工具链
   - 在 Windows 上安装 Visual Studio Build Tools

4. **摄像头访问问题**
   - 检查摄像头权限设置
   - 确保没有其他程序占用摄像头

获取帮助
--------

如果遇到安装问题，请：

1. 查看 `GitHub Issues <https://github.com/apolloil/robopilot/issues>`_
2. 参考 `MuJoCo 官方文档 <https://mujoco.readthedocs.io/>`_
3. 联系开发团队
