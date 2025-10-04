# AhaRobot 机器人遥操作项目

这是一个基于 MuJoCo 物理仿真的 Aharobot 机器人遥操作系统项目。

## 项目结构

```
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
│   └── calibration_results_test.yaml
├── modified_pkg/          # 修改的第三方包
│   ├── mr_urdf_loader/    # URDF 加载器
│   └── urchin/            # URDF 处理库
└── requirements.txt       # 项目依赖
```

## 功能特性

- 🤖 Aharobot 机器人模型仿真
- 🎮 遥操作控制接口
- 👁️ 计算机视觉跟踪
- 🔧 逆运动学求解
- 📐 机器人标定支持

## 依赖要求

- Python 3.8+
- MuJoCo 3.3.0+
- OpenCV 4.12.0+
- NumPy 2.0.0+
- PyTransform3D 3.14.0+

## 安装

1. 克隆仓库：
```bash
git clone https://github.com/apolloil/robopilot.git
```

2. 安装依赖：
```bash
cd robopilot
pip install -r requirements.txt
pip install .\modified_pkg\mr_urdf_loader\
pip install .\modified_pkg\urchin\  
```


## 使用方法

运行主程序：
```bash
python src/main_teleop.py
```

## 模型文件

项目包含完整的 Aharobot 机器人模型，包括：
- MuJoCo XML 格式的物理仿真模型（注意：通过damping设置，收束了末端夹爪的位置）
- URDF 格式的机器人描述文件

## 更多信息
详见[在Mujoco中测试RoboPilot](https://www.notion.so/aha-robot/Mujoco-RoboPilot-28333900bc8780c486f5d114ae5a719d)

