# AhaRobot 机器人遥操作系统文档

本目录包含 AhaRobot 机器人遥操作系统的完整文档，使用 Sphinx 构建。

## 快速开始

### 安装依赖

```bash
# 安装文档构建工具
pip install sphinx sphinx-rtd-theme sphinx-autodoc-typehints

# 安装实时预览工具（可选）
pip install sphinx-autobuild
```

### 构建文档

#### Linux/macOS

```bash
# 构建HTML文档
make html

# 清理构建文件
make clean

# 启动实时预览服务器
make livehtml
```

#### Windows

```bash
# 构建HTML文档
make.bat html

# 清理构建文件
make.bat clean

# 启动实时预览服务器
make.bat livehtml
```

## 文档结构

```
docs/
├── source/                 # 文档源文件
│   ├── conf.py            # Sphinx配置文件
│   ├── index.rst          # 主页面
│   ├── installation.rst   # 安装指南
│   ├── quickstart.rst     # 快速开始
│   ├── changelog.rst      # 更新日志
│   ├── contributing.rst   # 贡献指南
│   ├── user_guide/        # 用户指南
│   │   └── index.rst
│   ├── api/               # API文档
│   │   └── index.rst
│   ├── development/       # 开发文档
│   │   └── index.rst
│   └── _static/           # 静态文件
│       └── custom.css     # 自定义样式
├── _build/                # 构建输出目录
├── Makefile              # Linux/macOS构建脚本
└── make.bat              # Windows构建脚本
```

## 部署到 Read the Docs

### 1. 配置文件

项目根目录的 `.readthedocs.yaml` 文件已配置好，包含：

- 构建环境设置
- 依赖安装命令
- 文档构建命令
- 版本管理配置

### 2. 连接 GitHub

1. 登录 [Read the Docs](https://readthedocs.org/)
2. 导入项目仓库
3. 选择 `main_pr` 目录作为文档根目录
4. 启用自动构建

### 3. 自定义域名（可选）

在 Read the Docs 项目设置中可以配置自定义域名。

## 本地开发

### 实时预览

启动实时预览服务器，文档会在文件更改时自动重新构建：

```bash
# Linux/macOS
make livehtml

# Windows
make.bat livehtml
```

然后在浏览器中访问 `http://localhost:8000`

### 添加新文档

1. 在 `source/` 目录下创建新的 `.rst` 文件
2. 在相应的 `index.rst` 文件中添加链接
3. 使用 Sphinx 语法编写内容

### 文档语法

使用 reStructuredText (RST) 语法：

```rst
标题
====

子标题
------

.. code-block:: python

   # Python代码示例
   def hello():
       print("Hello, World!")

.. note::

   这是一个提示框

.. warning::

   这是一个警告框
```

## 样式定制

自定义样式在 `source/_static/custom.css` 中定义，包括：

- 主题色彩
- 代码高亮
- 表格样式
- 响应式设计

## 故障排除

### 常见问题

1. **构建失败**
   - 检查 Python 版本（需要 3.8+）
   - 确认所有依赖已安装
   - 查看构建日志中的错误信息

2. **样式不生效**
   - 确认 `custom.css` 文件存在
   - 检查 `conf.py` 中的静态文件配置
   - 清理构建文件后重新构建

3. **链接检查失败**
   - 使用 `make linkcheck` 检查链接
   - 更新失效的链接

### 获取帮助

- 查看 [Sphinx 官方文档](https://www.sphinx-doc.org/)
- 参考 [Read the Docs 文档](https://docs.readthedocs.io/)
- 提交 GitHub Issue

## 贡献文档

欢迎为文档做出贡献！请参考 `contributing.rst` 中的指南。

### 文档标准

- 使用清晰、简洁的中文
- 提供完整的代码示例
- 包含必要的截图和图表
- 保持文档结构的一致性
