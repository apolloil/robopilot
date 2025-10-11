贡献指南
========

感谢您对 AhaRobot 机器人遥操作系统项目的关注！我们欢迎各种形式的贡献。

如何贡献
--------

报告问题
~~~~~~~~

如果您发现了 bug 或有功能建议：

1. 查看 `GitHub Issues <https://github.com/apolloil/robopilot/issues>`_ 确认问题未被报告
2. 创建新的 issue，包含：
   - 详细的问题描述
   - 复现步骤
   - 系统环境信息
   - 相关日志或截图

提交代码
~~~~~~~~

1. Fork 仓库到您的 GitHub 账户
2. 创建功能分支：

   .. code-block:: bash

      git checkout -b feature/your-feature-name

3. 提交更改：

   .. code-block:: bash

      git commit -m "Add: 描述您的更改"

4. 推送到您的分支：

   .. code-block:: bash

      git push origin feature/your-feature-name

5. 创建 Pull Request

代码规范
--------

Python 代码风格
~~~~~~~~~~~~~~~

我们使用以下工具确保代码质量：

- **Black**: 代码格式化
- **Flake8**: 代码检查
- **MyPy**: 类型检查

运行代码检查：

.. code-block:: bash

   # 格式化代码
   black src/
   
   # 检查代码风格
   flake8 src/
   
   # 类型检查
   mypy src/

命名约定
~~~~~~~~

- 类名：PascalCase (``VisionTracker``)
- 函数和变量：snake_case (``get_target_position``)
- 常量：UPPER_CASE (``MAX_TORQUE``)
- 私有成员：前缀下划线 (``_private_method``)

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

测试要求
--------

所有新功能必须包含测试：

.. code-block:: python

   import pytest
   import numpy as np
   from src.your_module import YourClass
   
   def test_your_function():
       # 测试正常情况
       result = YourClass().your_method()
       assert result is not None
       
       # 测试边界情况
       with pytest.raises(ValueError):
           YourClass().your_method(invalid_input)

运行测试：

.. code-block:: bash

   pytest tests/

文档更新
--------

如果您添加了新功能，请同时更新：

1. **API 文档**: 在 ``docs/source/api/`` 中添加接口说明
2. **用户指南**: 在 ``docs/source/user_guide/`` 中添加使用说明
3. **示例代码**: 提供完整的使用示例
4. **更新日志**: 在 ``changelog.rst`` 中记录更改

提交信息规范
------------

使用清晰的提交信息：

.. code-block:: bash

   # 功能添加
   git commit -m "Add: 添加新的视觉跟踪算法"
   
   # Bug 修复
   git commit -m "Fix: 修复逆运动学求解中的数值稳定性问题"
   
   # 文档更新
   git commit -m "Docs: 更新安装指南"
   
   # 重构
   git commit -m "Refactor: 重构控制模块以提高性能"

Pull Request 指南
-----------------

创建 PR 时请确保：

1. **代码质量**: 通过所有代码检查
2. **测试覆盖**: 新功能有对应的测试
3. **文档完整**: 更新相关文档
4. **向后兼容**: 不破坏现有 API
5. **性能影响**: 考虑对系统性能的影响

PR 模板
~~~~~~~

.. code-block:: markdown

   ## 更改描述
   简要描述此 PR 的更改内容
   
   ## 更改类型
   - [ ] Bug 修复
   - [ ] 新功能
   - [ ] 文档更新
   - [ ] 重构
   - [ ] 性能优化
   
   ## 测试
   - [ ] 添加了新的测试
   - [ ] 所有测试通过
   - [ ] 手动测试完成
   
   ## 文档
   - [ ] 更新了 API 文档
   - [ ] 更新了用户指南
   - [ ] 更新了示例代码

开发环境设置
------------

1. 克隆仓库：

   .. code-block:: bash

      git clone https://github.com/apolloil/robopilot.git
      cd robopilot/main_pr

2. 安装依赖：

   .. code-block:: bash

      pip install -r requirements.txt
      pip install -e ./modified_pkg/mr_urdf_loader/
      pip install -e ./modified_pkg/urchin/

3. 安装开发工具：

   .. code-block:: bash

      pip install pytest black flake8 mypy sphinx

4. 运行测试：

   .. code-block:: bash

      pytest tests/

社区准则
--------

我们致力于创建一个友好、包容的社区环境：

- 尊重所有贡献者
- 建设性的反馈和讨论
- 帮助新成员学习
- 遵循项目规范

联系方式
--------

- **GitHub Issues**: 技术问题和功能建议
- **Discussions**: 一般讨论和问题
- **Email**: 联系开发团队

感谢您的贡献！
