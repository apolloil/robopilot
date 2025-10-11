# -*- coding: utf-8 -*-
"""
Sphinx配置文件 - AhaRobot 机器人遥操作系统文档
"""

import os
import sys
import sphinx_rtd_theme

# 添加项目根目录到Python路径
sys.path.insert(0, os.path.abspath('../../src'))
sys.path.insert(0, os.path.abspath('../../'))

# 项目信息
project = 'AhaRobot 机器人遥操作系统'
copyright = '2024, AhaRobot Team'
author = 'AhaRobot Team'
release = '1.0.0'
version = '1.0'

# 扩展配置
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.todo',
    'sphinx.ext.coverage',
    'sphinx.ext.mathjax',
    'sphinx.ext.ifconfig',
    'sphinx.ext.viewcode',
    'sphinx.ext.githubpages',
    'sphinx.ext.napoleon',
    'sphinx_rtd_theme',
]

# 自动生成文档
autosummary_generate = True
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__'
}

# 模板路径
templates_path = ['_templates']

# 源文件模式
source_suffix = '.rst'

# 主文档
master_doc = 'index'

# 语言设置
language = 'zh_CN'

# 排除的文件
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# HTML输出选项
html_theme = 'sphinx_rtd_theme'
html_theme_options = {
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    'vcs_pageview_mode': '',
    'style_nav_header_background': '#2980B9',
    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# 静态文件路径
html_static_path = ['_static']

# 自定义CSS
html_css_files = [
    'custom.css',
]

# 侧边栏配置
html_sidebars = {
    '**': [
        'relations.html',
        'sourcelink.html',
        'searchbox.html',
    ]
}

# 数学公式支持
mathjax_path = 'https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.7/MathJax.js?config=TeX-AMS-MML_HTMLorMML'

# 交叉引用配置
intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'opencv': ('https://docs.opencv.org/4.x/', None),
    'mujoco': ('https://mujoco.readthedocs.io/en/stable/', None),
}

# TODO扩展配置
todo_include_todos = True

# 代码高亮
pygments_style = 'sphinx'

# 文档字符串格式
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = False
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True
napoleon_use_admonition_for_examples = False
napoleon_use_admonition_for_notes = False
napoleon_use_admonition_for_references = False
napoleon_use_ivar = False
napoleon_use_param = True
napoleon_use_rtype = True
napoleon_preprocess_types = False
napoleon_type_aliases = None
napoleon_attr_annotations = True
