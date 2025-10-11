# AhaRobot 机器人遥操作系统文档构建 Makefile

# 变量定义
SPHINXOPTS    = -W -b html
SPHINXBUILD   = sphinx-build
SOURCEDIR     = source
BUILDDIR      = _build

# 帮助信息
help:
	@echo "可用的构建命令:"
	@echo "  make html        - 构建HTML文档"
	@echo "  make clean       - 清理构建文件"
	@echo "  make livehtml    - 启动实时预览服务器"
	@echo "  make pdf         - 构建PDF文档"
	@echo "  make epub        - 构建EPUB文档"
	@echo "  make help        - 显示此帮助信息"

# 构建HTML文档
html:
	$(SPHINXBUILD) $(SPHINXOPTS) $(SOURCEDIR) $(BUILDDIR)/html
	@echo "HTML文档构建完成，请查看 $(BUILDDIR)/html/index.html"

# 清理构建文件
clean:
	rm -rf $(BUILDDIR)/*

# 实时预览（需要安装sphinx-autobuild）
livehtml:
	sphinx-autobuild $(SOURCEDIR) $(BUILDDIR)/html --host 0.0.0.0 --port 8000

# 构建PDF文档
pdf:
	$(SPHINXBUILD) -b latex $(SOURCEDIR) $(BUILDDIR)/latex
	@echo "运行LaTeX构建PDF..."
	$(MAKE) -C $(BUILDDIR)/latex all-pdf
	@echo "PDF文档构建完成，请查看 $(BUILDDIR)/latex/*.pdf"

# 构建EPUB文档
epub:
	$(SPHINXBUILD) -b epub $(SOURCEDIR) $(BUILDDIR)/epub
	@echo "EPUB文档构建完成，请查看 $(BUILDDIR)/epub/*.epub"

# 检查链接
linkcheck:
	$(SPHINXBUILD) -b linkcheck $(SOURCEDIR) $(BUILDDIR)/linkcheck
	@echo "链接检查完成，请查看 $(BUILDDIR)/linkcheck/output.txt"

# 检查拼写
spelling:
	$(SPHINXBUILD) -b spelling $(SOURCEDIR) $(BUILDDIR)/spelling
	@echo "拼写检查完成，请查看 $(BUILDDIR)/spelling/output.txt"

# 安装依赖
install:
	pip install sphinx sphinx-rtd-theme sphinx-autodoc-typehints
	pip install sphinx-autobuild  # 用于实时预览

# 开发环境设置
dev-setup: install
	@echo "开发环境设置完成"
	@echo "运行 'make livehtml' 启动实时预览服务器"

.PHONY: help html clean livehtml pdf epub linkcheck spelling install dev-setup
