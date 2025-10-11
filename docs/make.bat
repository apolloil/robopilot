@echo off
REM AhaRobot 机器人遥操作系统文档构建批处理文件

if "%1" == "help" goto help
if "%1" == "html" goto html
if "%1" == "clean" goto clean
if "%1" == "livehtml" goto livehtml
if "%1" == "pdf" goto pdf
if "%1" == "epub" goto epub
if "%1" == "install" goto install
if "%1" == "dev-setup" goto dev-setup

:help
echo 可用的构建命令:
echo   make.bat html        - 构建HTML文档
echo   make.bat clean       - 清理构建文件
echo   make.bat livehtml    - 启动实时预览服务器
echo   make.bat pdf         - 构建PDF文档
echo   make.bat epub        - 构建EPUB文档
echo   make.bat install     - 安装依赖
echo   make.bat dev-setup   - 设置开发环境
echo   make.bat help        - 显示此帮助信息
goto end

:html
sphinx-build -W -b html source _build/html
echo HTML文档构建完成，请查看 _build/html/index.html
goto end

:clean
if exist _build rmdir /s /q _build
echo 构建文件已清理
goto end

:livehtml
sphinx-autobuild source _build/html --host 0.0.0.0 --port 8000
goto end

:pdf
sphinx-build -b latex source _build/latex
echo 运行LaTeX构建PDF...
cd _build/latex
pdflatex *.tex
cd ..\..
echo PDF文档构建完成，请查看 _build/latex/*.pdf
goto end

:epub
sphinx-build -b epub source _build/epub
echo EPUB文档构建完成，请查看 _build/epub/*.epub
goto end

:install
pip install sphinx sphinx-rtd-theme sphinx-autodoc-typehints
pip install sphinx-autobuild
echo 依赖安装完成
goto end

:dev-setup
call :install
echo 开发环境设置完成
echo 运行 'make.bat livehtml' 启动实时预览服务器
goto end

:end
