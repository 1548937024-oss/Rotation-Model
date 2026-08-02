@echo off
chcp 65001 >nul
setlocal
cd /d "%~dp0"

echo ========================================
echo   电机模组上位机 Windows EXE 构建程序
echo ========================================
echo.

where py >nul 2>&1
if errorlevel 1 (
  echo [失败] 未检测到 Python。
  echo 请安装 Python 3.11 或 3.12，并在安装时勾选 Add Python to PATH。
  pause
  exit /b 1
)

py -3 -c "import sys; raise SystemExit(0 if sys.version_info >= (3, 11) else 1)"
if errorlevel 1 (
  echo [失败] Python 版本过低，需要 Python 3.11 或更高版本。
  pause
  exit /b 1
)

if not exist ".venv-build\Scripts\python.exe" (
  echo [1/5] 创建独立构建环境...
  py -3 -m venv .venv-build
  if errorlevel 1 goto :failed
)

call ".venv-build\Scripts\activate.bat"
echo [2/5] 安装构建依赖...
python -m pip install --disable-pip-version-check --upgrade pip
if errorlevel 1 goto :failed
python -m pip install --disable-pip-version-check -r requirements-build.txt
if errorlevel 1 goto :failed

echo [3/5] 运行协议与模拟通信测试...
python -m unittest discover -s tests -v
if errorlevel 1 goto :failed

echo [4/5] 生成单文件 Windows EXE...
python -m PyInstaller --noconfirm --clean motor_module_host.spec
if errorlevel 1 goto :failed

if not exist "dist\电机模组上位机.exe" (
  echo [失败] PyInstaller 未生成目标 EXE。
  goto :failed
)

if not exist "release" mkdir "release"
copy /Y "dist\电机模组上位机.exe" "release\电机模组上位机.exe" >nul
copy /Y "README.md" "release\README.md" >nul

echo [5/5] 计算文件校验值...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$h=(Get-FileHash -Algorithm SHA256 'release\电机模组上位机.exe').Hash; Set-Content -Encoding ASCII 'release\SHA256.txt' ('SHA256  电机模组上位机.exe  '+$h); Write-Host ('SHA256: '+$h)"
if errorlevel 1 goto :failed

echo.
echo ========================================
echo 构建完成
echo EXE：release\电机模组上位机.exe
echo 校验：release\SHA256.txt
echo ========================================
explorer "release"
pause
exit /b 0

:failed
echo.
echo [构建失败] 请截图保留上方完整错误信息。
pause
exit /b 1
