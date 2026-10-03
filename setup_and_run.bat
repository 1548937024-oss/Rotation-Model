@echo off
setlocal
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" goto install_deps

set "PY_CMD=py -3"
where py >nul 2>&1
if not errorlevel 1 goto have_python

set "PY_CMD="
if exist "%LOCALAPPDATA%\Programs\Python\Python312\python.exe" set "PY_CMD=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
if exist "%LOCALAPPDATA%\Programs\Python\Python311\python.exe" set "PY_CMD=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
if not defined PY_CMD goto no_python

:have_python
echo Creating the local Python environment...
%PY_CMD% -m venv ".venv"
if errorlevel 1 goto failed

:install_deps
echo Checking runtime dependencies...
".venv\Scripts\python.exe" -m pip install --disable-pip-version-check -r requirements.txt
if errorlevel 1 goto failed

if /i "%~1"=="--check" goto check_only

".venv\Scripts\python.exe" main.py
exit /b %errorlevel%

:check_only
".venv\Scripts\python.exe" -c "import tkinter, serial; print('Launcher check passed.')"
exit /b %errorlevel%

:no_python
echo Python 3.11 or newer was not found.
echo Install Python and enable the "Add Python to PATH" option.
pause
exit /b 1

:failed
echo.
echo Setup or startup failed. Keep the error message shown above.
pause
exit /b 1
