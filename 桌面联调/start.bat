@echo off
cd /d "%~dp0"
echo [桌面联调] %CD%

REM 清掉旧 8765/8766
for %%P in (8765 8766) do (
  for /f "tokens=5" %%a in ('netstat -ano ^| findstr ":%%P" ^| findstr "LISTENING"') do (
    echo kill PID=%%a port=%%P
    taskkill /F /PID %%a >nul 2>&1
  )
)

python -m pip install -r requirements.txt -q
echo.
echo ========================================
echo  浏览器请打开: http://127.0.0.1:8766/
echo  查版本:       http://127.0.0.1:8766/api/health
echo  （注意端口是 8766，不是 8765）
echo ========================================
echo.
python server.py
pause
