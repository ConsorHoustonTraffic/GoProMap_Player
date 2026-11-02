@echo off
REM Builds GoProMapPlayer.exe. Needs Python 3.10+ on THIS machine only (python.org).
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py -3

echo Creating build environment...
%PY% -m venv .venv || goto :err
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip || goto :err
python -m pip install -r requirements.txt || goto :err

echo Building...
python build.py || goto :err
echo.
echo Finished. Copy the zip in the "dist" folder to any Windows laptop, unzip, run GoProMapPlayer.exe
pause
exit /b 0

:err
echo.
echo BUILD FAILED - see messages above.
pause
exit /b 1
