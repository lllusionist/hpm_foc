@echo off
rem FOC Scope launcher (tkinter)
rem Double-click to run.
rem To hide the console window, change python to pythonw below.
cd /d "%~dp0"
python foc_scope.py
if errorlevel 1 pause
