@echo off
set PATH=C:\msys64\ucrt64\bin;%PATH%
cd /d "%~dp0"
if not exist build mkdir build
cd build
cmake .. -G "Ninja" -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b 1
cmake --build . -j 8
if errorlevel 1 exit /b 1
cd ..
if not exist dist mkdir dist
copy /Y build\SPMeshHealer.exe dist\ >nul
copy /Y C:\msys64\ucrt64\bin\libgmp-10.dll dist\ >nul
copy /Y C:\msys64\ucrt64\bin\libgomp-1.dll dist\ >nul
copy /Y C:\msys64\ucrt64\bin\libwinpthread-1.dll dist\ >nul
copy /Y C:\msys64\ucrt64\bin\libgcc_s_seh-1.dll dist\ >nul
if not exist zbrush_plugin\SPMeshHealerData mkdir zbrush_plugin\SPMeshHealerData
copy /Y dist\*.exe zbrush_plugin\SPMeshHealerData\ >nul
copy /Y dist\*.dll zbrush_plugin\SPMeshHealerData\ >nul
echo Build complete: dist\SPMeshHealer.exe
