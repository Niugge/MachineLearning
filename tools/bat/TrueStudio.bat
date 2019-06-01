@echo off

@echo start compiling...

set WorkSpace_PATH=".\workspace"

set ProjectFolder=".."

set PLUG_IN_EXE=TrueSTUDIOc.exe --launcher.suppressErrors ^
-nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild %*

rem headless.bat -data %WorkSpace_PATH% -importAll %ProjectFolder% -build all

%PLUG_IN_EXE% -data %WorkSpace_PATH% -importAll %ProjectFolder% 
@if %ERRORLEVEL% == 0 (@echo import project Successfully) else  (@echo import project Failed & goto FAILED)


@echo. & @echo compile lib1...
%PLUG_IN_EXE% -data %WorkSpace_PATH% -build lib1/Release
@if %ERRORLEVEL% == 0 (@echo build lib1 Successfully) else (
@echo build lib1 Failed & goto FAILED)

@echo. & @echo compile lib2...
%PLUG_IN_EXE% -data %WorkSpace_PATH% -build lib2/Release
@if %ERRORLEVEL% == 0 (@echo build lib2 Successfully) else (
@echo build lib2 Failed & goto FAILED)

@echo. & @echo compile lib3...
%PLUG_IN_EXE% -data %WorkSpace_PATH% -build lib3/Release
@if %ERRORLEVEL% == 0 (@echo build lib3 Successfully) else (
@echo build lib3 Failed & goto FAILED)

@echo. & @echo compile pro...
%PLUG_IN_EXE% -data %WorkSpace_PATH% -build pro/Release
@if %ERRORLEVEL% == 0 (@echo build pro Successfully) else (
@echo build pro Failed & goto FAILED)

rem @rd /q /s %WorkSpace_PATH%

goto SUCCESS

:FAILED
echo. & echo ±‡“Î ß∞‹£° & echo. 
pause
exit 1

:SUCCESS
echo. & echo ±‡“Î≥…π¶£° & echo.  
pause
exit 0
