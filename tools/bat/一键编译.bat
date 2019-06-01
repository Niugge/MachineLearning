@echo off
::color 0A

set PROJECT=..\App\UserApp
set RCNAME=remote
set BOOTNAME=bootloader
set BOOTVER=V0.0.1.14_20180731
SET APP_IROM1_Start=0x0800A800
SET APP_IROM1_Size=0x00014000
SET IRAM1_Start=0x200000C8
SET IRAM1_Size=0x00003F38
SET IRAM2_Start=0x20000000
SET IRAM2_Size=0x000000C8

SET HARDWAREVERSION=0.0.1.4
SET LOADERVERSION=0.0.1.14
SET APPVERSION=0.0.29.0
SET HIGHVERSION=0.0.3.20
SET LOWVERSION=0.0.1.0

SET PATH=
 if exist C:\Keil_v5\ARM (SET PATH=C:\Keil_v5\ARM)

set ARMASM="%PATH%\ARMCC\bin\armasm.exe"
set ARMCC="%PATH%\ARMCC\bin\armcc.exe"
set ARMLINK="%PATH%\ARMCC\bin\armlink.exe"
set FROMELF="%PATH%\ARMCC\bin\fromelf.exe"

set ASM_OPTION=--cpu Cortex-M0 -g --apcs=interwork --pd "__MICROLIB SETA 1"
::set ASM_OPTION=%ASM_OPTION% -I %PROJECT%\pro\RTE
::set ASM_OPTION=%ASM_OPTION% -I %PATH%\PACK\Keil\STM32F0xx_DFP\1.5.0\Device\Include
::set ASM_OPTION=%ASM_OPTION% -I %PATH%\CMSIS\Include
set ASM_OPTION=%ASM_OPTION% --pd "__UVISION_VERSION SETA 517" --pd "STM32F072xB SETA 1" --list "..\obj\*.lst" --xref -o "..\obj\*.o" --depend "..\obj\*.d"

set CC_OPTION=--c99 -c --cpu Cortex-M0 -D__MICROLIB -g -O0 --apcs=interwork --split_sections -I..\cmsis -I..\..\DriverLib\STM32F0xx_StdPeriph_Driver\inc
set CC_OPTION=%CC_OPTION% --gnu -o1 -g -W
set CC_OPTION=%CC_OPTION% -I..\src\usb\inc -I..\..\DriverLib\STM32_USB_Device_Driver\inc -I..\inc -I..\..\STM32F0xx_StdPeriph_Driver\inc
::set CC_OPTION=%CC_OPTION% -I..\..\DriverLib\STM32_USB_Device_Library\Core\inc -I..\..\DriverLib\STM32_USB_Device_Library\Class\cdc\inc
::set CC_OPTION=%CC_OPTION% -I..\inc\alink -I..\inc\alink\xstar_2_0 -I..\..\DriverLib\CMSIS\Include
set CC_OPTION=%CC_OPTION% -I..\inc -I..\inc\alink -I..\inc\alink\xstar_2_0 -I..\..\..\inc\ -I..\..\DriverLib\CMSIS\Include -I..\..\STM32_USB_Device_Library\Core\inc
set CC_OPTION=%CC_OPTION% -I.\usb\inc -I..\..\DriverLib\STM32_USB_Device_Driver\inc -I..\..\DriverLib\STM32_USB_Device_Library\Core\inc -I..\..\DriverLib\STM32_USB_Device_Library\Class\cdc\inc
set CC_OPTION=%CC_OPTION% -I..\APP\UserApp\cmsis -I..\..\..\UserApp\cmsis -I..\..\CMSIS\Include -I..\..\..\UserApp\src\usb\inc -I..\..\..\STM32_USB_Device_Driver\inc\..\..\..\CMSIS\Include -I..\..\..\..\STM32F0xx_StdPeriph_Driver\inc -I..\..\..\..\STM32_USB_Device_Driver\inc -I..\..\..\..\..\UserApp\inc -I..\..\..\..\..\UserApp\inc\alink -I..\..\..\..\..\UserApp\inc\alink\xstar_2_0
set CC_OPTION=%CC_OPTION% -I..\..\..\..\DriverLib\STM32_USB_Device_Library\Class\cdc\inc -I..\..\..\..\DriverLib\CMSIS\Include -I..\..\..\..\DriverLib\STM32F0xx_StdPeriph_Driver\inc -I..\..\..\..\DriverLib\STM32_USB_Device_Driver\inc

set CC_OPTION=%CC_OPTION% -D__UVISION_VERSION="517" -DSTM32F072xB -DSTM32F072 -DUSE_STDPERIPH_DRIVER -o "..\obj\*.o" --omf_browse "..\obj\*.crf" --depend "..\obj\*.d"
set CC_OPTION=%CC_OPTION% -I..\..\..\..\UserApp\src\usb\inc -I..\..\..\cmsis -I..\..\..\..\UserApp\cmsis -I..\..\..\CMSIS\Include -I..\..\..\STM32F0xx_StdPeriph_Driver\inc
set CC_OPTION=%CC_OPTION% -I..\..\..\..\..\UserApp\src\usb\inc -I..\..\..\..\DriverLib\STM32_USB_Device_Library\Core\inc -I..\..\..\Core\inc -I..\..\..\..\..\UserApp\cmsis -I..
::set CC_OPTION=%CC_OPTION% -I %PATH%\RV31\INC
::set CC_OPTION=%CC_OPTION% -I %PATH%\Pack\ARM\CMSIS\5.0.1\CMSIS\Include
::set CC_OPTION=%CC_OPTION% -I %PATH%\INC\ST\STM32F0xx

set startup=startup_stm32f072

cd %PROJECT%

del /s/q .\obj

cd .\cmsis
%ARMASM% %startup%.s %ASM_OPTION% --list "..\obj\%startup%.lst" --xref -o "..\obj\%startup%.o" --depend "..\obj\%startup%.d"
echo compiling %startup%.s
if NOT %errorlevel% EQU 0 (
	goto FAILED
)

cd ..\src
for %%j in (*.c) do (@echo %%j>temp.txt
  for /f "delims=." %%i in (temp.txt) do (
    %ARMCC% %%j %CC_OPTION% -o "..\obj\%%i.o" --omf_browse "..\obj\%%i.crf" --depend "..\obj\%%i.d"
	  echo compiling %%j
	  if NOT %errorlevel% EQU 0   (
		goto FAILED
	  )
  )
)

cd ..\cmsis
for %%j in (*.c) do (@echo %%j>temp.txt
  for /f "delims=." %%i in (temp.txt) do (
    %ARMCC% %%j %CC_OPTION% -o "..\obj\%%i.o" --omf_browse "..\obj\%%i.crf" --depend "..\obj\%%i.d"
	  echo compiling %%j
	  if NOT %errorlevel% EQU 0   (
		goto FAILED
	  )
  )
)



cd ..\..\DriverLib\STM32F0xx_StdPeriph_Driver\src
for %%j in (*.c) do (@echo %%j>temp.txt
  for /f "delims=." %%i in (temp.txt) do (
    %ARMCC% %%j %CC_OPTION% -o "..\..\..\UserApp\obj\%%i.o" --omf_browse "..\..\..\UserApp\obj\%%i.crf" --depend "..\..\..\UserApp\obj\%%i.d"
	  echo compiling %%j
	  if NOT %errorlevel% EQU 0   (
		goto FAILED
	  )
  )
)



echo. & echo linking...

cd ..\..\..\UserApp\obj

echo ; *************************************************************>%RCNAME%.sct
echo ; *** Scatter-Loading Description File generated by uVision ***>>%RCNAME%.sct
echo ; *************************************************************>>%RCNAME%.sct
echo.>>%RCNAME%.sct
echo LR_IROM1 %APP_IROM1_Start% %APP_IROM1_Size%  {    ; load region size_region>>%RCNAME%.sct
echo   ER_IROM1 %APP_IROM1_Start% %APP_IROM1_Size%  {  ; load address = execution address>>%RCNAME%.sct
echo    *.o (RESET, +First)>>%RCNAME%.sct
echo    *(InRoot$$Sections)>>%RCNAME%.sct
echo    .ANY (+RO)>>%RCNAME%.sct
echo   }>>%RCNAME%.sct
echo   RW_IRAM1 %IRAM1_Start% %IRAM1_Size%  {  ; RW data>>%RCNAME%.sct
echo    .ANY (+RW +ZI)>>%RCNAME%.sct
echo  }>>%RCNAME%.sct
echo  RW_IRAM2 %IRAM2_Start% UNINIT %IRAM2_Size%  {  ; RW data>>%RCNAME%.sct
echo   .ANY (+RW +ZI)>>%RCNAME%.sct
echo   }>>%RCNAME%.sct
echo }>>%RCNAME%.sct

echo --cpu Cortex-M0>%RCNAME%.lnp
for %%i in (*.o) do (
  @echo "..\obj\%%i">>%RCNAME%.lnp
  echo linking %%i
)
echo "..\..\DriverLib\stm32_cryp_lib\binary\MK-ARM\M0_CryptoFW_2_0_6.lib">>%RCNAME%.lnp
echo --library_type=microlib --strict --scatter "..\obj\%RCNAME%.sct">>%RCNAME%.lnp
echo --summary_stderr --info summarysizes --map --xref --callgraph --symbols>>%RCNAME%.lnp
echo --info sizes --info totals --info unused --info veneers>>%RCNAME%.lnp
echo --list "..\obj\%RCNAME%.map" -o ..\obj\%RCNAME%.axf>>%RCNAME%.lnp

%ARMLINK% --Via ".\%RCNAME%.lnp"
if NOT %errorlevel% EQU 0 (
	goto FAILED
)

echo. & echo FromELF: creating hex file...
%FROMELF% "..\obj\%RCNAME%.axf" --i32combined --output "..\obj\%RCNAME%.hex"
if NOT %errorlevel% EQU 0 (
	goto FAILED
)

echo. & echo After Build - User command #1: fromelf.exe --bin -o ..\obj\%RCNAME%.bin ..\obj\%RCNAME%.axf
%FROMELF% --bin -o ..\obj\%RCNAME%.bin ..\obj\%RCNAME%.axf
if NOT %errorlevel% EQU 0 (
	goto FAILED
)

cd ..\..\..\一键编译
del /q .\releases
::del /q .\src
copy %PROJECT%\obj\%RCNAME%.bin .\src
::del .\src\%RCNAME%_V%APPVERSION%.bin
::rename .\src\%RCNAME%.bin %RCNAME%_V%APPVERSION%.bin

rem 定义Application文件路径和文件名（文件格式必须是二进制格式）
set APP_PATHNAME=.\src\%RCNAME%.bin

rem 定义生成的目标文件保存的路径和文件名（生成的目标文件格式为二进制格式）
rem set TARGET_PATHNAME=%OUTPUT%\Rc_App
set TARGET_PATHNAME=.\releases\Rc_App

rem 设备ID
set DEVICE_ID=14

rem 硬件版本号
set HW_VERSION=%HARDWAREVERSION%

rem bootlader软件版本号
set BOOT_SW_VERSION=%LOADERVERSION%

rem App软件版本号
set APP_SW_VERSION=%APPVERSION%

rem 统一发布软件版本号，即固件支持的最高硬件版本号
set RLS_SW_VERSION=%HIGHVERSION%

rem 统一发布硬件版本号，即固件支持的最低硬件版本号
set RLS_HW_VERSION=%LOWVERSION%

rem 产品（设备）类型信息(最长64个字符)
set MANUFACTURE="RC-PRO"

rem IMAGE打包工具
set TOOL_IMAGE=".\tools\BinLinker.exe"

%TOOL_IMAGE% %APP_PATHNAME% %TARGET_PATHNAME% %DEVICE_ID% %HW_VERSION% %BOOT_SW_VERSION% %APP_SW_VERSION% %RLS_SW_VERSION% %RLS_HW_VERSION% %MANUFACTURE%

::======================================================镜像文件==============================================================::

set PROJECT=..\Bootloader\UserApp
set BOOT=boot
copy %PROJECT%\obj\%BOOT%.bin .\src
::del /q .\src\%RCNAME%_V%APPVERSION%.bin
del /q .\src\%BOOTNAME%.bin
rename .\src\%BOOT%.bin %BOOTNAME%.bin

copy .\src\%BOOTNAME%.bin .\releases
rename .\releases\%BOOTNAME%.bin %BOOTNAME%__%BOOTVER%.bin


rem 定义boot文件路径和文件名（文件格式必须是二进制格式）
set BOOT_PATHNAME=.\src\%BOOTNAME%.bin

rem 定义Application文件路径和文件名（文件格式必须是二进制格式）
set APP_PATHNAME=.\src\%RCNAME%.bin

rem 定义生成的目标文件保存的路径和文件名（生成的目标文件格式为二进制格式）
rem set TARGET_PATHNAME=%OUTPUT%\Rc_App
set TARGET_PATHNAME=.\releases\Rc_Image

rem 设备ID
set DEVICE_ID=14

rem 定义Bootconfig在Flash中的偏移地址（十六进制格式）
set CFG_ADDRESS_OFFSET=0xA000

rem 定义Application在Flash中的偏移地址（十六进制格式）
set APP_ADDRESS_OFFSET=0xA800

rem Bootloader硬件版本号,即硬件版本号
set BOOT_HW_VERSION=%HARDWAREVERSION%

rem Bootloader软件版本号
set BOOT_SW_VERSION=%LOADERVERSION%

rem App硬件版本号
set APP_HW_VERSION=%HARDWAREVERSION%

rem App软件版本号
set APP_SW_VERSION=%APPVERSION%

rem 统一发布软件版本号，即固件支持的最高硬件版本号
set HW_HIGHEST_VERSION=%HIGHVERSION%

rem 统一发布硬件版本号，即固件支持的最低硬件版本号
set HW_LOWEST_VERSION=%LOWVERSION%

rem 产品（设备）类型信息(最长64个字符)
set MANUFACTURE="RC-PRO"

rem 产品序列号(最长31个字符)
set SERIAL_NUMBER=""

rem IMAGE打包工具
set TOOL_IMAGE=".\tools\BinLinker.exe"
::set TOOL_IMAGE=".\tools\BinLinkerNew.exe"



%TOOL_IMAGE% %BOOT_PATHNAME% %APP_PATHNAME% %TARGET_PATHNAME% %DEVICE_ID% %CFG_ADDRESS_OFFSET% %APP_ADDRESS_OFFSET% ^
            %BOOT_HW_VERSION% %BOOT_SW_VERSION% %APP_HW_VERSION% %APP_SW_VERSION% %HW_HIGHEST_VERSION% %HW_LOWEST_VERSION% ^
			%MANUFACTURE% %SERIAL_NUMBER%





::------------------------------------------------------------------------------------------------------------------::
goto SUCCESS

:FAILED
echo. & echo 编译失败！
pause
exit 1

:SUCCESS
echo. & echo 编译成功！
pause
exit 0

pause



cd ..\src\usb\src
for %%j in (*.c) do (@echo %%j>temp.txt
  for /f "delims=." %%i in (temp.txt) do (
    %ARMCC% %%j %CC_OPTION% -o "..\..\..\obj\%%i.o" --omf_browse "..\..\..\obj\%%i.crf" --depend "..\..\..\obj\%%i.d"
	  echo compiling %%j
	  if NOT %errorlevel% EQU 0   (
		goto FAILED
	  )
  )
)

cd ..\..\STM32_USB_Device_Driver\src
for %%j in (*.c) do (@echo %%j>temp.txt
  for /f "delims=." %%i in (temp.txt) do (
    %ARMCC% %%j %CC_OPTION% -o "..\..\..\UserApp\obj\%%i.o" --omf_browse "..\..\..\UserApp\obj\%%i.crf" --depend "..\..\..\UserApp\obj\%%i.d"
	  echo compiling %%j
	  if NOT %errorlevel% EQU 0   (
		goto FAILED
	  )
  )
)

cd ..\..\STM32_USB_Device_Library\Core\src
for %%j in (*.c) do (@echo %%j>temp.txt
  for /f "delims=." %%i in (temp.txt) do (
    %ARMCC% %%j %CC_OPTION% -o "..\..\..\..\UserApp\obj\%%i.o" --omf_browse "..\..\..\..\UserApp\obj\%%i.crf" --depend "..\..\..\..\UserApp\obj\%%i.d"
	  echo compiling %%j
	  if NOT %errorlevel% EQU 0   (
		goto FAILED
	  )
  )
)

cd ..\..\..\STM32_USB_Device_Library\Class\cdc\src
for %%j in (*.c) do (@echo %%j>temp.txt
  for /f "delims=." %%i in (temp.txt) do (
    %ARMCC% %%j %CC_OPTION% -o "..\..\..\..\..\UserApp\obj\%%i.o" --omf_browse "..\..\..\..\..\UserApp\obj\%%i.crf" --depend "..\..\..\..\..\UserApp\obj\%%i.d"
	  echo compiling %%j
	  if NOT %errorlevel% EQU 0   (
		goto FAILED
	  )
  )
)