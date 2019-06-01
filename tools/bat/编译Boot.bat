@echo OFF

set COMPILER=C:\Keil\UV4\UV4.exe

::检查编译器
if not exist %COMPILER%  (echo 找不到编译器,请先安装Keil v4.74  &  goto exit)

::清空工程
echo 清空工程...
call clean.bat
echo.

::编译Bootloader
cd BOOT_5.8G_A5130\mdk\
echo 编译Bootloader...
%COMPILER% -r "5.8G M0 bootloader.uvproj" -t"RF_TX" -o "../conf_tx/bin/log.txt"
IF ERRORLEVEL 2 ( ECHO Error%ERRORLEVEL%,编译Boot_TX失败！ & GOTO error )
%COMPILER% -r "5.8G M0 bootloader.uvproj" -t"RF_RX" -o "../conf_rx/bin/log.txt"
IF ERRORLEVEL 2 ( ECHO Error%ERRORLEVEL%,编译Boot_RX失败！ & GOTO error )
cd ..\..\

::拷贝生成的文件
if not exist BOOT_5.8G_A5130\conf_tx\bin\boot.bin ( echo 找不到生成的文件 & goto exit)
if not exist BOOT_5.8G_A5130\conf_rx\bin\boot.bin ( echo 找不到生成的文件 & goto exit)
copy BOOT_5.8G_A5130\conf_tx\bin\boot.bin Release\
ren  Release\boot.bin boot_tx.bin
copy BOOT_5.8G_A5130\conf_rx\bin\boot.bin Release\
ren  Release\boot.bin boot_rx.bin
goto exit

:error
cd ..\..\

:exit
echo.
echo.
echo 结束! & pause > nul


