@echo OFF

set COMPILER=C:\Keil\UV4\UV4.exe

::检查编译器
if not exist %COMPILER%  (echo 找不到编译器,请先安装Keil v4.74  &  goto exit)

::清空工程
echo 清空工程...
call clean.bat
echo.

::检查Bootloader
echo 检查是否存在Boot...
if not exist boot\boot_tx.bin ( echo 出错！请先编译Boot程序，然后拷贝boot_tx.bin到目录boot下 & goto exit )

::编译APP
cd APP_5.8G_A5130\mdk\
echo 编译App...
%COMPILER% -r "2.4G_AT86RF233.uvproj" -t"RF_TX" -o "../conf_tx/bin/log.txt"
IF ERRORLEVEL 2 ( ECHO Error%ERRORLEVEL%,编译App_TX失败！ & GOTO error )
cd ..\..\

::制作发布文件
cd APP_5.8G_A5130\bat\
echo 制作工厂烧录文件...
call TX_MakeImageFile.bat
echo 制作升级文件...
call TX_MakeUpgradeFile.bat
cd ..\..\
copy APP_5.8G_A5130\bat\readme.txt Release\
goto exit

:error
cd ..\..\

:exit
echo.
echo.
echo 结束! & pause > nul


