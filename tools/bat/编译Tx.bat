@echo OFF

set COMPILER=C:\Keil\UV4\UV4.exe

::��������
if not exist %COMPILER%  (echo �Ҳ���������,���Ȱ�װKeil v4.74  &  goto exit)

::��չ���
echo ��չ���...
call clean.bat
echo.

::���Bootloader
echo ����Ƿ����Boot...
if not exist boot\boot_tx.bin ( echo �������ȱ���Boot����Ȼ�󿽱�boot_tx.bin��Ŀ¼boot�� & goto exit )

::����APP
cd APP_5.8G_A5130\mdk\
echo ����App...
%COMPILER% -r "2.4G_AT86RF233.uvproj" -t"RF_TX" -o "../conf_tx/bin/log.txt"
IF ERRORLEVEL 2 ( ECHO Error%ERRORLEVEL%,����App_TXʧ�ܣ� & GOTO error )
cd ..\..\

::���������ļ�
cd APP_5.8G_A5130\bat\
echo ����������¼�ļ�...
call TX_MakeImageFile.bat
echo ���������ļ�...
call TX_MakeUpgradeFile.bat
cd ..\..\
copy APP_5.8G_A5130\bat\readme.txt Release\
goto exit

:error
cd ..\..\

:exit
echo.
echo.
echo ����! & pause > nul


