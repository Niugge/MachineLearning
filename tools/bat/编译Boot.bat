@echo OFF

set COMPILER=C:\Keil\UV4\UV4.exe

::��������
if not exist %COMPILER%  (echo �Ҳ���������,���Ȱ�װKeil v4.74  &  goto exit)

::��չ���
echo ��չ���...
call clean.bat
echo.

::����Bootloader
cd BOOT_5.8G_A5130\mdk\
echo ����Bootloader...
%COMPILER% -r "5.8G M0 bootloader.uvproj" -t"RF_TX" -o "../conf_tx/bin/log.txt"
IF ERRORLEVEL 2 ( ECHO Error%ERRORLEVEL%,����Boot_TXʧ�ܣ� & GOTO error )
%COMPILER% -r "5.8G M0 bootloader.uvproj" -t"RF_RX" -o "../conf_rx/bin/log.txt"
IF ERRORLEVEL 2 ( ECHO Error%ERRORLEVEL%,����Boot_RXʧ�ܣ� & GOTO error )
cd ..\..\

::�������ɵ��ļ�
if not exist BOOT_5.8G_A5130\conf_tx\bin\boot.bin ( echo �Ҳ������ɵ��ļ� & goto exit)
if not exist BOOT_5.8G_A5130\conf_rx\bin\boot.bin ( echo �Ҳ������ɵ��ļ� & goto exit)
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
echo ����! & pause > nul


