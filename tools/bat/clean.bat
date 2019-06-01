@echo OFF

if not exist Release\ ( mkdir Release\ )

DEL /q Release\*
cd 1\
call clean.bat
cd ..

::ECHO clean ok! & PAUSE > nul