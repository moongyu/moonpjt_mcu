REM /*********************************************/
REM program ���۽� internet ���� off �� Local ȯ�濡�� ����
REM program ���� �� internet �ڵ� Ȱ��ȭ
REM 1�� ������ network ���� üũ�Ͽ� interent ȯ�濡���� ���� ����
REM ���� �� pname, dir, ipadress setting �ʿ� 
REM /*********************************************/

@echo off
echo [Starting Auto Program Execution...]

DATE /T 
TIME /T

REM var=xxx
set pname=windirstat.exe
set dir=C:\Program Files (x86)\windirstat
set ipadress=192.168.219.104

REM /*********************************************/
echo Disconnect internet
netsh interface set interface "���� ���� ����" disable
REM netsh wlan disconnect

timeout /t 1


REM /*********************************************/
echo [Check status of internet connection]
ping %ipadress% -n 1
REM /*********************************************/

if errorlevel 1 (
echo [Internet NOT conneted]
echo [Executing program]
GOTO Start
GOTO Monitor
)
if errorlevel 0 (
echo [Internet conneted]
echo [Check internet connection]
pause
)


REM /*********************************************/
REM Lable
:Start
REM cd C:\Program Files (x86)\windirstat
REM windirstat.exe
start /d "%dir%" %pname%

:Monitor
REM Program ����� �ڵ� ���ͳ� ���� �� batch file ���� 
timeout /t 1

ping %ipadress% -n 1
if %ERRORLEVEL% == 0 (
taskkill /f /im %pname%
echo [Disconnect Internet before Executing program]
pause
GOTO:EOF
)

tasklist /fi "imagename eq %pname%" | findstr %pname%
if %ERRORLEVEL% == 0 (
REM echo Executing program...
GOTO Monitor
) else (
echo [program terminated...]
taskkill /f /im %pname%
netsh interface set interface "���� ���� ����" enable
REM netsh wlan connect "���� �̸�"
timeout /t 3
REM pause
EXIT
)