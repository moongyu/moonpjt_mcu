REM /*********************************************/
REM program 시작시 internet 강제 off 후 Local 환경에서 동작
REM program 종료 시 internet 자동 활성화
REM 1초 단위로 network 상태 체크하여 interent 환경에서는 동작 중지
REM 실행 전 pname, dir, ipadress setting 필요 
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
netsh interface set interface "로컬 영역 연결" disable
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
REM Program 종료시 자동 인터넷 연결 후 batch file 종료 
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
netsh interface set interface "로컬 영역 연결" enable
REM netsh wlan connect "연결 이름"
timeout /t 3
REM pause
EXIT
)