SET DIRECTORY_NAME="C:\Users\songl\Documents\Senseback_implant\pca10040\blank "
TAKEOWN /f %DIRECTORY_NAME% /r /d y
ICACLS %DIRECTORY_NAME% /grant administrators:F /t
PAUSE
