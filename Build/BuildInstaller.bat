if not exist ..\Installer mkdir ..\Installer

SET CPBUILD="BuildControlPanel.bat"
SET INNO="C:\Program Files (x86)\Inno Setup 5\ISCC.exe"
SET INSTALLERSCRIPT="Installer Script.iss"
SET ARGS="/FSTM32 Demo Installer" "/O..\Installer"

;Build the control panel, then the installer 

call %CPBUILD%

%INNO% %ARGS% %INSTALLERSCRIPT%

PAUSE