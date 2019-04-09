[Files]
Source: "..\bin\Control Panel.exe"; DestDir: "{app}\bin"; DestName: "Control Panel.exe"; Flags: ignoreversion
Source: "..\bin\Newtonsoft.Json.dll"; DestDir: "{app}\bin"; DestName: "Newtonsoft.Json.dll"; Flags: ignoreversion
Source: "..\bin\OxyPlot.dll"; DestDir: "{app}\bin"; DestName: "OxyPlot.dll"; Flags: ignoreversion
Source: "..\bin\OxyPlot.Wpf.dll"; DestDir: "{app}\bin"; DestName: "OxyPlot.Wpf.dll"; Flags: ignoreversion

[Setup]
AppName=STM32 Demo Control Panel
AppVersion=1.0.1
AppId={{A81F910E-2610-4255-96AC-0A9E19296545}
RestartIfNeededByRun=False
ExtraDiskSpaceRequired=7
AppPublisher=Donald Posterick Dunwoody College of Technology
AppPublisherURL=https://dmposterick.com/
UninstallDisplayName=STM32 Demo
VersionInfoVersion=1.0.1
VersionInfoCompany=Dunwoody College of Technology
SolidCompression=True
Compression=lzma2/ultra64
DefaultDirName="{pf32}\STM32 Control Panel"
[Files]

[Icons]
Name: "{userdesktop}\STM32 Demo"; Filename: "{app}\bin\Control Panel.exe"; WorkingDir: "{app}"
