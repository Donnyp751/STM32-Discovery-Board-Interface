if not exist ..\Bin mkdir ..\Bin


SET MSBUILD="C:\Program Files (x86)\Microsoft Visual Studio\2017\Enterprise\MSBuild\15.0\Bin\msbuild.exe"

SET PROJECTFILE="..\Control Panel\Control Panel.sln"

SET OUTDIR="-property:OutDir=..\..\Bin"

SET PROPS="/p:Configuration=Release"

%MSBUILD% %PROPS% %OUTDIR% %PROJECTFILE%

