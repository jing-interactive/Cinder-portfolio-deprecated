call "c:/Program Files/Microsoft Visual Studio 9.0/VC/vcvarsall.bat"
msbuild  box2d.sln /p:configuration=debug
msbuild  box2d.sln /p:configuration=release
pause