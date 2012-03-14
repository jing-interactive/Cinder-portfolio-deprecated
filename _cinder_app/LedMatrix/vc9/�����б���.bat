call "D:/Program Files/Microsoft Visual Studio 9.0/VC/vcvarsall.bat"
msbuild  /p:configuration=release
call "Release/LedMatrix.exe"