@set vcbuild_path="%VS90COMNTOOLS%\..\..\VC\vcpackages\vcbuild.exe"

@if exist %vcbuild_path% goto do_build 

@echo ERROR: vcbuild.exe not found in
@echo %vcbuild_path%
@echo Please edit this batch file to set correct path to vcbuild.exe
@pause

@goto end

:do_build
@%vcbuild_path% /MP vc9\CiApp.vcproj Release

:end
