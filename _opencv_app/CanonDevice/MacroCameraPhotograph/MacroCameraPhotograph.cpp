#include "../FC/EDSDK.h"
#include "../FC/EDSDKErrors.h"
#include "../FC/EDSDKTypes.h"
#include "../fc/stdafx.h"

#include <stdio.h>

#include "../fc/CanonControl/Debug.h"
#include "../fc/CanonControl/controller.h"
#include "../fc/CanonControl/DownloadCommand.h"
#include "../fc/Configuration.h"

using namespace std;

TCHAR* picturename = _T("");

bool visual = false;

void help()
{
	printf("==========================================================\n");
	printf("One mode: Take picture.\n");
	printf("==========================================================");
	printf("Take picture:\n");
	printf("MacroCameraPhotograph.exe -n picturename.jpg.\n");
	printf("==========================================================\n");
}

void printArg()
{
	wprintf(L"take picture:%s\n", picturename);
}

TCHAR* getNxtArg(TCHAR* argv[], int argc)
{
	static int idx = 1;
	if(idx < argc)	return argv[idx++];
	else			return 0;
}

int parseTakePictureModeArgs(TCHAR* argv[], int argc)
{
	TCHAR* arg = getNxtArg(argv, argc);
	if(!arg) return -1;

	while(arg){
		if(arg[0] != _T('-')) return -1;
		switch(arg[1]){
			case _T('t')://
			case _T('T'):
				arg = getNxtArg(argv, argc);
				if(!arg) return -1;
				break;
			case _T('n')://
			case _T('N'):
				arg = getNxtArg(argv, argc);
				if(!arg) return -1;
				picturename = arg;
				break;
			case _T('V'):
				visual = true;
				break;
			default:
				return -1;
		}
		arg = getNxtArg(argv, argc);
	}

	//check
	if(picturename)						return  0;
	else					    		return -1;
}

int _tmain(int argc, TCHAR* argv[])
{
	if(0 != parseTakePictureModeArgs(argv, argc)){
		help();
		return -1;
	}

	printArg();

	int len = (int) wcslen(picturename) + 1;
	len *= 2;

	char* buf = new char[len];

	WideCharToMultiByte(CP_ACP, 0, picturename, -1, buf, len, 0, 0);

#if 1
	Camera camera;
#else 
	Camera camera("Canon EOS 500D");
#endif
	camera.setSavePathName(buf);
	camera.takePicture();

	delete []buf; 

	return 0;
}


