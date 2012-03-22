#ifndef DEBUG_H
#define DEBUG_H

#include "../EDSDK.h"
#include "../EDSDKErrors.h"
#include "../EDSDKTypes.h"

class Debug
{
public:
	static void WriteLine(char * format, char* sdkFunctionName, EdsError result);
	static void WriteLine(char * format, EdsUInt32 data);
	static void WriteLine(char * format);
	static void WriteLine(EdsError err);
	static void showResult(char* sdkFunctionName, EdsError result);
	static void exitProgram(char* message);
};

#endif