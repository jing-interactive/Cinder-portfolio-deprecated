#include "Debug.h"
#include "stdio.h"

 void Debug::WriteLine(char * format, char* sdkFunctionName, EdsError result)
{
	printf(sdkFunctionName);
	printf("  ");
	printf("%d\n",result);

}
 void Debug::WriteLine(char * format, EdsUInt32 data)
{
	printf(format);
	printf("  ");
	printf("%s\n",data);
}
 void Debug::WriteLine(char * format)
{
#ifdef DEBUG
	printf(format);
	printf("\n");
#endif
}
 void Debug::WriteLine(EdsError err)
{
	printf("ERROR:%d\n",err);
}

 void Debug::showResult(char* sdkFunctionName, EdsError result)
 {
#ifdef _DEBUG
	 WriteLine("%s: %u\n", sdkFunctionName, result);
#endif
	 //if the result is an error, exit the program
	 if(result != EDS_ERR_OK)
		 exitProgram("Terminating");
 }

 void Debug::exitProgram(char* message)
 {
	 WriteLine(message);

	 //pause the terminal before the program exits
	 system("PAUSE");

	 exit(1);
 }