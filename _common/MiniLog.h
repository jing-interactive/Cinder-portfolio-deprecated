#ifndef _MINI_LOG_H_
#define _MINI_LOG_H_

#include <stdio.h>

static FILE* the_fp = NULL;

static struct MiniLogHelper
{
	~MiniLogHelper()
	{
		if (the_fp)
			fclose(the_fp);
	}
}the_helper;

inline void MiniLog(char *string, ...)
{
	//
	if (!the_fp)
	{
		time_t rawtime;
		char fileName[256];
		time ( &rawtime );
		tm* timeinfo = localtime ( &rawtime );
#ifdef WIN32
        system("mkdir log");
		strftime(fileName,80,"log/%Y_%m_%d__%H_%M.log",timeinfo);
#else
		strftime(fileName,80,"%Y_%m_%d__%H_%M.log",timeinfo);
#endif
		printf("%s created.\n", fileName);
		the_fp = fopen(fileName,"w");
		assert(the_fp && "can not open the debug file");
	}
	static char buffer[256];

	va_list arglist;

	if (!string || !the_fp)
		return;

	va_start(arglist,string);
	vsprintf(buffer,string,arglist);
	va_end(arglist);

	fprintf(the_fp,buffer);

	fflush(the_fp);
}

#endif //_MINI_LOG_H_