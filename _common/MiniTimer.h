#ifndef _MINI_TIMER_H_
#define _MINI_TIMER_H_

#ifdef WIN32
#include <windows.h>
#pragma comment(lib,"winmm.lib")
#else
#include <sys/time.h>
#endif
 
//uncomment this if u have "MiniLog.h"
//#define USING_FLO_WRITE

#ifdef USING_FLO_WRITE
#include "MiniLog.h"
#else
#include <stdio.h>
#define FloWrite(str) 
#endif

class MiniTimer
{
public:
	MiniTimer()
	{
		resetStartTime();
	}
    
    static unsigned int getGlobalTime()
    {
#ifdef WIN32
        return timeGetTime();
#else
        timeval tv;
        gettimeofday(&tv, 0 );
        return tv.tv_usec;
#endif
    }
    
	unsigned int getTimeElapsedMS()//mil-seconds
	{
		return getGlobalTime() - _start_time;
	}
    
	void resetStartTime()
	{
		_start_time = getGlobalTime();
	}
    
	float getTimeElapsed()//seconds
	{
		return 0.001f*(getGlobalTime() - _start_time);
	}

	void profileFunction(char* funcName)
	{
#ifdef USING_FLO_WRITE
		MiniLog("%s[%s] : %d ms\n", getProperBlank(), funcName, getTimeElapsedMS());
#else
		printf("%s[%s] : %d ms\n", getProperBlank(), funcName, getTimeElapsedMS());	
#endif
		resetStartTime();
	}

	static char* getProperBlank()
	{
		return "--";
	}

private:
	unsigned int _start_time;
};

#endif
