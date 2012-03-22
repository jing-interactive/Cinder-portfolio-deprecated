/******************************************************************************
*                                                                             *
*   PROJECT : EOS Digital Software Development Kit EDSDK                      *
*      NAME : Thread.h	                                                      *
*                                                                             *
*   Description: This is the Sample code to show the usage of EDSDK.          *
*                                                                             *
*                                                                             *
*******************************************************************************
*                                                                             *
*   Written and developed by Camera Design Dept.53                            *
*   Copyright Canon Inc. 2006 All Rights Reserved                             *
*                                                                             *
*******************************************************************************
*   File Update Information:                                                  *
*     DATE      Identify    Comment                                           *
*   -----------------------------------------------------------------------   *
*   06-03-22    F-001        create first version.                            *
*                                                                             *
******************************************************************************/


#pragma once

#include <windows.h>
#include <process.h>
#include "Synchronized.h"

class Thread  
{
private:
	HANDLE		_hThread;
	bool		_active;

public:
	Thread() : _hThread(),_active(false){} 

	virtual ~Thread(){}


	bool start() 
	{
		_hThread = (HANDLE)_beginthread(threadProc, 0, this);
		return (_hThread != NULL);
	}

	void join()
	{
		if(_hThread)
		{
			::WaitForSingleObject( _hThread, INFINITE );
			_hThread = NULL;
		}
	}

	void sleep(int millisec) const
	{
		if(_hThread)
		{
			Sleep(millisec);
		}
	}

public:
	virtual void runProcess() = 0;

protected:

	static void threadProc(void* lParam)
	{
 		Thread* thread = (Thread *)lParam;
 		if (thread != NULL)
 		{
 		    thread->_active = true;
 			thread->runProcess();
 			thread->_active = false;	
 		}
 		_endthread();
	}

};
