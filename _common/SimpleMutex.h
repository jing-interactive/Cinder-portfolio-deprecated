#pragma once

#include <windows.h>

class CSimpleMutex: public CRITICAL_SECTION
{
public:
    CSimpleMutex ()
            : m_nLockCount(0)
    {
        ::InitializeCriticalSection(this);
    }

    ~CSimpleMutex ()
    {
        ::DeleteCriticalSection(this);
    }

    void lock ()
    {
        ::EnterCriticalSection(this);
        m_nLockCount++;
    }
    void unlock ()
    {
        ::LeaveCriticalSection(this);
        m_nLockCount--;
    }


public :
    int m_nLockCount;
};

struct ScopedLocker
{
	CSimpleMutex& _mutex;
	ScopedLocker(CSimpleMutex& mutex):_mutex(mutex)
	{
		_mutex.lock();
	}
	~ScopedLocker()
	{
		_mutex.unlock();
	}
};

/*usage
CSimpleMutex a_mutex;

{
ScopedLocker(a_mutex);
//do something
}
*/
#define LOCK_START(mutex) {ScopedLocker _locker(mutex)
#define LOCK_END() }

class CSimpleEvent
{
public:
	CSimpleEvent ()
	{
		// start in non-signaled state (red light)
		// auto reset after every Wait
		_handle = CreateEvent (0, FALSE, FALSE, 0);
	}

	~CSimpleEvent ()
	{
		CloseHandle (_handle);
	}

	// put into signaled state
	void set () { SetEvent (_handle); }
	void wait ()
	{
		// Wait until event is in signaled (green) state
		WaitForSingleObject (_handle, INFINITE);
	}
	operator HANDLE () { return _handle; }
private:
	HANDLE _handle;
};
