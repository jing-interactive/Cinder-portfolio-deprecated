#include "ofxThread.h" 

const DWORD MS_VC_EXCEPTION=0x406D1388;

#pragma pack(push,8)
typedef struct tagTHREADNAME_INFO
{
	DWORD dwType; // Must be 0x1000.
	LPCSTR szName; // Pointer to name (in user addr space).
	DWORD dwThreadID; // Thread ID (-1=caller thread).
	DWORD dwFlags; // Reserved for future use, must be zero.
} THREADNAME_INFO;
#pragma pack(pop)

void SetThreadName( DWORD dwThreadID, char* threadName)
{
	THREADNAME_INFO info;
	info.dwType = 0x1000;
	info.szName = threadName;
	info.dwThreadID = dwThreadID;
	info.dwFlags = 0;

	__try
	{
		RaiseException( MS_VC_EXCEPTION, 0, sizeof(info)/sizeof(ULONG_PTR), (ULONG_PTR*)&info );
	}
	__except(EXCEPTION_EXECUTE_HANDLER)
	{
	}
}

//------------------------------------------------- 
ofxThread::ofxThread(const std::string& name):name_(name){ 
   threadRunning = false; 
   #ifdef WIN32 
      InitializeCriticalSection(&critSec); 
   #else 
      pthread_mutex_init(&myMutex, NULL); 
   #endif 
} 

//------------------------------------------------- 
ofxThread::~ofxThread(){ 
   #ifndef WIN32 
      pthread_mutex_destroy(&myMutex); 
   #endif 
   stopThread(); 
} 

//------------------------------------------------- 
bool ofxThread::isThreadRunning(){ 
   //should be thread safe - no writing of vars here 
   return threadRunning; 
} 

//------------------------------------------------- 
void ofxThread::startThread(bool _blocking, bool _verbose){ 
   if( threadRunning ){ 
      if(verbose)printf("ofxThread: thread already running\n"); 
      return; 
   } 

   //have to put this here because the thread can be running 
   //before the call to create it returns 
   threadRunning   = true; 

   #ifdef WIN32 
      //InitializeCriticalSection(&critSec); 
      myThread = (HANDLE)_beginthreadex(NULL, 0, this->thread,  (void *)this, 0, NULL);
	  DWORD thread_id = GetThreadId(myThread);
	  SetThreadName(thread_id, (char*)name_.c_str());

   #else 
      //pthread_mutex_init(&myMutex, NULL); 
      pthread_create(&myThread, NULL, thread, (void *)this); 
   #endif 

   blocking      =   _blocking; 
   verbose         = _verbose; 
} 

//------------------------------------------------- 
//returns false if it can't lock 
bool ofxThread::lock(){ 

   #ifdef WIN32 
      if(blocking)EnterCriticalSection(&critSec); 
      else { 
         if(!TryEnterCriticalSection(&critSec)){ 
            if(verbose)printf("ofxThread: mutext is busy \n"); 
            return false; 
         } 
      } 
      if(verbose)printf("ofxThread: we are in -- mutext is now locked \n"); 
   #else 

      if(blocking){ 
         if(verbose)printf("ofxThread: waiting till mutext is unlocked\n"); 
         pthread_mutex_lock(&myMutex); 
         if(verbose)printf("ofxThread: we are in -- mutext is now locked \n"); 
      }else{ 
         int value = pthread_mutex_trylock(&myMutex); 
         if(value == 0){ 
            if(verbose)printf("ofxThread: we are in -- mutext is now locked \n"); 
         } 
         else{ 
            if(verbose)printf("ofxThread: mutext is busy - already locked\n"); 
            return false; 
         } 
      } 
   #endif 

   return true; 
} 

//------------------------------------------------- 
bool ofxThread::unlock(){ 

   #ifdef WIN32 
      LeaveCriticalSection(&critSec); 
   #else 
      pthread_mutex_unlock(&myMutex); 
   #endif 

   if(verbose)printf("ofxThread: we are out -- mutext is now unlocked \n"); 

   return true; 
} 

//------------------------------------------------- 
void ofxThread::stopThread(){ 
   if(threadRunning){ 
      if(verbose)printf("ofxThread: thread stopped\n"); 
      threadRunning = false; 
#ifdef WIN32 
	  //::WaitForSingleObject( myThread, INFINITE );
	  ::CloseHandle( myThread );
#else 
	  //pthread_mutex_destroy(&myMutex); 
	  pthread_detach(myThread); 
#endif 
   }else{ 
      if(verbose)printf("ofxThread: thread already stopped\n"); 
   } 
}
