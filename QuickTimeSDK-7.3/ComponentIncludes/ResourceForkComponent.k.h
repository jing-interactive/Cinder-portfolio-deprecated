/*
     File:       ResourceForkComponent.k.h
 
     Version:    QuickTime 7.3
 
     Copyright:  © 1984-2007 by Apple Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://developer.apple.com/bugreporter/
 
*/
#ifndef __RESOURCEFORKCOMPONENT_K__
#define __RESOURCEFORKCOMPONENT_K__

#include <ResourceForkComponent.h>

/*
	Example usage:

		#define RESOURCEFORK_BASENAME()	Fred
		#define RESOURCEFORK_GLOBALS()	FredGlobalsHandle
		#include <ResourceForkComponent.k.h>

	To specify that your component implementation does not use globals, do not #define RESOURCEFORK_GLOBALS
*/
#ifdef RESOURCEFORK_BASENAME
	#ifndef RESOURCEFORK_GLOBALS
		#define RESOURCEFORK_GLOBALS() 
		#define ADD_RESOURCEFORK_COMMA 
	#else
		#define ADD_RESOURCEFORK_COMMA ,
	#endif
	#define RESOURCEFORK_GLUE(a,b) a##b
	#define RESOURCEFORK_STRCAT(a,b) RESOURCEFORK_GLUE(a,b)
	#define ADD_RESOURCEFORK_BASENAME(name) RESOURCEFORK_STRCAT(RESOURCEFORK_BASENAME(),name)

#if CALL_NOT_IN_CARBON
	EXTERN_API( ComponentResult  ) ADD_RESOURCEFORK_BASENAME(PathNameMap) (RESOURCEFORK_GLOBALS() ADD_RESOURCEFORK_COMMA char * inDataPath, char * outRsrcPath, unsigned long  maxLen);

	EXTERN_API( ComponentResult  ) ADD_RESOURCEFORK_BASENAME(OffsetAndLength) (RESOURCEFORK_GLOBALS() ADD_RESOURCEFORK_COMMA char * accessPath, long  queryFlags, long * dataOffset, long * dataLength);

	EXTERN_API( ComponentResult  ) ADD_RESOURCEFORK_BASENAME(GetMethodInfo) (RESOURCEFORK_GLOBALS() ADD_RESOURCEFORK_COMMA long  queryFlags, long  queryType, long * answer);

	EXTERN_API( ComponentResult  ) ADD_RESOURCEFORK_BASENAME(FileLength) (RESOURCEFORK_GLOBALS() ADD_RESOURCEFORK_COMMA char * accessPath, long  queryFlags, long * fileLength);

	EXTERN_API( ComponentResult  ) ADD_RESOURCEFORK_BASENAME(OffsetAndLength64) (RESOURCEFORK_GLOBALS() ADD_RESOURCEFORK_COMMA char * accessPath, long  queryFlags, wide * dataOffset, wide * dataLength);

	EXTERN_API( ComponentResult  ) ADD_RESOURCEFORK_BASENAME(FileLength64) (RESOURCEFORK_GLOBALS() ADD_RESOURCEFORK_COMMA char * accessPath, long  queryFlags, wide * fileLength);


	/* MixedMode ProcInfo constants for component calls */
	enum {
		uppResourceForkPathNameMapProcInfo = 0x00003FF0,
		uppResourceForkOffsetAndLengthProcInfo = 0x0000FFF0,
		uppResourceForkGetMethodInfoProcInfo = 0x00003FF0,
		uppResourceForkFileLengthProcInfo = 0x00003FF0,
		uppResourceForkOffsetAndLength64ProcInfo = 0x0000FFF0,
		uppResourceForkFileLength64ProcInfo = 0x00003FF0
	};

#endif	/* RESOURCEFORK_BASENAME */

#endif  /* CALL_NOT_IN_CARBON */


#endif /* __RESOURCEFORKCOMPONENT_K__ */

