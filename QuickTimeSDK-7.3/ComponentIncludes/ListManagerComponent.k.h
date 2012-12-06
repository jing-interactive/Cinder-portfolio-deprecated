/*
     File:       ListManagerComponent.k.h
 
     Contains:   QuickTime Interfaces.
 
     Version:    Technology: QuickTime 6.0
                 Release:    QuickTime 7.3
 
     Copyright:  © 2007 © 1990-2002 by Apple Computer, Inc., all rights reserved
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://developer.apple.com/bugreporter/
 
*/
#ifndef __LISTMANAGERCOMPONENT_K__
#define __LISTMANAGERCOMPONENT_K__

#include <ListManagerComponent.h>

/*
	Example usage:

		#define LIST_BASENAME()	Fred
		#define LIST_GLOBALS()	FredGlobalsHandle
		#include <ListManagerComponent.k.h>

	To specify that your component implementation does not use globals, do not #define LIST_GLOBALS
*/
#ifdef LIST_BASENAME
	#ifndef LIST_GLOBALS
		#define LIST_GLOBALS() 
		#define ADD_LIST_COMMA 
	#else
		#define ADD_LIST_COMMA ,
	#endif
	#define LIST_GLUE(a,b) a##b
	#define LIST_STRCAT(a,b) LIST_GLUE(a,b)
	#define ADD_LIST_BASENAME(name) LIST_STRCAT(LIST_BASENAME(),name)

#if CALL_NOT_IN_CARBON
	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(Create) (LIST_GLOBALS() ADD_LIST_COMMA ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LDispose) (LIST_GLOBALS() ADD_LIST_COMMA ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LAddRow) (LIST_GLOBALS() ADD_LIST_COMMA short  count, short  rowNum, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LDelRow) (LIST_GLOBALS() ADD_LIST_COMMA short  count, short  rowNum, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LSetDrawingMode) (LIST_GLOBALS() ADD_LIST_COMMA Boolean  drawIt, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LAutoScroll) (LIST_GLOBALS() ADD_LIST_COMMA ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LActivate) (LIST_GLOBALS() ADD_LIST_COMMA Boolean  act, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LAddToCell) (LIST_GLOBALS() ADD_LIST_COMMA const void * dataPtr, short  dataLen, Cell  theCell, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LClrCell) (LIST_GLOBALS() ADD_LIST_COMMA Cell  theCell, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LSetCell) (LIST_GLOBALS() ADD_LIST_COMMA const void * dataPtr, short  dataLen, Cell  theCell, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LSetSelect) (LIST_GLOBALS() ADD_LIST_COMMA Boolean  setIt, Cell  theCell, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LDraw) (LIST_GLOBALS() ADD_LIST_COMMA Cell  theCell, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(GetVisibleBounds) (LIST_GLOBALS() ADD_LIST_COMMA Rect * bounds, ListHandle  lHandle);

	EXTERN_API( ComponentResult  ) ADD_LIST_BASENAME(LScroll) (LIST_GLOBALS() ADD_LIST_COMMA short  dh, short  dv, ListHandle  lHandle);


	/* MixedMode ProcInfo constants for component calls */
	enum {
		uppLISTCreateProcInfo = 0x000003F0,
		uppLISTLDisposeProcInfo = 0x000003F0,
		uppLISTLAddRowProcInfo = 0x00003AF0,
		uppLISTLDelRowProcInfo = 0x00003AF0,
		uppLISTLSetDrawingModeProcInfo = 0x00000DF0,
		uppLISTLAutoScrollProcInfo = 0x000003F0,
		uppLISTLActivateProcInfo = 0x00000DF0,
		uppLISTLAddToCellProcInfo = 0x0000FBF0,
		uppLISTLClrCellProcInfo = 0x00000FF0,
		uppLISTLSetCellProcInfo = 0x0000FBF0,
		uppLISTLSetSelectProcInfo = 0x00003DF0,
		uppLISTLDrawProcInfo = 0x00000FF0,
		uppLISTGetVisibleBoundsProcInfo = 0x00000FF0,
		uppLISTLScrollProcInfo = 0x00003AF0
	};

#endif	/* LIST_BASENAME */

#endif  /* CALL_NOT_IN_CARBON */


#endif /* __LISTMANAGERCOMPONENT_K__ */

