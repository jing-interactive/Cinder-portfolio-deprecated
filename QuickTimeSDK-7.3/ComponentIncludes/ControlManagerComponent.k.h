/*
     File:       ControlManagerComponent.k.h
 
     Contains:   QuickTime Interfaces.
 
     Version:    Technology: QuickTime 6.0
                 Release:    QuickTime 7.3
 
     Copyright:  © 2007 © 1990-2002 by Apple Computer, Inc., all rights reserved
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://developer.apple.com/bugreporter/
 
*/
#ifndef __CONTROLMANAGERCOMPONENT_K__
#define __CONTROLMANAGERCOMPONENT_K__

#include <ControlManagerComponent.h>

/*
	Example usage:

		#define CTRL_BASENAME()	Fred
		#define CTRL_GLOBALS()	FredGlobalsHandle
		#include <ControlManagerComponent.k.h>

	To specify that your component implementation does not use globals, do not #define CTRL_GLOBALS
*/
#ifdef CTRL_BASENAME
	#ifndef CTRL_GLOBALS
		#define CTRL_GLOBALS() 
		#define ADD_CTRL_COMMA 
	#else
		#define ADD_CTRL_COMMA ,
	#endif
	#define CTRL_GLUE(a,b) a##b
	#define CTRL_STRCAT(a,b) CTRL_GLUE(a,b)
	#define ADD_CTRL_BASENAME(name) CTRL_STRCAT(CTRL_BASENAME(),name)

#if CALL_NOT_IN_CARBON
	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(GetCookie) (CTRL_GLOBALS() ADD_CTRL_COMMA void * cookie);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetCookie) (CTRL_GLOBALS() ADD_CTRL_COMMA long  cookie);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(GetCapabilities) (CTRL_GLOBALS() ADD_CTRL_COMMA long * capabilities);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(Create) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetControlTitle) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(DisposeControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(HideControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(ShowControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(Draw1Control) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(HiliteControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(MoveControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SizeControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetControlValue) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetControlMinimum) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetControlMaximum) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(HaveFocus) (CTRL_GLOBALS() ADD_CTRL_COMMA short * haveFocus);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetPreFilterProc) (CTRL_GLOBALS() ADD_CTRL_COMMA long  preFilterProc);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(GetPreFilterProc) (CTRL_GLOBALS() ADD_CTRL_COMMA void * preFilterProc);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetText) (CTRL_GLOBALS() ADD_CTRL_COMMA StringPtr  str);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(GetText) (CTRL_GLOBALS() ADD_CTRL_COMMA StringPtr  str);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SelectText) (CTRL_GLOBALS() ADD_CTRL_COMMA short  startSel, short  endSel);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetDefaultItem) (CTRL_GLOBALS());

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetDrawProc) (CTRL_GLOBALS() ADD_CTRL_COMMA short  theItem, ProcPtr  drawProc);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(TrackControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl, Point  localPt, ControlActionUPP  actionProc);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetFocus) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(TestControl) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl, Point  localPt);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(SetControlData) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl, ControlPartCode  part, ResType  tagName, Size  size, Ptr  data);

	EXTERN_API( ComponentResult  ) ADD_CTRL_BASENAME(GetControlData) (CTRL_GLOBALS() ADD_CTRL_COMMA ControlHandle  hControl, ControlPartCode  part, ResType  tagName, Size  bufferSize, Ptr  buffer, Size * actualSize);


	/* MixedMode ProcInfo constants for component calls */
	enum {
		uppCTRLGetCookieProcInfo = 0x000003F0,
		uppCTRLSetCookieProcInfo = 0x000003F0,
		uppCTRLGetCapabilitiesProcInfo = 0x000003F0,
		uppCTRLCreateProcInfo = 0x000003F0,
		uppCTRLSetControlTitleProcInfo = 0x000003F0,
		uppCTRLDisposeControlProcInfo = 0x000003F0,
		uppCTRLHideControlProcInfo = 0x000003F0,
		uppCTRLShowControlProcInfo = 0x000003F0,
		uppCTRLDraw1ControlProcInfo = 0x000003F0,
		uppCTRLHiliteControlProcInfo = 0x000003F0,
		uppCTRLMoveControlProcInfo = 0x000003F0,
		uppCTRLSizeControlProcInfo = 0x000003F0,
		uppCTRLSetControlValueProcInfo = 0x000003F0,
		uppCTRLSetControlMinimumProcInfo = 0x000003F0,
		uppCTRLSetControlMaximumProcInfo = 0x000003F0,
		uppCTRLHaveFocusProcInfo = 0x000003F0,
		uppCTRLSetPreFilterProcProcInfo = 0x000003F0,
		uppCTRLGetPreFilterProcProcInfo = 0x000003F0,
		uppCTRLSetTextProcInfo = 0x000003F0,
		uppCTRLGetTextProcInfo = 0x000003F0,
		uppCTRLSelectTextProcInfo = 0x00000AF0,
		uppCTRLSetDefaultItemProcInfo = 0x000000F0,
		uppCTRLSetDrawProcProcInfo = 0x00000EF0,
		uppCTRLTrackControlProcInfo = 0x00003FF0,
		uppCTRLSetFocusProcInfo = 0x000003F0,
		uppCTRLTestControlProcInfo = 0x00000FF0,
		uppCTRLSetControlDataProcInfo = 0x0003FBF0,
		uppCTRLGetControlDataProcInfo = 0x000FFBF0
	};

#endif	/* CTRL_BASENAME */

#endif  /* CALL_NOT_IN_CARBON */


#endif /* __CONTROLMANAGERCOMPONENT_K__ */

