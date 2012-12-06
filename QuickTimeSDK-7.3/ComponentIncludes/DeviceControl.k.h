/*
     File:       DeviceControl.k.h
 
     Contains:   Component API for doing AVC transactions.
 
     Version:    QuickTime 7.3
 
     Copyright:  © 2007 © 1999-2001 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://developer.apple.com/bugreporter/
 
*/
#ifndef __DEVICECONTROL_K__
#define __DEVICECONTROL_K__

#include <DeviceControl.h>

/*
	Example usage:

		#define DEVICECONTROL_BASENAME()	Fred
		#define DEVICECONTROL_GLOBALS()	FredGlobalsHandle
		#include <DeviceControl.k.h>

	To specify that your component implementation does not use globals, do not #define DEVICECONTROL_GLOBALS
*/
#ifdef DEVICECONTROL_BASENAME
	#ifndef DEVICECONTROL_GLOBALS
		#define DEVICECONTROL_GLOBALS() 
		#define ADD_DEVICECONTROL_COMMA 
	#else
		#define ADD_DEVICECONTROL_COMMA ,
	#endif
	#define DEVICECONTROL_GLUE(a,b) a##b
	#define DEVICECONTROL_STRCAT(a,b) DEVICECONTROL_GLUE(a,b)
	#define ADD_DEVICECONTROL_BASENAME(name) DEVICECONTROL_STRCAT(DEVICECONTROL_BASENAME(),name)

	EXTERN_API( ComponentResult  ) ADD_DEVICECONTROL_BASENAME(DoAVCTransaction) (DEVICECONTROL_GLOBALS() ADD_DEVICECONTROL_COMMA DVCTransactionParams * params);


	/* MixedMode ProcInfo constants for component calls */
	enum {
		uppDeviceControlDoAVCTransactionProcInfo = 0x000003F0
	};

#endif	/* DEVICECONTROL_BASENAME */


#endif /* __DEVICECONTROL_K__ */

