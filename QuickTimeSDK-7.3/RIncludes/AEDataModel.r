/*
     File:       AEDataModel.r
 
     Contains:   AppleEvent Data Model Interfaces.
 
     Version:    QuickTime 7.3
 
     Copyright:  © 2007 © 1996-2001 by Apple Computer, Inc., all rights reserved
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://developer.apple.com/bugreporter/
 
*/

#ifndef __AEDATAMODEL_R__
#define __AEDATAMODEL_R__

#ifndef __CONDITIONALMACROS_R__
#include "ConditionalMacros.r"
#endif

/* Apple event descriptor types */
#define typeBoolean 					'bool'
#define typeChar 						'TEXT'

/* Preferred numeric Apple event descriptor types */
#define typeSInt16 						'shor'
#define typeSInt32 						'long'
#define typeUInt32 						'magn'
#define typeSInt64 						'comp'
#define typeIEEE32BitFloatingPoint 		'sing'
#define typeIEEE64BitFloatingPoint 		'doub'
#define type128BitFloatingPoint 		'ldbl'
#define typeDecimalStruct 				'decm'

/* Non-preferred Apple event descriptor types */
#define typeSMInt 						'shor'
#define typeShortInteger 				'shor'
#define typeInteger 					'long'
#define typeLongInteger 				'long'
#define typeMagnitude 					'magn'
#define typeComp 						'comp'
#define typeSMFloat 					'sing'
#define typeShortFloat 					'sing'
#define typeFloat 						'doub'
#define typeLongFloat 					'doub'
#define typeExtended 					'exte'

/* More Apple event descriptor types */
#define typeAEList 						'list'
#define typeAERecord 					'reco'
#define typeAppleEvent 					'aevt'
#define typeEventRecord 				'evrc'
#define typeTrue 						'true'
#define typeFalse 						'fals'
#define typeAlias 						'alis'
#define typeEnumerated 					'enum'
#define typeType 						'type'
#define typeAppParameters 				'appa'
#define typeProperty 					'prop'
#define typeFSS 						'fss '
#define typeFSRef 						'fsrf'
#define typeFileURL 					'furl'
#define typeKeyword 					'keyw'
#define typeSectionH 					'sect'
#define typeWildCard 					'****'
#define typeApplSignature 				'sign'
#define typeQDRectangle 				'qdrt'
#define typeFixed 						'fixd'
#define typeProcessSerialNumber 		'psn '
#define typeApplicationURL 				'aprl'
#define typeNull 						'null'				/*  null or nonexistent data  */

/* Deprecated addressing modes under Carbon */
#if CALL_NOT_IN_CARBON
#define typeSessionID 					'ssid'
#define typeTargetID 					'targ'
#define typeDispatcherID 				'dspt'

#endif  /* CALL_NOT_IN_CARBON */

/* Keywords for Apple event attributes */
#define keyTransactionIDAttr 			'tran'
#define keyReturnIDAttr 				'rtid'
#define keyEventClassAttr 				'evcl'
#define keyEventIDAttr 					'evid'
#define keyAddressAttr 					'addr'
#define keyOptionalKeywordAttr 			'optk'
#define keyTimeoutAttr 					'timo'
#define keyInteractLevelAttr 			'inte'				/*  this attribute is read only - will be set in AESend  */
#define keyEventSourceAttr 				'esrc'				/*  this attribute is read only - returned as typeShortInteger  */
#define keyMissedKeywordAttr 			'miss'				/*  this attribute is read only  */
#define keyOriginalAddressAttr 			'from'				/*  new in 1.0.1  */
#define keyAcceptTimeoutAttr 			'actm'				/*  new for Mac OS X  */

#define kAENormalPriority 				0x00000000			/*  post message at the end of the event queue  */
#define kAEHighPriority 				0x00000001			/*  post message at the front of the event queue (same as nAttnMsg)  */

#define kAENoReply 						0x00000001			/*  sender doesn't want a reply to event  */
#define kAEQueueReply 					0x00000002			/*  sender wants a reply but won't wait  */
#define kAEWaitReply 					0x00000003			/*  sender wants a reply and will wait  */
#define kAEDontReconnect 				0x00000080			/*  don't reconnect if there is a sessClosedErr from PPCToolbox  */
#define kAEWantReceipt 					0x00000200			/*  (nReturnReceipt) sender wants a receipt of message  */
#define kAENeverInteract 				0x00000010			/*  server should not interact with user  */
#define kAECanInteract 					0x00000020			/*  server may try to interact with user  */
#define kAEAlwaysInteract 				0x00000030			/*  server should always interact with user where appropriate  */
#define kAECanSwitchLayer 				0x00000040			/*  interaction may switch layer  */
#define kAEDontRecord 					0x00001000			/*  don't record this event - available only in vers 1.0.1 and greater  */
#define kAEDontExecute 					0x00002000			/*  don't send the event for recording - available only in vers 1.0.1 and greater  */
#define kAEProcessNonReplyEvents 		0x00008000			/*  allow processing of non-reply events while awaiting synchronous AppleEvent reply  */


#endif /* __AEDATAMODEL_R__ */

