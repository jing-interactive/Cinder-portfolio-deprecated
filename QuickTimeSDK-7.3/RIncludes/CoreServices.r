/*
     File:       CoreServices.r
 
     Contains:   Master include for CoreServices (non-UI toolbox)
 
     Version:    QuickTime 7.3
 
     Copyright:  © 2007 © 1999-2001 by Apple Computer, Inc., all rights reserved.
 
     Bugs?:      For bug reports, consult the following page on
                 the World Wide Web:
 
                     http://developer.apple.com/bugreporter/
 
*/

#ifndef __CORESERVICES_R__
#define __CORESERVICES_R__

#ifndef TARGET_API_MAC_CARBON
#define TARGET_API_MAC_CARBON 1
#endif  /* !defined(TARGET_API_MAC_CARBON) */

#ifndef __CONDITIONALMACROS_R__
#include "ConditionalMacros.r"
#endif

#ifndef __MACTYPES_R__
#include "MacTypes.r"
#endif

#ifndef __FINDER_R__
#include "Finder.r"
#endif

#ifndef __SCRIPT_R__
#include "Script.r"
#endif

#ifndef __TEXTCOMMON_R__
#include "TextCommon.r"
#endif

#ifndef __COLLECTIONS_R__
#include "Collections.r"
#endif

#ifndef __GESTALT_R__
#include "Gestalt.r"
#endif

#ifndef __MIXEDMODE_R__
#include "MixedMode.r"
#endif

#ifndef __COMPONENTS_R__
#include "Components.r"
#endif

#ifndef __OSUTILS_R__
#include "OSUtils.r"
#endif

#ifndef __CODEFRAGMENTS_R__
#include "CodeFragments.r"
#endif

#ifndef __DEVICES_R__
#include "Devices.r"
#endif

#ifndef __INTLRESOURCES_R__
#include "IntlResources.r"
#endif

#ifndef __UNICODEUTILITIES_R__
#include "UnicodeUtilities.r"
#endif

#ifndef __UNICODECONVERTER_R__
#include "UnicodeConverter.r"
#endif

#ifndef __FOLDERS_R__
#include "Folders.r"
#endif

#ifndef __ICONSTORAGE_R__
#include "IconStorage.r"
#endif


#endif /* __CORESERVICES_R__ */

