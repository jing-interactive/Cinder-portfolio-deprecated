/************************************************************************
*                                                                       *
*   MSR_NuiProps.h -- This module defines the APIs for the Natural      *
*                    User Interface(NUI) properties enumerator.         *
*                                                                       *
*   Copyright (c) Microsoft Corp. All rights reserved.                  *
*                                                                       *
************************************************************************/
#pragma once

#ifndef NUIAPI
#error "You must include nuiapi.h rather than including nuiprops.h directly"
#endif

namespace MsrNui
{

typedef enum _NUI_PROPSINDEX
{
    INDEX_UNIQUE_DEVICE_NAME = 0,
    INDEX_LAST // don't use!
} NUI_PROPSINDEX;

typedef enum _NUI_PROPSTYPE
{
    PROPTYPE_UNKNOWN = 0,   // don't use
    PROPTYPE_UINT,      // no need to return anything smaller than an int
    PROPTYPE_FLOAT,
    PROPTYPE_BSTR,      // returns new BSTR. Use SysFreeString( BSTR ) when you're done
    PROPTYPE_BLOB
} NUI_PROPSTYPE;

typedef struct _NUI_PROPTYPEANDSIZE
{
    NUI_PROPSINDEX index;
    NUI_PROPSTYPE  type;
    size_t         size;
} NUI_PROPTYPEANDSIZE;

static const NUI_PROPTYPEANDSIZE g_spNuiPropsType [] =
{
    { INDEX_UNIQUE_DEVICE_NAME,   PROPTYPE_BSTR, sizeof( HANDLE ) },
};

}; // namespace MsrNui
