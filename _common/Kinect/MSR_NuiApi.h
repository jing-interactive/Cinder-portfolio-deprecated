/************************************************************************
*                                                                       *
*   MSR_NuiApi.h -- This module aggregates all the Natural User         *
*                    Interface(NUI) API headers.                        *
*                                                                       *
*   Copyright (c) Microsoft Corp. All rights reserved.                  *
*                                                                       *
************************************************************************/

#pragma once

#ifndef NUIAPI
    #define NUIAPI extern "C" __declspec( dllimport ) 
#endif

#include <pshpack8.h>

typedef struct _Vector4
{
#pragma warning(push)
#pragma warning(disable: 4201) // allow nameless union
    union
    {
        float        vector4_f32[4];
        unsigned int vector4_u32[4];
        struct
        {
            FLOAT x;
            FLOAT y;
            FLOAT z;
            FLOAT w;
        };
        FLOAT v[4];
        UINT  u[4];
    };
#pragma warning(pop)
} Vector4;

typedef const Vector4 FVector4;

#ifdef __cplusplus
extern "C" {
#endif

//
// NUI Common Initialization Declarations
//

#define NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX 0x00000001
#define NUI_INITIALIZE_FLAG_USES_COLOR                  0x00000002
#define NUI_INITIALIZE_FLAG_USES_SKELETON               0x00000008  
#define NUI_INITIALIZE_FLAG_USES_DEPTH                  0x00000020

#define NUI_INITIALIZE_DEFAULT_HARDWARE_THREAD          0xFFFFFFFF

NUIAPI
HRESULT 
NuiInitialize(
    _In_ DWORD dwFlags
    );


NUIAPI
VOID
NuiShutdown(
    );

#include "MSR_NuiProps.h"

NUIAPI bool                    MSR_NuiGetPropsBlob( MsrNui::NUI_PROPSINDEX Index, void * pBlob, DWORD * pdwInOutSize );
NUIAPI MsrNui::NUI_PROPSTYPE   MSR_NuiGetPropsType( MsrNui::NUI_PROPSINDEX Index );

//
// Define NUI error codes derived from win32 errors
//

#define E_NUI_DEVICE_NOT_CONNECTED  __HRESULT_FROM_WIN32(ERROR_DEVICE_NOT_CONNECTED)
#define E_NUI_DEVICE_NOT_READY      __HRESULT_FROM_WIN32(ERROR_NOT_READY)
#define E_NUI_ALREADY_INITIALIZED   __HRESULT_FROM_WIN32(ERROR_ALREADY_INITIALIZED)
#define E_NUI_NO_MORE_ITEMS         __HRESULT_FROM_WIN32(ERROR_NO_MORE_ITEMS)

//
// Define NUI specific error codes
// **** ALSO DEFINED IN NuiError.h.  Keep in sync! ****
//

#ifndef _NUI_HRESULTS
#define _NUI_HRESULTS
#define FACILITY_NUI 0x301
#define E_NUI_FRAME_NO_DATA                     MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 1)
//static_assert(E_NUI_FRAME_NO_DATA == 0x83010001, "Error code has changed.");
#define E_NUI_STREAM_NOT_ENABLED                MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 2)
#define E_NUI_IMAGE_STREAM_IN_USE               MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 3)
#define E_NUI_FRAME_LIMIT_EXCEEDED              MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 4)
#define E_NUI_FEATURE_NOT_INITIALIZED           MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 5)
#define E_NUI_DATABASE_NOT_FOUND                MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 13)
#define E_NUI_DATABASE_VERSION_MISMATCH         MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 14)
// The hub is no longer connected to the machine
#define E_NUI_NOTCONNECTED                      MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, /* 20 */ ERROR_BAD_UNIT)                         // 0x83010014
// Some part of the device is not connected.
#define E_NUI_NOTREADY                          MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, /* 21 */ ERROR_NOT_READY)                        // 0x83010015
// Skeletal engine is already in use
#define E_NUI_SKELETAL_ENGINE_BUSY              MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, /* 170 */ ERROR_BUSY)
// The hub and motor are connected, but the camera is not
#define E_NUI_NOTPOWERED                        MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, /* 639 */ ERROR_INSUFFICIENT_POWER)               // 0x8301027F
// Bad index passed in to NuiCreateInstanceByXXX
#define E_NUI_BADIINDEX                         MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, /* 1413 */ ERROR_INVALID_INDEX)
#endif

// NOTE: Error codes in the range 100-199 associated with FACILITY_NUI are
// assigned to the NUI Handles API subsystem of NUI. See NuiHandles.h for
// details on those error codes.


#ifdef __cplusplus
} // end extern "C"
#endif

#include "MSR_NuiImageCamera.h"
#include "MSR_NuiSkeleton.h"

#include <poppack.h>

//***********************
// this interface duplicates exactly the public DLL NUI**** methods that
// work on just device #0. If you want to work with multiple devices,
// use these methods off the INuiInstance, after getting a INuiInstance * from
// the multiple-device methods below
//***********************

interface INuiInstance
{
    virtual int         InstanceIndex( ) = 0; // which instance # was it created with, in MSR_NuiCreateInstanceByIndex( )/etc?
    virtual HRESULT     NuiInitialize( _In_ DWORD dwFlags ) = 0;
    virtual VOID        NuiShutdown( ) = 0;
    virtual HRESULT     NuiImageStreamOpen(
        _In_ NUI_IMAGE_TYPE eImageType,
        _In_ NUI_IMAGE_RESOLUTION eResolution,
        _In_ DWORD dwImageFrameFlags_NotUsed,
        _In_ DWORD dwFrameLimit,
        _In_opt_ HANDLE hNextFrameEvent,
        _Out_ HANDLE *phStreamHandle
        ) = 0;

    virtual HRESULT     NuiImageStreamGetNextFrame(
        _In_  HANDLE hStream,
        _In_  DWORD dwMillisecondsToWait,
        _Deref_out_ CONST NUI_IMAGE_FRAME **ppcImageFrame
        ) = 0;

    virtual HRESULT     NuiImageStreamReleaseFrame(
        _In_ HANDLE hStream,
        CONST NUI_IMAGE_FRAME *pImageFrame
        ) = 0;

    virtual HRESULT NuiImageGetColorPixelCoordinatesFromDepthPixel(
        _In_ NUI_IMAGE_RESOLUTION eColorResolution,
        _In_opt_ CONST NUI_IMAGE_VIEW_AREA *pcViewArea,
        _In_ LONG   lDepthX,
        _In_ LONG   lDepthY,
        _In_ USHORT usDepthValue,
        _Out_ LONG *plColorX,
        _Out_ LONG *plColorY
        ) = 0;    

    virtual HRESULT NuiCameraElevationSetAngle(
        _In_ LONG lAngleDegrees
        ) = 0;

    virtual HRESULT NuiCameraElevationGetAngle(
        _In_ LONG * plAngleDegrees
        ) = 0;

    virtual HRESULT NuiSkeletonTrackingEnable(
        _In_opt_ HANDLE hNextFrameEvent,
        _In_     DWORD  dwFlags
        ) = 0;

    virtual HRESULT NuiSkeletonTrackingDisable(
        ) = 0;

    virtual HRESULT NuiSkeletonGetNextFrame(
        _In_  DWORD dwMillisecondsToWait,
        _Out_ NUI_SKELETON_FRAME *pSkeletonFrame
        ) = 0;

    virtual HRESULT NuiTransformSmooth( 
        NUI_SKELETON_FRAME *pSkeletonFrame, 
        const NUI_TRANSFORM_SMOOTH_PARAMETERS *pSmoothingParams 
        ) = 0;

    virtual bool                  MSR_NuiGetPropsBlob( MsrNui::NUI_PROPSINDEX Index, void * pBlob, DWORD * pdwInOutSize ) = 0;
    virtual MsrNui::NUI_PROPSTYPE MSR_NuiGetPropsType( MsrNui::NUI_PROPSINDEX Index ) = 0;
    virtual BSTR NuiInstanceName() = 0;
    virtual HRESULT NuiStatus() = 0;
    virtual DWORD NuiInitializationFlags() = 0;
};

//***********************
// NUI enumeration function
//***********************

NUIAPI HRESULT MSR_NUIGetDeviceCount( int * pCount ); // note capitalization
NUIAPI HRESULT MSR_NuiCreateInstanceByIndex( int Index, INuiInstance ** ppInstance );
NUIAPI void    MSR_NuiDestroyInstance( INuiInstance * pInstance );
NUIAPI HRESULT MSR_NuiCreateInstanceByName( const OLECHAR *strInstanceName, INuiInstance ** ppNuiInstance );

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _NuiStatusData
{
    size_t cb;
    const OLECHAR* instanceName;
    HRESULT hrStatus;
} NuiStatusData;

typedef void (CALLBACK* NuiStatusProc)( const NuiStatusData * );
NUIAPI NuiStatusProc MSR_NuiSetDeviceStatusCallback( NuiStatusProc callback );

inline bool HasSkeletalEngine(INuiInstance *pInstance)
{
    if (!pInstance)
        return false;

    return (pInstance->NuiInitializationFlags() & NUI_INITIALIZE_FLAG_USES_SKELETON) || (pInstance->NuiInitializationFlags() & NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX);
};

#ifdef __cplusplus
} // end extern "C"
#endif

