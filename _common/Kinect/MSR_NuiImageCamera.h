/************************************************************************
*                                                                       *
*   MSR_NuiImageCamera.h -- This module defines the APIs for the Natural*
*                       User Interface(NUI) image and camera services.  *
*                                                                       *
*   Copyright (c) Microsoft Corp. All rights reserved.                  *
*                                                                       *
************************************************************************/
#pragma once

#ifndef NUIAPI
#error "You must include nuiapi.h rather than including nuiimagecamera.h directly"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _NUI_IMAGE_TYPE
{
  NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX = 0,            // USHORT
  NUI_IMAGE_TYPE_COLOR,                                 // RGB32 data
  NUI_IMAGE_TYPE_COLOR_YUV,                             // YUY2 stream from camera h/w, but converted to RGB32 before user getting it.
  NUI_IMAGE_TYPE_COLOR_RAW_YUV,                         // YUY2 stream from camera h/w.
  NUI_IMAGE_TYPE_DEPTH,                                 // USHORT
} NUI_IMAGE_TYPE;

typedef enum _NUI_IMAGE_RESOLUTION
{
  NUI_IMAGE_RESOLUTION_INVALID = -1,
  NUI_IMAGE_RESOLUTION_80x60 = 0,
  NUI_IMAGE_RESOLUTION_320x240,
  NUI_IMAGE_RESOLUTION_640x480,
  NUI_IMAGE_RESOLUTION_1280x1024                        // for hires color only
} NUI_IMAGE_RESOLUTION;

inline void NuiImageResolutionToSize( NUI_IMAGE_RESOLUTION res, DWORD & refWidth, DWORD & refHeight )
{
    switch( res )
    {
    case NUI_IMAGE_RESOLUTION_80x60:
        refWidth = 80;
        refHeight = 60;
        break;
    case NUI_IMAGE_RESOLUTION_320x240:
        refWidth = 320;
        refHeight = 240;
        break;
    case NUI_IMAGE_RESOLUTION_640x480:
        refWidth = 640;
        refHeight = 480;
        break;
    case NUI_IMAGE_RESOLUTION_1280x1024 :
        refWidth = 1280;
        refHeight = 1024;
        break;
    default:
        refWidth = 0;
        refHeight = 0;
        break;
    }
}

#define NUI_IMAGE_PLAYER_INDEX_SHIFT          3
#define NUI_IMAGE_PLAYER_INDEX_MASK           ((1 << NUI_IMAGE_PLAYER_INDEX_SHIFT)-1)
#define NUI_IMAGE_DEPTH_MAXIMUM               ((4000 << NUI_IMAGE_PLAYER_INDEX_SHIFT) | NUI_IMAGE_PLAYER_INDEX_MASK)
#define NUI_IMAGE_DEPTH_MINIMUM               (800 << NUI_IMAGE_PLAYER_INDEX_SHIFT)
#define NUI_IMAGE_DEPTH_NO_VALUE              0

#define NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS         (285.63f)   // Based on 320x240 pixel size.
#define NUI_CAMERA_DEPTH_NOMINAL_INVERSE_FOCAL_LENGTH_IN_PIXELS (3.501e-3f) // (1/NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS)
#define NUI_CAMERA_DEPTH_NOMINAL_DIAGONAL_FOV                   (70.0f)
#define NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV                 (58.5f)
#define NUI_CAMERA_DEPTH_NOMINAL_VERTICAL_FOV                   (45.6f)

#define NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS         (531.15f)   // Based on 640x480 pixel size.
#define NUI_CAMERA_COLOR_NOMINAL_INVERSE_FOCAL_LENGTH_IN_PIXELS (1.83e-3f)  // (1/NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS)
#define NUI_CAMERA_COLOR_NOMINAL_DIAGONAL_FOV                   ( 73.9f)
#define NUI_CAMERA_COLOR_NOMINAL_HORIZONTAL_FOV                 ( 62.0f)
#define NUI_CAMERA_COLOR_NOMINAL_VERTICAL_FOV                   ( 48.6f)

typedef struct _NUI_IMAGE_VIEW_AREA
{
    int					   eDigitalZoom_NotUsed;
    LONG                   lCenterX_NotUsed;
    LONG                   lCenterY_NotUsed;
} NUI_IMAGE_VIEW_AREA;

typedef struct _NUI_LOCKED_RECT
{
    INT                 Pitch;
    void *              pBits;
} NUI_LOCKED_RECT;

typedef struct _NUI_SURFACE_DESC
{
    UINT                Width;
    UINT                Height;
} NUI_SURFACE_DESC;

interface INuiFrameTexture
{
public:
    STDMETHOD_(int, BufferLen)() PURE;
    STDMETHOD_(int, Pitch)() PURE;
    STDMETHOD(LockRect)(UINT Level, NUI_LOCKED_RECT* pLockedRect, CONST RECT* pRectUsuallyNull, DWORD Flags) PURE;
    STDMETHOD(GetLevelDesc)(UINT Level, NUI_SURFACE_DESC* pDesc) PURE;
    STDMETHOD(UnlockRect)(UINT Level) PURE;
};

// TODO: Remove these types prior to commercial release.

// These typedefs are provided for compatibility with Kinect SDK Beta 1,
// but are now deprecated.

typedef INuiFrameTexture __declspec(deprecated("Use INuiFrameTexture instead of NuiImageBuffer")) NuiImageBuffer;
typedef NUI_LOCKED_RECT __declspec(deprecated("Use NUI_LOCKED_RECT instead of KINECT_LOCKED_RECT")) KINECT_LOCKED_RECT;
typedef NUI_SURFACE_DESC __declspec(deprecated("Use NUI_SURFACE_DESC instead of KINECT_SURFACE_DESC")) KINECT_SURFACE_DESC;

typedef struct _NUI_IMAGE_FRAME
{
  LARGE_INTEGER             liTimeStamp;
  DWORD                     dwFrameNumber;
  NUI_IMAGE_TYPE            eImageType;
  NUI_IMAGE_RESOLUTION      eResolution;
  INuiFrameTexture*         pFrameTexture;
  DWORD                     dwFrameFlags_NotUsed;  
  NUI_IMAGE_VIEW_AREA       ViewArea_NotUsed;
} NUI_IMAGE_FRAME;

// the max # of NUI output frames you can hold w/o releasing
#define NUI_IMAGE_STREAM_FRAME_LIMIT_MAXIMUM 4

// return S_FALSE instead of E_NUI_FRAME_NO_DATA if NuiImageStreamGetNextFrame( ) doesn't have a frame ready and a timeout != INFINITE is used
#define NUI_IMAGE_STREAM_FLAG_SUPPRESS_NO_FRAME_DATA  0x00010000

NUIAPI
HRESULT
NuiImageStreamOpen(
    _In_ NUI_IMAGE_TYPE eImageType,
    _In_ NUI_IMAGE_RESOLUTION eResolution,
    _In_ DWORD dwImageFrameFlags_NotUsed,
    _In_ DWORD dwFrameLimit,
    _In_opt_ HANDLE hNextFrameEvent,
    _Out_ HANDLE *phStreamHandle
    );

NUIAPI
HRESULT
NuiImageStreamGetNextFrame(
    _In_  HANDLE hStream,
    _In_  DWORD dwMillisecondsToWait,
    _Deref_out_ CONST NUI_IMAGE_FRAME **ppcImageFrame
    );

NUIAPI
HRESULT
NuiImageStreamReleaseFrame(
    _In_ HANDLE hStream,
    CONST NUI_IMAGE_FRAME *pImageFrame
    );

NUIAPI    
HRESULT
NuiImageGetColorPixelCoordinatesFromDepthPixel(
    _In_ NUI_IMAGE_RESOLUTION eColorResolution,
    _In_opt_ CONST NUI_IMAGE_VIEW_AREA *pcViewArea,
    _In_ LONG   lDepthX,
    _In_ LONG   lDepthY,
    _In_ USHORT usDepthValue,
    _Out_ LONG *plColorX,
    _Out_ LONG *plColorY
    );    

#define NUI_CAMERA_ELEVATION_MAXIMUM  27
#define NUI_CAMERA_ELEVATION_MINIMUM (-27)

NUIAPI
HRESULT
NuiCameraElevationSetAngle(
    _In_ LONG lAngleDegrees
    );

NUIAPI
HRESULT
NuiCameraElevationGetAngle(
    _In_ LONG * plAngleDegrees
    );

#ifdef __cplusplus
} //close extern "C"
#endif

