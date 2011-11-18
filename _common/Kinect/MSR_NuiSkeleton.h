/************************************************************************
*                                                                       *
*   MSR_NuiSkeleton.h -- This module defines the APIs for the Natural   *
*                    User Interface(NUI) skeleton services.             *
*                                                                       *
*   Copyright (c) Microsoft Corp. All rights reserved.                  *
*                                                                       *
************************************************************************/
#pragma once

#ifndef NUIAPI
#error "You must include nuiapi.h rather than including nuiskeleton.h directly"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FLT_EPSILON
  #define FLT_EPSILON     1.192092896e-07F        /* smallest such that 1.0+FLT_EPSILON != 1.0 */
#endif

typedef enum _NUI_SKELETON_POSITION_INDEX
{
    NUI_SKELETON_POSITION_HIP_CENTER = 0,
    NUI_SKELETON_POSITION_SPINE,
    NUI_SKELETON_POSITION_SHOULDER_CENTER,
    NUI_SKELETON_POSITION_HEAD,
    NUI_SKELETON_POSITION_SHOULDER_LEFT,
    NUI_SKELETON_POSITION_ELBOW_LEFT,
    NUI_SKELETON_POSITION_WRIST_LEFT,
    NUI_SKELETON_POSITION_HAND_LEFT,
    NUI_SKELETON_POSITION_SHOULDER_RIGHT,
    NUI_SKELETON_POSITION_ELBOW_RIGHT,
    NUI_SKELETON_POSITION_WRIST_RIGHT,
    NUI_SKELETON_POSITION_HAND_RIGHT,
    NUI_SKELETON_POSITION_HIP_LEFT,
    NUI_SKELETON_POSITION_KNEE_LEFT,
    NUI_SKELETON_POSITION_ANKLE_LEFT,
    NUI_SKELETON_POSITION_FOOT_LEFT,
    NUI_SKELETON_POSITION_HIP_RIGHT,
    NUI_SKELETON_POSITION_KNEE_RIGHT,
    NUI_SKELETON_POSITION_ANKLE_RIGHT,
    NUI_SKELETON_POSITION_FOOT_RIGHT,
    NUI_SKELETON_POSITION_COUNT
} NUI_SKELETON_POSITION_INDEX;

typedef struct _NUI_TRANSFORM_SMOOTH_PARAMETERS {
    FLOAT fSmoothing;
    FLOAT fCorrection;
    FLOAT fPrediction;
    FLOAT fJitterRadius;
    FLOAT fMaxDeviationRadius;
} NUI_TRANSFORM_SMOOTH_PARAMETERS;

//
//  Number of NUI_SKELETON_DATA elements in NUI_SKELETON_FRAME
//

#define NUI_SKELETON_COUNT 6

//
//  Number of NUI_SKELETON_DATA elements that can be in the NUI_SKELETON_TRACKED state
//

#define NUI_SKELETON_MAX_TRACKED_COUNT 2

//
//  Tracking IDs start at 1
//

#define NUI_SKELETON_INVALID_TRACKING_ID 0

typedef enum _NUI_SKELETON_POSITION_TRACKING_STATE
{
    NUI_SKELETON_POSITION_NOT_TRACKED = 0,
    NUI_SKELETON_POSITION_INFERRED,
    NUI_SKELETON_POSITION_TRACKED
} NUI_SKELETON_POSITION_TRACKING_STATE;

typedef enum _NUI_SKELETON_TRACKING_STATE
{
    NUI_SKELETON_NOT_TRACKED = 0,
    NUI_SKELETON_POSITION_ONLY,
    NUI_SKELETON_TRACKED
} NUI_SKELETON_TRACKING_STATE;

typedef struct _NUI_SKELETON_DATA
{
  NUI_SKELETON_TRACKING_STATE eTrackingState;
  DWORD dwTrackingID;
  DWORD dwEnrollmentIndex_NotUsed;
  DWORD dwUserIndex;
  Vector4 Position;
  Vector4 SkeletonPositions[NUI_SKELETON_POSITION_COUNT];
  NUI_SKELETON_POSITION_TRACKING_STATE eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_COUNT];
  DWORD dwQualityFlags;
} NUI_SKELETON_DATA;


//
// Force a link error if the size of NUI_SKELETON_DATA is different
// between the static library and the header included by the title.
//

__declspec(selectany) DWORD NuiSkeletonDataSize = sizeof(NUI_SKELETON_DATA);


#define NUI_SKELETON_QUALITY_CLIPPED_RIGHT  0x00000001
#define NUI_SKELETON_QUALITY_CLIPPED_LEFT   0x00000002
#define NUI_SKELETON_QUALITY_CLIPPED_TOP    0x00000004
#define NUI_SKELETON_QUALITY_CLIPPED_BOTTOM 0x00000008

#pragma pack(push, 16)
typedef struct _NUI_SKELETON_FRAME
{
  LARGE_INTEGER         liTimeStamp;
  DWORD                 dwFrameNumber;
  DWORD                 dwFlags;
  Vector4              vFloorClipPlane;
  Vector4              vNormalToGravity;
  NUI_SKELETON_DATA     SkeletonData[NUI_SKELETON_COUNT];
} NUI_SKELETON_FRAME;
#pragma pack(pop)


#define NUI_SKELETON_FRAME_FLAG_CAMERA_MOTION      	0x00000001
#define NUI_SKELETON_FRAME_FLAG_EXTRAPOLATED_FLOOR 	0x00000002
#define NUI_SKELETON_FRAME_FLAG_UPPER_BODY_SKELETON	0x00000004

#define NUI_SKELETON_TRACKING_FLAG_SUPPRESS_NO_FRAME_DATA 0x00000001

NUIAPI
HRESULT
NuiSkeletonTrackingEnable(
    _In_opt_ HANDLE hNextFrameEvent,
    _In_     DWORD  dwFlags
    );

NUIAPI
HRESULT
NuiSkeletonTrackingDisable(
    );

NUIAPI
HRESULT
NuiSkeletonGetNextFrame(
    _In_  DWORD dwMillisecondsToWait,
    _Out_ NUI_SKELETON_FRAME *pSkeletonFrame
    );

NUIAPI
HRESULT NuiTransformSmooth(
         NUI_SKELETON_FRAME *pSkeletonFrame,
         const NUI_TRANSFORM_SMOOTH_PARAMETERS *pSmoothingParams
    );

#ifdef __cplusplus
} //close extern "C"

// Assuming a pixel resolution of 320x240
// x_meters = (x_pixelcoord - 160) * NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 * z_meters;
// y_meters = (y_pixelcoord - 120) * NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 * z_meters;
#define NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 (NUI_CAMERA_DEPTH_NOMINAL_INVERSE_FOCAL_LENGTH_IN_PIXELS)
 
// Assuming a pixel resolution of 320x240
// x_pixelcoord = (x_meters) * NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 / z_meters + 160;
// y_pixelcoord = (y_meters) * NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 / z_meters + 120;
#define NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 (NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS)

// Added these new methods to return floating point XY values that range from 0-1, instead of assuming
// depth is always 320x240. This is exactly the same code as from the XBOX, except post-fixed with "F"
// for floating-point

inline
VOID
NuiTransformSkeletonToDepthImageF(
    Vector4 vPoint,
    _Out_ float *pfDepthX, // returns 0-1
    _Out_ float *pfDepthY,
    _Out_ USHORT *pusDepthValue
    )
{
    if((pfDepthX == NULL) || (pfDepthY == NULL) || (pusDepthValue == NULL))
    {
        return;
    }
    
    //
    // Requires a valid depth value.
    //
    
    if(vPoint.z > FLT_EPSILON)
    {
        //
        // Center of depth sensor is at (0,0,0) in skeleton space, and
        // and (160,120) in depth image coordinates.  Note that positive Y
        // is up in skeleton space and down in image coordinates.
        //
        
        *pfDepthX = static_cast<float>( 0.5f + vPoint.x * NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 / ( vPoint.z * 320.0f ) );
        *pfDepthY = static_cast<float>( 0.5f - vPoint.y * NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 / ( vPoint.z * 240.0f ) );
        
        //
        //  Depth is in meters in skeleton space.
        //  The depth image pixel format has depth in millimeters shifted left by 3.
        //
        
        *pusDepthValue = static_cast<USHORT>(vPoint.z *1000) << 3;
    } else
    {
        *pfDepthX = 0;
        *pfDepthY = 0;
        *pusDepthValue = 0;
    }
}

inline
VOID
NuiTransformSkeletonToDepthImageF(
    Vector4 vPoint,
    _Out_ FLOAT *pfDepthX, // returns 0-1
    _Out_ FLOAT *pfDepthY
    )
{
    if((pfDepthX == NULL) || (pfDepthY == NULL))
    {
        return;
    }
    
    //
    // Requires a valid depth value.
    //
    
    if(vPoint.z > FLT_EPSILON)
    {
        //
        // Center of depth sensor is at (0,0,0) in skeleton space, and
        // and (160,120) in depth image coordinates.  Note that positive Y
        // is up in skeleton space and down in image coordinates.
        //
        
        *pfDepthX = 0.5f + vPoint.x * ( NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 / vPoint.z ) / 320.0f;
        *pfDepthY = 0.5f - vPoint.y * ( NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 / vPoint.z ) / 240.0f;
        
    } else
    {
        *pfDepthX = 0.0f;
        *pfDepthY = 0.0f;
    }
}

inline
Vector4
NuiTransformDepthImageToSkeletonF(
    _In_ float fDepthX,
    _In_ float fDepthY,
    _In_ USHORT usDepthValue
    )
{
    
    //
    //  Depth is in meters in skeleton space.
    //  The depth image pixel format has depth in millimeters shifted left by 3.
    //
    
    FLOAT fSkeletonZ = static_cast<FLOAT>(usDepthValue >> 3) / 1000.0f;

    //
    // Center of depth sensor is at (0,0,0) in skeleton space, and
    // and (160,120) in depth image coordinates.  Note that positive Y
    // is up in skeleton space and down in image coordinates.
    //
    
    FLOAT fSkeletonX = static_cast<FLOAT>(fDepthX - 0.5f) * ( NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 * fSkeletonZ ) * 320.0f;
    FLOAT fSkeletonY = static_cast<FLOAT>(0.5f - fDepthY) * ( NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 * fSkeletonZ ) * 240.0f;

    //
    // Return the result as a vector.
    //
        
    Vector4 v4;
    v4.x = fSkeletonX;
    v4.y = fSkeletonY;
    v4.z = fSkeletonZ;
    v4.w = 1.0f;
    return v4;
}

#endif

