/************************************************************************
*                                                                       *
*   MSRKinectAudio.h -- This module defines the APIs and CLSIDs for     *
*                    Kinect audio.                                      *
*                                                                       *
*   Copyright (c) Microsoft Corp. All rights reserved.                  *
*                                                                       *
************************************************************************/

#include <INITGUID.H> //To be able to define the guid here


// {4DDF4183-C038-4C8B-AF9D-61E932B58846}
DEFINE_GUID(CLSID_CMSRKinectAudio, 
0x4ddf4183, 0xc038, 0x4c8b, 0xaf, 0x9d, 0x61, 0xe9, 0x32, 0xb5, 0x88, 0x46);


#pragma once

#define MICARRAY_ADAPTIVE_BEAM 0x1100

extern "C" {

// {8C3CEBFA-A35D-497E-BC9A-E9752A8155E0}
DEFINE_GUID(IID_ISoundSourceLocalizer, 
0x8c3cebfa, 0xa35d, 0x497e, 0xbc, 0x9a, 0xe9, 0x75, 0x2a, 0x81, 0x55, 0xe0);


//
// ISoundSourceLocalizer
//
DECLARE_INTERFACE_(ISoundSourceLocalizer, IUnknown) 
{
	STDMETHOD(GetBeam)
        ( THIS_
          double *pdAngle
        ) PURE;

	STDMETHOD(GetPosition)
        ( THIS_
          double *pdAngle,
		  double *pdConfidence
        ) PURE;

	STDMETHOD(SetBeam)
        ( THIS_
          double dBeamAngle
        ) PURE;
};

/*
//To use the DMO in filter mode the geometry needs to be specified. This can be done by including the following:

#include <Ks.h>
#include <Ksmedia.h>

//The KSAUDIO_MIC_ARRAY_GEOMETRY only has room for 1 mic coordinate so we manually allocate space for the other 3 
typedef struct 
{
	KSAUDIO_MIC_ARRAY_GEOMETRY geometry;
	KSAUDIO_MICROPHONE_COORDINATES coordinates[3];
} KINECT_GEOMETRY;

KINECT_GEOMETRY maKinectGeometry = {256, 0, 0, 0, -8726, 8726, 200, 7200, 4, 
{2, 0, -113, 0, 0, 0},
{{2, 0, 36, 0, 0, 0},
{2, 0, 76, 0, 0, 0},
{2, 0, 113, 0, 0, 0}}};
*/

#define MAX_STR_LEN 512
typedef struct
{
    wchar_t szDeviceName[MAX_STR_LEN];
    wchar_t szDeviceID[MAX_STR_LEN];
    int iDeviceIndex;
} KINECT_AUDIO_INFO, *PKINECT_AUDIO_INFO;

//Fills the array of KINECT_AUDIO_INFO structs with the Kinect devices found on the system. 
//
//pDeviceInfo: Array allocated by the caller, upon return contains the device information for up to piDeviceCount devices. Can be null
//to just retrieve the number of items in piDeviceCount
//piDeviceCount [in]: The number of items in the array
//piDeviceCount [out]: The actual number of devices found.
HRESULT GetKinectDeviceInfo(PKINECT_AUDIO_INFO pDeviceInfo, int *piDeviceCount);

}

