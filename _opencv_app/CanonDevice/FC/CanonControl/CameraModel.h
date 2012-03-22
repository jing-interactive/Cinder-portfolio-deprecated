/******************************************************************************
*                                                                             *
*   PROJECT : EOS Digital Software Development Kit EDSDK                      *
*      NAME : CameraModel.h	                                                  *
*                                                                             *
*   Description: This is the Sample code to show the usage of EDSDK.          *
*                                                                             *
*                                                                             *
*******************************************************************************
*                                                                             *
*   Written and developed by Camera Design Dept.53                            *
*   Copyright Canon Inc. 2006-2008 All Rights Reserved                        *
*                                                                             *
*******************************************************************************/

#pragma once

#include "../EDSDK.h"


class CameraModel 
{
private:
	EdsCameraRef _camera;
	EdsChar      _modelName[EDS_MAX_NAME];

	// Taking a picture parameter
	EdsUInt32 _AEMode;
	EdsUInt32 _Av;
	EdsUInt32 _Tv;
	EdsUInt32 _Iso;
	EdsUInt32 _MeteringMode;
	EdsUInt32 _ExposureCompensation;
	EdsUInt32 _ImageQuality;
	EdsUInt32 _AvailableShot;
	EdsUInt32 _evfMode;
	EdsUInt32 _evfOutputDevice;
	EdsUInt32 _evfDepthOfFieldPreview;
	EdsUInt32 _evfZoom;
	EdsPoint  _evfZoomPosition;
	EdsRect	  _evfZoomRect;
	EdsUInt32 _evfAFMode;

	EdsFocusInfo _focusInfo;

//Access to camera
public:
	virtual bool isLegacy()
	{
		return false;
	}
	EdsCameraRef getCameraObject() const {return _camera;}
	void setCameraObject(EdsCameraRef camera){ _camera = camera; }

	void setPropertyUInt32(EdsUInt32 propertyID, EdsUInt32 value)	
	{
		switch(propertyID) 
		{
		case kEdsPropID_AEMode:					setAEMode(value);					break;
		case kEdsPropID_Tv:						setTv(value);						break;		               
		case kEdsPropID_Av:						setAv(value);						break;           	  
		case kEdsPropID_ISOSpeed:				setIso(value);						break;       
		case kEdsPropID_MeteringMode:			setMeteringMode(value);				break;       
		case kEdsPropID_ExposureCompensation:	setExposureCompensation(value);		break;
		case kEdsPropID_ImageQuality:			setImageQuality(value);				break;
		case kEdsPropID_Evf_Mode:				setEvfMode(value);					break;
		case kEdsPropID_Evf_OutputDevice:		setEvfOutputDevice(value);			break;
		case kEdsPropID_Evf_DepthOfFieldPreview:setEvfDepthOfFieldPreview(value);	break;	
		case kEdsPropID_Evf_AFMode:				setEvfAFMode(value);				break;
		}
	}
	void setAEMode(EdsUInt32 value )				{ _AEMode = value;}
	void setTv( EdsUInt32 value )					{ _Tv = value;}
	void setAv( EdsUInt32 value )					{ _Av = value;}
	void setIso( EdsUInt32 value )					{ _Iso = value; }
	void setMeteringMode( EdsUInt32 value )			{ _MeteringMode = value; }
	void setExposureCompensation( EdsUInt32 value)	{ _ExposureCompensation = value; }
	void setImageQuality( EdsUInt32 value)			{ _ImageQuality = value; }
	void setEvfMode( EdsUInt32 value)				{ _evfMode = value; }
	void setEvfOutputDevice( EdsUInt32 value)		{ _evfOutputDevice = value; }
	void setEvfDepthOfFieldPreview( EdsUInt32 value){ _evfDepthOfFieldPreview = value; }
	void setEvfZoom( EdsUInt32 value)				{ _evfZoom = value; }
	void setEvfZoomPosition( EdsPoint value)		{ _evfZoomPosition = value; }
	void setEvfZoomRect( EdsRect value)				{ _evfZoomRect = value; }
	void setModelName(EdsChar *modelName)			{ strcpy(_modelName, modelName); }
	void setEvfAFMode( EdsUInt32 value)				{ _evfAFMode = value; }
	void setFocusInfo( EdsFocusInfo value)				{ _focusInfo = value; }

	void setPropertyString(EdsUInt32 propertyID, EdsChar *str)	
	{	
		switch(propertyID) 
		{
		case kEdsPropID_ProductName:			setModelName(str);					break;
		}
	}

	void setProeprtyFocusInfo(EdsUInt32 propertyID, EdsFocusInfo info)
	{
		switch(propertyID) 
		{
		case kEdsPropID_FocusInfo:				setFocusInfo(info);				break;
		}
	}

};

