/******************************************************************************
*                                                                             *
*   PROJECT : EOS Digital Software Development Kit EDSDK                      *
*      NAME : TakePictureCommand.h	                                          *
*                                                                             *
*   Description: This is the Sample code to show the usage of EDSDK.          *
*                                                                             *
*                                                                             *
*******************************************************************************
*                                                                             *
*   Written and developed by Camera Design Dept.53                            *
*   Copyright Canon Inc. 2006 All Rights Reserved                             *
*                                                                             *
*******************************************************************************/

#pragma once

#include "Command.h"
#include "../EDSDK.h"
#include "Debug.h"
#include "controller.h"
#include "stdio.h"

class TakePictureCommand : public Command
{

public:
	TakePictureCommand(CameraModel *model) : Command(model){}


	// Execute command	
	virtual bool execute()
	{
		EdsError err = EDS_ERR_OK;
		bool	 locked = false;

		// For cameras earlier than the 30D , the UI must be locked before commands are reissued
		if( _model->isLegacy())
		{
			err = EdsSendStatusCommand(_model->getCameraObject(), kEdsCameraStatusCommand_UILock, 0);

			if(err == EDS_ERR_OK)
			{
				locked = true;
			}		
		}
//		printf("%d\n",_model->test);

		EdsCameraRef ref = _model->getCameraObject();
		//        EdsCameraRef ref = _controller->getCameraRef();

		//Taking a picture
		err = EdsSendCommand(ref, kEdsCameraCommand_TakePicture, 0);

		//It releases it when locked
		if(locked)
		{
			err = EdsSendStatusCommand(_model->getCameraObject(), kEdsCameraStatusCommand_UIUnLock, 0);
		}

		//Notification of error
		if(err != EDS_ERR_OK)
		{
			// It retries it at device busy
			if(err == EDS_ERR_DEVICE_BUSY)
			{
				Debug::WriteLine("Eds: DeviceBusy");
				return true;
			}

			Debug::WriteLine(err);
		}

		return true;
	}
	
};