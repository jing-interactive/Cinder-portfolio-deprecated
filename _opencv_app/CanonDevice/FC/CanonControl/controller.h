#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "camera.h"
#include "Command.h"
#include "TakePictureCommand.h"
#include "Processor.h"
#include "CameraModel.h"
#include "GetPropertyCommand.h"
#include "SetCapacityCommand.h"
#include "SetPropertyCommand.h"
#include "DownloadCommand.h"
#include <queue>

class Camera;

class Controller
{
public:
	Controller();
	~Controller();
	void run();
    CameraModel*  getCameraModel();
 	void          setCameraRef(EdsCameraRef camera);
 	EdsCameraRef  getCameraRef();
	void StoreAsync( Command *command );

	static void setSavePathName(const char* savePathName)
	{
		strcpy(_savePathName, savePathName);
	}

	static char _savePathName[EDS_MAX_NAME];


	static EdsError EDSCALLBACK propertyEventHandler(EdsPropertyEvent inEvent, 
		                                             EdsPropertyID    inPropertyID, 
		                                             EdsUInt32        inParam, 
		                                             EdsVoid*         inContext)
	{
		return EDS_ERR_OK;
	}

	static EdsError EDSCALLBACK objectEventHandler(EdsObjectEvent inEvent, 
		                                           EdsBaseRef     inRef, 
		                                           EdsVoid*       inContext)
	{
		Controller*	controller = (Controller *)inContext;

		dirItemRef = inRef;

		if(!strcmp(_savePathName, ""))
			controller->StoreAsync(new DownloadCommand(controller->_model, inRef));
		else
			controller->StoreAsync(new DownloadCommand(controller->_model, inRef, controller->_savePathName));

		switch(inEvent)
		{
		case kEdsObjectEvent_DirItemRequestTransfer:
			break;

		default:
			//Object without the necessity is released
			if(inRef != NULL)
			{
				EdsRelease(inRef);
			}
			break;
		}

		return EDS_ERR_OK;
	}

	static EdsError EDSCALLBACK stateEventHandler(EdsStateEvent inEvent, 
		                                          EdsUInt32     inEventData,
												  EdsVoid*      inContext)
	{
		return EDS_ERR_OK;
	}
private:
	CameraModel*  _model;
	Processor     _processor;
};

#endif