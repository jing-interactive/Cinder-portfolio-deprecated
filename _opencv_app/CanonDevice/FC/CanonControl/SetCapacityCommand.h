/******************************************************************************
*                                                                             *
*   PROJECT : EOS Digital Software Development Kit EDSDK                      *
*      NAME : SaveCapacityCommand.h	                                          *
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

#include "Command.h"
#include "../EDSDK.h"

class SetCapacityCommand : public Command
{
private:
	EdsCapacity _capacity;

public:
	SetCapacityCommand(CameraModel *model) : Command(model){}
	SetCapacityCommand(CameraModel *model, EdsCapacity capacity) : Command(model), _capacity(capacity) {}

	void setCapacity(EdsCapacity capacity)
	{
		_capacity = capacity;
	}

	// Execute command	
	virtual bool execute()
	{
		// It is a function only of the model since 30D.
		if(! _model->isLegacy())
		{
			EdsError err = EDS_ERR_OK;

			//Acquisition of the number of sheets that can be taken a picture
			if(err == EDS_ERR_OK)
			{
				err = EdsSetCapacity( _model->getCameraObject(), _capacity);
			}
		}

		return true;
	}


};