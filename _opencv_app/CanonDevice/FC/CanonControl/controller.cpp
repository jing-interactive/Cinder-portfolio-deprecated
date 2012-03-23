#include "controller.h"
#include <queue>
using namespace std;


char Controller::_savePathName[EDS_MAX_NAME] = "";

Controller::Controller()
{
	_model = new CameraModel();
	strcmp(_savePathName, "");
}

Controller::~Controller()
{
   delete _model;
}

 void Controller::setCameraRef(EdsCameraRef camera)
 {
    _model->setCameraObject(camera);
 }

EdsCameraRef Controller::getCameraRef()
{
   return _model->getCameraObject();
}

CameraModel* Controller::getCameraModel()
{
	return _model;
}

void Controller::run()
{
	_processor.start();
	StoreAsync(new GetPropertyCommand (_model, kEdsPropID_Unknown));
	EdsCapacity capacity = {0x7FFFFFFF, 0x1000, 1};
	StoreAsync(new SetCapacityCommand(_model, capacity));
	EdsUInt32 saveTo = kEdsSaveTo_Host;
	StoreAsync(new SetPropertyCommand<EdsUInt32>(_model, kEdsPropID_SaveTo, saveTo));
}

void Controller::StoreAsync( Command *command )
{
	if ( command != NULL )
	{
		_processor.enqueue( command );
	}
}
