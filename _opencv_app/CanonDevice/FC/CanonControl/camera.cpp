 #include "../EDSDK.h"
 #include "../EDSDKErrors.h"
 #include "../EDSDKTypes.h"
#include "controller.h"
#include "Debug.h"

Camera::Camera()
{
	_controller = new Controller();
	bInit = true;
	CameraInit(NULL);
}

Camera::Camera(const char* productName)
{
	_controller = new Controller();
	bInit = false;
	CameraInit(productName);
}

Camera::~Camera()
{
	delete _controller;
}

void Camera::CameraInit(const char* productName)
{
	EdsError err = EdsInitializeSDK();
 	Debug::WriteLine("EDSDK Initialized");
	EdsCameraListRef cameraListRef;
	EdsGetCameraList(&cameraListRef);
	EdsUInt32 numCameras;
	EdsGetChildCount(cameraListRef, &numCameras);

	// 找到第一个跟指定的型号相同的相机。
	// 由于500d不能保存artist的信息，（断电后会丢失），因此暂时只能采取实用productName这个属性来区分上下两个相机。
	// 也许可以实用IDEx这个信息？
	if( productName == NULL)
		EdsGetChildAtIndex(cameraListRef, 0, &cameraRef);
	else{
		for(int i = 0; i < (int) numCameras; i++){
			EdsGetChildAtIndex(cameraListRef, i, &cameraRef);

			EdsError err = EdsOpenSession(cameraRef);
			char buf[256];
			EdsGetPropertyData(cameraRef, kEdsPropID_ProductName, 0, 256, buf);
			/*printf("Camera Name: %s\n", buf);

			if(!strcmp(productName, buf)) break;

			EdsCloseSession(cameraRef);
			cameraRef = NULL;*/
            if(!strcmp(productName, buf))
			{
				printf("Camera Name: %s initialized.\n", buf);
				bInit = true;
				break;
			}
		}
	}

	if (!bInit)
	{
		printf("Camera %s NOT initialized!!!\n", productName);
		return;
	}

	_controller->setCameraRef(cameraRef);
	setCallbacks();
	_controller->run();
}

void Camera::setCallbacks()
{
	//set property event callback
	Debug::showResult("EdsSetPropertyEventHandler", EdsSetPropertyEventHandler(cameraRef, kEdsPropertyEvent_All, &Controller::propertyEventHandler, 0));

	//set object event callback
	Debug::showResult("EdsSetObjectEventHandler", EdsSetObjectEventHandler(cameraRef, kEdsObjectEvent_All, &Controller::objectEventHandler, _controller));

	//set camera state event callback
	Debug::showResult("EdsSetCameraStateEventHandler", EdsSetCameraStateEventHandler(cameraRef, kEdsStateEvent_All, &Controller::stateEventHandler, 0));
	//*********************************

	//inform the SDK to save to host
	EdsUInt32 saveTo = kEdsSaveTo_Host;
	Debug::showResult("EdsSetPropertyData.kEdsPropID_SaveTo", EdsSetPropertyData(cameraRef, kEdsPropID_SaveTo, 0, sizeof(saveTo), &saveTo));

	//set the capacity of the host (this hard-coded value comes from Canon's VC example)
	EdsCapacity capacity = {0x7FFFFFFF, 0x1000, 1};
	Debug::showResult("EdsSetCapacity", EdsSetCapacity(cameraRef, capacity));
}

Controller* Camera::getController()
{
	return _controller;
}


void Camera::takePicture()
{
	bool taken = false;
	bool downloaded = false;
	MSG Msg;
	while(GetMessage(&Msg, NULL, 0, 0))
	{
		TranslateMessage(&Msg);
		DispatchMessage(&Msg);

		//wait for message 275 (WM_TIMER) which appears to indicate no new messages
		if(Msg.message == 275)
		{
			if(taken && downloaded) break;

			if(dirItemRef == NULL)
			{
				if(taken == false){
					_controller->StoreAsync(new TakePictureCommand(_controller->getCameraModel()));
					taken = true;
					//Sleep(500);
				}
			}
			else
			{
				Sleep(500);

				if( numDownloads > 0 ){
					downloaded = true;
					numDownloads = 0;
					printf("downloading.\n");
				}
				
			}
			//Sleep(500);
		}
	}
}

void Camera::setSavePathName(const char* savePathName)
{
	_controller->setSavePathName(savePathName);
}

/*
EdsUInt32 driveLensAmount;

void Camera::setFocus(EdsUInt32 driveLensAmount)
{       
//	Debug::WriteLine("SetFocus: "+driveLensAmount);
	this->driveLensAmount = driveLensAmount;
	c->queue(&Camera::driveLens);   
}
void Camera::driveLens()
{
//	Debug::WriteLine("Focusing: "+driveLensAmount);
	EdsError err = EDS_ERR_OK;
	err = EdsSendCommand(cameraRef,kEdsCameraCommand_DriveLensEvf,driveLensAmount);
	if(err != EDS_ERR_OK)
		;
// 		Debug::WriteLine(err);  

}
void Camera::zoom_1()
{
	EdsUInt32 zoom = kEdsEvfZoom_Fit;
	EdsError err = EDS_ERR_OK;
	err = EdsSetPropertyData(cameraRef, kEdsPropID_Evf_Zoom, zoom , sizeof(zoom), &zoom);
}
void Camera::zoom_5()
{
	EdsUInt32 zoom = kEdsEvfZoom_x5;
	EdsError err = EDS_ERR_OK;
	err = EdsSetPropertyData(cameraRef, kEdsPropID_Evf_Zoom, zoom , sizeof(zoom), &zoom);

}
void Camera::zoom_10()
{
	EdsUInt32 zoom = kEdsEvfZoom_x10;
	EdsError err = EDS_ERR_OK;
	err = EdsSetPropertyData(cameraRef, kEdsPropID_Evf_Zoom, zoom , sizeof(zoom), &zoom);
}

void Camera::zoomPosition(int xi, int yi)
{
	PropertyID = kEdsPropID_Evf_ZoomPosition;
 	p.x = xi*20;    
 	p.y = yi*20;
// 	Debug::WriteLine(p.x + " " +p.y);
	c->queue(&Camera::queueZoomChange);
}
void Camera::queueZoomChange()
{
	EdsError err = EDS_ERR_OK;
	err = EdsSetPropertyData(cameraRef, PropertyID, 0 , sizeof(p), &p);
}
void Camera::setTv(EdsInt32 inParam){
	parameter = inParam;
	PropertyID = kEdsPropID_Tv;
	c->queue(&Camera::queuePropertyChange);
}
void Camera::setAv(EdsInt32 inParam){
	parameter = inParam;
	PropertyID = kEdsPropID_Av;
	c->queue(&Camera::queuePropertyChange);
}
void Camera::setIso(EdsInt32 inParam){
	parameter = inParam;
	PropertyID = kEdsPropID_ISOSpeed;
	c->queue(&Camera::queuePropertyChange);
}
void Camera::queuePropertyChange()
{
	EdsError err = EDS_ERR_OK;
	err = EdsSetPropertyData(cameraRef, PropertyID, 0 , sizeof(parameter), &parameter);
}

std::vector<int> Camera::getProperty(EdsPropertyID PropID){
	EdsError err;
	EdsPropertyDesc *propertyDesc = new EdsPropertyDesc;
	showResult("GetProp", EdsGetPropertyDesc(cameraRef, PropID,propertyDesc));

	int size = propertyDesc->numElements;
	std::vector<int> out(propertyDesc->propDesc,propertyDesc->propDesc+propertyDesc->numElements);
	//memcpy(out,propertyDesc->propDesc,size);
	//std::fill((int)propertyDesc->propDesc, (int)propertyDesc->propDesc + (int)propertyDesc->numElements, out);
	return out;
}





*/