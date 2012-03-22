#ifndef CAMERA_H
#define CAMERA_H
#include "../EDSDK.h"
#include "../EDSDKErrors.h"
#include "../EDSDKTypes.h"
#include "controller.h"
#include "DownloadCommand.h"
#include <vector>
class Controller;

class Camera {
public:
	Camera();
	Camera(const char* productName);
	~Camera();
	Controller*   getController();
	void          setCallbacks();
	void          CameraInit(const char* productName);
	void          takePicture();
	void          setSavePathName(const char* savePathName);
private:
	Controller *  _controller;
	EdsCameraRef  cameraRef;
	EdsUInt32     driveLensAmount;
	EdsPoint      p;
	bool          bInit;

/*	void test1();
	void driveLens();
	void setFocus(EdsUInt32 driveLensAmount);
	void zoom_1();
	void zoom_5();
	void zoom_10();
	void zoomPosition(int x, int y);
	void startLV();
	void continueLV();
	void stopLV();
	void takePicture();
	void setTv(EdsInt32 inParam);
	void setAv(EdsInt32 inParam);
	void setIso(EdsInt32 inParam);
	void queuePropertyChange();
	void queueZoomChange();
	std::vector<int> getProperty(EdsPropertyID PropID);*/
};

#endif
