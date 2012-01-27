#include "Fushion.h"
#include "../../_common/pcl/filters.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#define PARAM_WINDOW "param_panel"

bool KinectFushionApp::setup()
{
	HRESULT hr = KinectDevice::setup(false, true, false);
	if (FAILED(hr))
		return false;

	cv::namedWindow(PARAM_WINDOW);
	//wait
	_evt.wait();
	boost::mutex::scoped_lock lock (_mtx);
	{
		_raw_viewer = NEW_XYZRGB_CLOUD_VIEWER(_cloud_raw);
		_raw_viewer->_distance = 7;
		_raw_viewer->_far = 100000;

		_icp_viewer = NEW_XYZRGB_CLOUD_VIEWER(_cloud_registered);
		_icp_viewer->_distance = 7;
		_icp_viewer->_far = 100000;

		cv::createTrackbar("near", PARAM_WINDOW, &_thresh_near, 100);
		cv::createTrackbar("far", PARAM_WINDOW, &_thresh_far, 100);
	}

	return true;
}


void KinectFushionApp::onDepthData(const cv::Mat& depth_u16)
{
	if (exit)
		return;

	static int fps = 0;
	FPS_CALC (fps);

	boost::mutex::scoped_lock lock (_mtx);
	{
		mat_to_cloud(depth_u16, *_cloud_raw);
		thresholdDepth(_cloud_raw, _cloud_registered, _thresh_near*0.01*Z_FAR,_thresh_far*0.01*Z_FAR);
	}

	if (_raw_viewer.empty())
	{
		_evt.set();
	}
	else
	{
		_raw_viewer->setInputCloud(_cloud_raw);
		_icp_viewer->setInputCloud(_cloud_registered);
		_raw_viewer->update();
		_icp_viewer->update();

		_icp_viewer->_sum_mouse_dx = _raw_viewer->_sum_mouse_dx;
		_icp_viewer->_sum_mouse_dy = _raw_viewer->_sum_mouse_dy;
	}

	_cloud = _model;

	//	pcl::io::savePLYFile("kinect_raw.ply", *_cloud_raw);
	//	pcl::io::savePLYFile("kinect_reg.ply", *_cloud_registered);
} 