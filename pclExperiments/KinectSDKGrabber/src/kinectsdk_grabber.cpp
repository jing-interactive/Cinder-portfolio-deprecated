#include "kinectsdk_grabber.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

#include "../../_common/Kinect/KinectDevice.h"

typedef union
{
	struct /*anonymous*/
	{
		unsigned char Blue;
		unsigned char Green;
		unsigned char Red;
		unsigned char Alpha;
	};
	float float_value;
	long long_value;
} RGBValue;

struct MyKinectDevice : public KinectDevice
{
	MyKinectDevice(pcl::KinectSdkGrabber& grabber):grabber_(grabber){}
	virtual void depth_callback()
	{
		cv::Mat& depth = this->rawDepth;
		grabber_.depthCallback(depth);
	}
	pcl::KinectSdkGrabber& grabber_;
};

namespace pcl
{
	KinectSdkGrabber::KinectSdkGrabber (int device_id)
	{
		// create the signal for a XYZRGB point cloud
		point_cloud_signal_ = createSignal <void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&)>();
		device_ = boost::make_shared<KinectDevice>(device_id);
		cloud_ = boost::make_shared<PointCloud<PointXYZ>>();
	}

	KinectSdkGrabber::~KinectSdkGrabber () {}

	void KinectSdkGrabber::stop ()
	{
		device_->release();
	}

	std::string KinectSdkGrabber::getName () const
	{
		return "SphereGrabber";
	}

	bool KinectSdkGrabber::isRunning () const
	{
		return true; // not running, since this is a triggered grabber
	}

	void KinectSdkGrabber::start ()
	{
		// assemble the point cloud
		//publish cloud
		device_->setup(false, true, false);
	}

	void KinectSdkGrabber::depthCallback(cv::Mat& depth)
	{
		mat2cloud(depth, *cloud_);
		(*point_cloud_signal_)(cloud_);
	}

	void KinectSdkGrabber::mat2cloud( cv::Mat& depth, PointCloud<PointXYZ>& cloud )
	{
		cloud.width = depth.cols;
		cloud.height = depth.rows;
		cloud.resize(cloud.width*cloud.height);
		cloud.is_dense = true;
		
		cv::Mat_<uchar> M(depth);
		for(int i = 0; i < M.rows; i++)
			for(int j = 0; j < M.cols; j++)
			{
				cloud(j,i).x = 50.0f*j;
				cloud(j,i).y = 50.0f*i;
				cloud(j,i).z = M(i,j);
			}
		
	}
}
