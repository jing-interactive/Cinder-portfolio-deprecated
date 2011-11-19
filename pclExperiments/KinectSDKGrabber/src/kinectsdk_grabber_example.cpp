#include <boost/thread/thread.hpp>
#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/incremental_registration.h>
 
#include "../../_common/Kinect/KinectDevice.h"

#include <windows.h>

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
	do \
{ \
	static unsigned count = 0;\
	static double last = pcl::getTime ();\
	double now = pcl::getTime (); \
	++count; \
	if (now - last >= 1.0) \
	{ \
	std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
	count = 0; \
	last = now; \
	} \
}while(false)
#else
#define FPS_CALC(_WHAT_) \
	do \
{ \
}while(false)
#endif

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr CloudConstPtr; 

template <typename type>
void mat2cloud(const cv::Mat& depth, Cloud& cloud )
{
	cloud.width = depth.cols;
	cloud.height = depth.rows;
	cloud.resize(cloud.width*cloud.height);
	cloud.is_dense = true;

	cv::Mat_<type> M(depth);
	for(int i = 0; i < M.rows; i++)
		for(int j = 0; j < M.cols; j++)
		{
			cloud(j,i).x = 4400/320.0f*j;
			cloud(j,i).y = 3200/240.0f*i;
			cloud(j,i).z = M(i,j);
		}
}

struct MyKinectDevice : public KinectDevice
{ 
	MyKinectDevice(int device_id)
		:KinectDevice(device_id)
	{
		cloud_ = CloudPtr(new Cloud);
		cloud_raw_ = CloudPtr(new Cloud);
		cloud_registered_ = CloudPtr(new Cloud);
		model_ = CloudPtr(new Cloud);

		grid_.setLeafSize (0.01f, 0.01f, 0.01f);
		grid_.setFilterFieldName ("z");
		grid_.setFilterLimits (0, 5.);
	}

	void setup()
	{
		KinectDevice::setup(false, true, false);
	}

	virtual void onDepthEvent(const cv::Mat& depth_u16)
	{ 
		FPS_CALC ("callback");
		boost::mutex::scoped_lock lock (mtx_);

		mat2cloud<ushort>(depth_u16, *cloud_raw_);

		reg_.setInputCloud(cloud_raw_);
		reg_.setDownsamplingLeafSizeInput(0.03);
		reg_.setDownsamplingLeafSizeModel(0.03);
		reg_.setRegistrationDistanceThreshold(0.5);

		bool use_vanilla_icp = false;
		reg_.align(*cloud_registered_, use_vanilla_icp);
		pcl::copyPointCloud(*reg_.getModel(), *model_);

		cloud_ = model_;

	//	pcl::io::savePLYFile("kinect.ply", *cloud_);
	}
	CloudPtr cloud_;
	CloudPtr cloud_raw_;
	CloudPtr cloud_registered_;
	CloudPtr model_;
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> grid_;
	pcl::registration::IncrementalRegistration<pcl::PointXYZ> reg_;

	boost::mutex mtx_;
};

 
int main(int argc, char** argv)
{
	MyKinectDevice device(0);

	device.setup();

	while (true)
	{
		//do some rendering
		::Sleep(30);
	}

 	return 0;
}