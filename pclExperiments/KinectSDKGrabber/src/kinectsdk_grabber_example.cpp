//#include <boost/thread/thread.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <windows.h>

#include "kinectsdk_grabber.h"

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

typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr CloudConstPtr;
void globalFunction (const CloudConstPtr& cloud)
{

}

template <typename PointType>
class KinectVoxelGrid
{
public:
	typedef pcl::PointCloud<PointType> Cloud;
	typedef typename Cloud::Ptr CloudPtr;
	typedef typename Cloud::ConstPtr CloudConstPtr;

	KinectVoxelGrid (int device_id = 0, 
		const std::string& field_name = "z", float min_v = 0, float max_v = 5.0,
		float leaf_size_x = 0.01, float leaf_size_y = 0.01, float leaf_size_z = 0.01)
		:// viewer ("PCL OpenNI VoxelGrid Viewer"),
	device_id_(device_id)
	{
		grid_.setLeafSize (leaf_size_x, leaf_size_y, leaf_size_z);
		grid_.setFilterFieldName (field_name);
		grid_.setFilterLimits (min_v, max_v);
	}

	void 
		cloud_cb_ (const CloudConstPtr& cloud)
	{
		set (cloud);
	}

	void
		set (const CloudConstPtr& cloud)
	{
		//lock while we set our cloud;
		//		boost::mutex::scoped_lock lock (mtx_);
		cloud_  = cloud;
	}

	CloudPtr
		get ()
	{
		//lock while we swap our cloud and reset it.
		//		boost::mutex::scoped_lock lock (mtx_);
		CloudPtr temp_cloud (new Cloud);

		grid_.setInputCloud (cloud_);
		grid_.filter (*temp_cloud);

		return (temp_cloud);
	}

	void
		run ()
	{
		pcl::Grabber* interface = new pcl::KinectSdkGrabber (device_id_);

		boost::function<void (const CloudConstPtr&)> f = boost::bind (&KinectVoxelGrid::cloud_cb_, this, _1);
		boost::signals2::connection c = interface->registerCallback (f);

		interface->start ();

		// 		while (!viewer.wasStopped ())
		// 		{
		// 			if (cloud_)
		// 			{
		// 				FPS_CALC ("drawing");
		// 				//the call to get() sets the cloud_ to null;
		// 				viewer.showCloud (get ());
		// 			}
		// 		}

		interface->stop ();
	}

	pcl::ApproximateVoxelGrid<PointType> grid_;
	//	pcl::visualization::CloudViewer viewer;
	int device_id_;
	//	boost::mutex mtx_;
	CloudConstPtr cloud_;
};

int main(int argc, char** argv)
{
	pcl::Grabber* grabber = new pcl::KinectSdkGrabber(0);

	if (grabber->providesCallback<void (const CloudConstPtr&)>() )
	{
		boost::function<void (const CloudConstPtr&)> global_function =
			boost::bind(globalFunction, _1);
		grabber->registerCallback(global_function);
	}

	grabber->start();
	while(true);
	grabber->stop ();

	return 0;
}

