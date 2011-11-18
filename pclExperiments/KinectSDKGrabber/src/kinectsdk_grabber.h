#ifndef KINECTSDK_GRABBER_H
#define	KINECTSDK_GRABBER_H

#include <pcl/io/grabber.h>
#include <pcl/pcl_macros.h>

class KinectDevice;

namespace cv
{
	class Mat;
}

namespace pcl
{
	struct PointXYZ;
	struct PointXYZRGB;
	template <typename T> class PointCloud;

	/** \brief Kinect grabber based on Microsoft Kinect SDK.
	* It's easier to use than openNI interface.
	* \author vinjn.z
	* \ingroup io
	*/
	class PCL_EXPORTS KinectSdkGrabber : public Grabber
	{
	public:
		KinectSdkGrabber (int device_id = 0);
		virtual ~KinectSdkGrabber ();

		virtual void start ();
		virtual void stop ();
		virtual std::string getName () const;
		virtual bool isRunning () const;

		void depthCallback(cv::Mat& depth);
		void mat2cloud( cv::Mat& depth, PointCloud<PointXYZ>& cloud_ );

	private:

		boost::signals2::signal<void (const boost::shared_ptr<const PointCloud<PointXYZ> >&)>* point_cloud_signal_;
		boost::shared_ptr<KinectDevice> device_;
		boost::shared_ptr<PointCloud<PointXYZ> > cloud_;
	};
}

#endif	/* SPHERE_GRABBER_H */

