#ifndef KINECTSDK_GRABBER_H
#define	KINECTSDK_GRABBER_H

#include <pcl/io/grabber.h>
#include <pcl/pcl_macros.h>

namespace pcl
{
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
      KinectSdkGrabber ( float center_x, float center_y, float center_z, float radius_, unsigned longitudes, unsigned latitudes);
      virtual ~KinectSdkGrabber ();
      
      virtual void start ();
      virtual void stop ();
      virtual std::string getName () const;
      virtual bool isRunning () const;
      
  private:
    boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > assembleSphere () const;
    float radius_;
    float center_x_;
    float center_y_;
    float center_z_;
    
    unsigned longitudes_;
    unsigned latitudes_;
    
    boost::signals2::signal<void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&)>* point_cloud_rgb_signal_;
  };
}

#endif	/* SPHERE_GRABBER_H */

