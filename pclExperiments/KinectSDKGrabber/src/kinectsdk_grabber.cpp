#include "kinectsdk_grabber.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

namespace pcl
{
  KinectSdkGrabber::KinectSdkGrabber ( float center_x, float center_y, float center_z, float radius, unsigned longitudes, unsigned latitudes)
  : radius_ (radius)
  , center_x_ (center_x)
  , center_y_ (center_y)
  , center_z_ (center_z)
  , longitudes_ (longitudes)
  , latitudes_ (latitudes)
  {
    // create the signal for a XYZRGB point cloud
    point_cloud_rgb_signal_ = createSignal <void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&)>();
  }
  
  KinectSdkGrabber::~KinectSdkGrabber () {}
  
  void KinectSdkGrabber::stop ()
  {
    // trigger based -> do nothing
  }
  
  std::string KinectSdkGrabber::getName () const
  {
    return "SphereGrabber";
  }
  
  bool KinectSdkGrabber::isRunning () const
  {
    return false; // not running, since this is a triggered grabber
  }
  
  void KinectSdkGrabber::start ()
  {
    // assemble the point cloud
    boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > cloud = assembleSphere ();
    
    //publish cloud
    point_cloud_rgb_signal_->operator()(cloud);
    //(*point_cloud_rgb_signal_)(cloud);
  }
  
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > KinectSdkGrabber::assembleSphere () const
  {
    static int count = 0;
    ++count;
    float radius = (3.0 + sin (count * 0.1)) * 0.25 * radius_;
    // generate a new cloud and publish it
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(new pcl::PointCloud<pcl::PointXYZRGB > ());

    cloud->header.frame_id = "sphere";
    cloud->height = latitudes_;
    cloud->width = longitudes_;
    cloud->is_dense = true;
    cloud->points.resize(cloud->height * cloud->width);
    
    float long_delta = 2.0 * M_PI / (float)longitudes_;
    float lat_delta = M_PI / (float)latitudes_;
    
    float lat_angle = - M_PI * 0.5;
    unsigned pIdx = 0;
    
    RGBValue color;
    for (unsigned latIdx = 0; latIdx < latitudes_; ++latIdx, lat_angle += lat_delta)
    {
      float long_angle = 0;
      for (unsigned longIdx = 0; longIdx < longitudes_; ++longIdx, ++pIdx, long_angle += long_delta)
      {
        cloud->points[pIdx].x = radius * cos (long_angle) * cos (lat_angle) + center_x_;
        cloud->points[pIdx].y = radius * sin (long_angle) * cos (lat_angle) + center_y_;
        cloud->points[pIdx].z = radius * sin (lat_angle) + center_z_;
        color.Red = 255;
        color.Blue = sin (long_angle + count * 0.1) * 127 + 127;
        color.Green = sin (lat_angle + count * 0.1) * 127 + 127;
        cloud->points[pIdx].rgb = color.float_value;
      }
    }
    
    return cloud;
  }
}
