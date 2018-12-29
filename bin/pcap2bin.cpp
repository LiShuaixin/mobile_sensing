/**
 * \brief 系统的输入数据与KITTI数据相同，imu/gps数据与相机/velodyne数据一一对应，
 * 一次测量数据单独保存为一个txt/bin文件. 
 * \func 本程序实现将一个点云pcap文件转为KITTI的bin文件结构
 */
#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/pcd_io.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_camera/openni_driver.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <typeinfo>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::visualization;

#define SHOW_FPS 0
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = getTime ();\
    double now = getTime (); \
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointType>
class SimpleVLPViewer
{
  public:
    typedef PointCloud<PointType> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef typename Cloud::Ptr CloudPtr;

    SimpleVLPViewer (Grabber& grabber,
                     PointCloudColorHandler<PointType> *handler) :
        cloud_viewer_ (new PCLVisualizer ("PCL VLP Cloud")),
        grabber_ (grabber),
        handler_ (handler)
    {
    }

    void
    cloud_callback (const CloudConstPtr& cloud)
    {
      FPS_CALC("cloud callback");
      boost::mutex::scoped_lock lock (cloud_mutex_);
      cloud_ = cloud;
    }

    void
    keyboard_callback (const KeyboardEvent& event,
                       void* /*cookie*/)
    {
      if (event.keyUp ())
      {
        switch (event.getKeyCode ())
        {
          case '0':
            delete handler_;
            handler_ = new PointCloudColorHandlerCustom<PointXYZI> (255, 255, 255);
            break;
          case '1':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("x");
            break;
          case '2':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("y");
            break;
          case '3':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("z");
            break;
          case '4':
            delete handler_;
            handler_ = new PointCloudColorHandlerGenericField<PointXYZI> ("intensity");
            break;
          case 'a':
            cloud_viewer_->removeAllCoordinateSystems ();
            cloud_viewer_->addCoordinateSystem (1.0, "global");
            break;
          case 'A':
            cloud_viewer_->removeAllCoordinateSystems ();
            break;
        }
      }
    }

    void
    run ()
    {
      cloud_viewer_->addCoordinateSystem (1.0, "global");
      cloud_viewer_->setBackgroundColor (0, 0, 0);
      cloud_viewer_->initCameraParameters ();
      cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
      cloud_viewer_->setCameraClipDistances (0.0, 50.0);
      cloud_viewer_->registerKeyboardCallback (&SimpleVLPViewer::keyboard_callback, *this);

      boost::function<void
      (const CloudConstPtr&)> cloud_cb = boost::bind (&SimpleVLPViewer::cloud_callback, this, _1);
      boost::signals2::connection cloud_connection = grabber_.registerCallback (cloud_cb);

      grabber_.start ();
  
      ofstream out_timestamps( "timestamps.txt", ios::out );
      int count = 0;
      while (!cloud_viewer_->wasStopped ())
      {
        CloudConstPtr tmp, cloud;

        if (cloud_mutex_.try_lock ())
        {
          cloud_.swap (cloud);
          cloud_mutex_.unlock ();
        }

        if (cloud)
        {
          pcl::PointCloud<pcl::PointXYZI> c = *cloud;
          pcl::PointCloud<pcl::PointXYZI>::Ptr t = c.makeShared();
          
          char tmp[256];
          std::sprintf(tmp, "%010d.bin", count);
          string out_pts_file(tmp);
          ofstream out_pts( out_pts_file, ios::out | ios::binary );
      
          //-- get time stamp and write timestamps.txt
          unsigned int timestamp_upper = cloud->header.stamp >> 32; // time()
          unsigned int timestamp_lower = cloud->header.stamp & 0xffffffff;  // microseconds from the top of the hour
          out_timestamps << std::to_string( timestamp_lower ) << '\n';

          //-- get pointcloud and write pts.bin
          for(int i = 0; i < t->size(); i++)
          {
            pcl::PointXYZI point = t->points[i];
            float px = point.x;
            float py = point.y;
            float pz = point.z;
            float pr = point.intensity;
            out_pts.write((char *)(&px), sizeof(px));
            out_pts.write((char *)(&py), sizeof(py));
            out_pts.write((char *)(&pz), sizeof(pz));
            out_pts.write((char *)(&pr), sizeof(pr));

          }

          out_pts.close();

          FPS_CALC("drawing cloud");
          handler_->setInputCloud (cloud);
          if (!cloud_viewer_->updatePointCloud (cloud, *handler_, "VLP"))
            cloud_viewer_->addPointCloud (cloud, *handler_, "VLP");

          cloud_viewer_->spinOnce ();
          count++;
        }

        if (!grabber_.isRunning ())
          cloud_viewer_->spin ();

        boost::this_thread::sleep (boost::posix_time::microseconds (100));
      }

      grabber_.stop ();

      cloud_connection.disconnect ();
      out_timestamps.close();
    }

    boost::shared_ptr<PCLVisualizer> cloud_viewer_;
    boost::shared_ptr<ImageViewer> image_viewer_;

    Grabber& grabber_;
    boost::mutex cloud_mutex_;
    boost::mutex image_mutex_;

    CloudConstPtr cloud_;
    PointCloudColorHandler<PointType> *handler_;
};

void
usage (char ** argv)
{
  cout << "usage: " << argv[0] << " [-pcapFile <path-to-pcap-file>] [-h | --help]" << endl;
  cout << argv[0] << " -h | --help : shows this help" << endl;
  return;
}

int
main (int argc,
      char ** argv)
{
  std::string pcapFile;

  if (find_switch (argc, argv, "-h") || find_switch (argc, argv, "--help"))
  {
    usage (argv);
    return (0);
  }

  parse_argument (argc, argv, "-pcapFile", pcapFile);

  VLPGrabber* grabber;
  grabber = new VLPGrabber(pcapFile);

  PointCloudColorHandlerGenericField<PointXYZI> *color_handler = new PointCloudColorHandlerGenericField<PointXYZI> ("intensity");

  SimpleVLPViewer<PointXYZI> v (*grabber, color_handler);
  v.run ();

  return (0);
}

