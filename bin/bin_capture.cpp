/**
 * \brief 系统的输入数据与KITTI数据相同，imu/gps数据与相机/velodyne数据一一对应，
 * 一次测量数据单独保存为一个txt文件. 
 * \func 本程序实现bin文件的可视化功能，在数据处理钱可预先浏览点云数据
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>
#include <string>
#include <stdio.h>  
#include <stdlib.h>  

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/time.h> //fps calculations
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>            
#include <pcl/filters/filter.h>  

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

int main( int argc, char* argv[] )
{
    PointCloudColorHandlerGenericField<PointXYZI> *color_handler = new PointCloudColorHandlerGenericField<PointXYZI> ("intensity");
    boost::shared_ptr<PCLVisualizer> cloud_viewer_(new PCLVisualizer ("PCL VLP Cloud"));
    std::cout << "create viwer done" << std::endl;
    
    cloud_viewer_->addCoordinateSystem (1.0, "global");
    cloud_viewer_->setBackgroundColor (0, 0, 0);
    cloud_viewer_->initCameraParameters ();
    cloud_viewer_->setCameraPosition (0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
    cloud_viewer_->setCameraClipDistances (0.0, 50.0);
    
    if(argc != 3)
    {
        std::cout << "Usage:: ./bin filepath/ timestamp.txt" << std::endl;
	return -1;
    }

    std::string filepath(argv[1]); // filepath of .bin files
    std::string timestamps(argv[2]);
    
    /* read file */
    ifstream ftime;
    FILE *fPCD;
    int count = 0;
    ftime.open(timestamps.c_str());
    
    while(!ftime.eof())
    {
	/* allocate 4 MB buffer */
	int32_t numPoints = 1000000; // the number of point clouds
	float *data = (float*)malloc(numPoints * sizeof(float));
	
	
	/* pointers */
	float *px = data + 0; // point to the first entry of data
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;
	
	char tmp[256];
	std::sprintf(tmp, "%010d.bin", count);
	std::string filename(tmp);
	
	std::string s;
	std::getline(ftime,s);
	
	if(!s.empty())
	{
	    pcl::PointCloud<PointXYZI>::Ptr m(new pcl::PointCloud<PointXYZI>());
	    
	    std::string currFilenameBinary = filepath + filename;
	    
	    if((fPCD = fopen(currFilenameBinary.c_str(), "rb")) == NULL ){
		    std::cout << "Load binary file failed!" << std::endl;
	    }
	    numPoints = fread(data, sizeof(float), numPoints, fPCD)/4; // read file from fPCD file and save in the data pointer
	    
	    for(int32_t i = 0; i < numPoints; i++)
	    {
		pcl::PointXYZI point;
		point.x = *px;
		point.y = *py;
		point.z = *pz;
		point.intensity = *pr;
		
		m->push_back(point);
		
		px += 4;
		py += 4;
		pz += 4;
		pr += 4;
	    }
	    std::fclose(fPCD);
	    
	    //从点云中移除NAN点也就是无效点
	    std::vector<int> indices;
	    pcl::removeNaNFromPointCloud(*m,*m, indices);
	    // std::cout << "The start time is: " << m.stime << ", the end time is: " << m.etime << ", the forward time is: " << m.time << std::endl;
	    // std::cout << "The size of point cloud is: " << m.cloud->points.size() << std::endl;
	    
	    // visualize
	    FPS_CALC("drawing cloud");
	    color_handler->setInputCloud (m);
	    if (!cloud_viewer_->updatePointCloud (m, *color_handler, "VLP"))
	        cloud_viewer_->addPointCloud (m, *color_handler, "VLP");

	    cloud_viewer_->spinOnce ();
	}
	
	boost::this_thread::sleep (boost::posix_time::microseconds (100));
	count++;
    }
    
    return (0);
}
