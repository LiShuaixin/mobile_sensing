#ifndef SCAN_H
#define SCAN_H

#include <vector>
#include <list>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>

#include <pcl/point_cloud.h>                        
#include <pcl/filters/voxel_grid.h>            
#include <pcl/filters/filter.h>      
#include <pcl/kdtree/kdtree_flann.h> // 此头文件不能置于opencv之后
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <boost/make_shared.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <rich_point.h>
#include <scan_projection.h>
#include <utils.h>
#include <projection_params.h>
#include <spanpose.h>

namespace Mobile_Sensing
{
  
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class ProjectionParams;
class ScanProjection;

// Frame class for point cloud preprocess
class Scan
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    Scan ();
    
    Scan (const Scan& scan);
    
    Scan (PointCloud::Ptr velo_cloud, const long long& timestamp, ProjectionParams* proj_params);
      
    void search_pose(const std::vector<SPANPose> poses);
    
    void removenan();
    
    void calculate_real_stamps();
    
    void initial_projection();
    
    void set_start_time( long long& start_time );
    
    void set_pose( const Eigen::Matrix3f& Rwc, const Eigen::Vector3f& twc );
    
    void set_pose( const Eigen::Matrix4f& Twc );
    
    void set(Scan& scan);
    
    
    
    inline PointCloud::Ptr get_full_cloud() { return cloud_; }
    inline Eigen::Matrix4f get_pose() { return trans_matrix(Rwc_, twc_); }  
    inline void push_back(const RichPoint& point) { points_.push_back(point); }
    inline PointCloud::Ptr get_cloud() 
    { 
	// we donot erase point from pointcloud, since the erasing of point is too slow, 
	// which is more than 100 times slower than create a new pointcloud pointer
	PointCloud tmp;
	std::list<size_t>::const_iterator iter_mask = mask_.begin();
	
	for( int idx_point = 0; idx_point < cloud_->points.size(); ++idx_point )
	{
	    if( idx_point == int(*iter_mask) )
	    {
		// printf("idx_point = %d, and iter_mask = %d, so the point is masked.\n", idx_point, int(*iter_mask));
		iter_mask++;
		
		continue;
	    }
	    
	    PointT point = cloud_->points[idx_point];
	    tmp.push_back(point);
	}	

	return tmp.makeShared(); 
    }
    
public:       
    RichPoint::AlignedVector points_;
    PointCloud::Ptr cloud_;
    std::list<size_t> mask_;
    
    ScanProjection* scan_projection_;
    
    Eigen::Vector3f twc_; // IMU center
    Eigen::Matrix3f Rwc_; 
    
    
    long long start_time_;
    std::vector<long long> timestamp_; // real timestamps for each point
    
    std::vector<float> azimuth_; // azimuth for each point 
    std::vector<float> vertical_; // vertical for each point  
};

}

#endif